"""
Distributed under the MIT license.
Copyright (c) 2018 Morse Micro - Julius Baxter (julius@morsemicro.com)

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

---

This modules provides a class which implements the command-level abstraction to
an SDIO interface.

There's probably plenty of behaviour it doesn't support, but it'll do the basics
to model an SDIO 2.0 host interface.

All spec references are to the simplified SD specifications.

"""
import os
import sys
import logging
import random
import copy

import cocotb
from cocotb.result import TestFailure, TestSuccess, ReturnValue
from cocotb.clock import Clock
from cocotb.triggers import Timer, FallingEdge, RisingEdge, ReadOnly
from cocotb.drivers import BusDriver
from cocotb.binary import BinaryValue
from cocotb.log import SimLog

import sdio_utils

class SDIOProtocolError(Exception):
    pass

class SDIOResponseError(Exception):
    pass

class SDIODataError(Exception):
    pass

class SDIOHost(object):
    """
    A SDIO host driver (not complete) according to the Simplified (Open Source) SDIO and SD specs.
    """
    def __init__(self, clock, phy, spi_mode=False):
        self.log = SimLog("sdio_host")
        self.log.setLevel(logging.INFO)

        # We're we're meant to be in SPI mode or not
        self.spi_mode = spi_mode
        
        self.phy = phy
        self.clock = clock

        self.init_state()

    def init_state(self):
        """
        Variables to set up during initialization
        """
        self.rca = None
        # Bits in register 8 of the CCCR region
        self.smb = False # Support multiple block transfer
        self.sdc = False # Support direct command (CMD52)
        self.srw = False # Support read wait
        self.sbs = False # Support bus control (suspend/resume)
        self.s4mi = False # Support block gap interrupt (card generates interrupts between gaps of 4-bit data)
        self.lsc = False # Low-speed card, else it's fullspeed
        self.b4ls = False # 4-bit low speed card support

        self.fn_cis_addrs = [0]*8
        self.fn_max_blocksizes = [0]*8
        self.fn_count = 1
        self.cis_data = []

    @cocotb.coroutine
    def get_cmd_response(self,cmd,timeout=1000,timeout_possible=False):
        """
        Await the response from the host to the command we sent,
        and check a number of standard things in it.
        """
        cmd_num = cmd[45:40].integer
        if self.spi_mode:
            response_type, response_length = sdio_utils.get_spi_response_type(cmd_num)
        else:
            response_type, response_length = sdio_utils.get_response_type(cmd_num)
        response = yield self.phy.get_cmd_response_bits(cmd,timeout,timeout_possible)

        if response is "timeout" and timeout_possible:
            # Just return the timeout indication
            raise ReturnValue(response)

        if self.spi_mode:
            # No CRC on command responses
            if response_type in [4,7]:
                r1_offset = 32
            elif response_type == 5:
                r1_offset = 8
            else:
                r1_offset = 0
            self.log.debug("Getting R%d from CMD%d, data: %s",response_type,cmd_num,response.binstr)
            # Check the R1 status fields
            if response[r1_offset + 7]:
                for _ in range(0,4): yield FallingEdge(self.clock)
                raise SDIOResponseError("SPI command cmd%d response indicated parameter error (R1:%02x)" %(cmd_num,response.integer))
            if response[r1_offset + 4]:
                for _ in range(0,4): yield FallingEdge(self.clock)
                raise SDIOResponseError("SPI command cmd%d response indicated function number error (R1:%02x)" %(cmd_num,response.integer))
            if response[r1_offset + 3]:
                for _ in range(0,4): yield FallingEdge(self.clock)
                raise SDIOResponseError("SPI command cmd%d response indicated CRC error (R1:%02x)" %(cmd_num,response.integer))
            if response[r1_offset + 2]:
                for _ in range(0,4): yield FallingEdge(self.clock)
                raise SDIOResponseError("SPI command cmd%d response indicated illegal instruction error (R1:%02x)" %(cmd_num,response.integer))
            raise ReturnValue(response)

        else:
            # Do response checks
            # Check the CRC7
            crc7 = sdio_utils.crc7_gen(number=response[47:8].integer)
            if crc7 != response[7:1].integer:
                for _ in range(0,4): yield FallingEdge(self.clock)
                raise SDIOProtocolError("Response CRC7 error: in response to cmd %d, expected CRC of %x got %x, response was 'b%s" %(cmd_num, crc7,
                                                                                                                                   response[7:1].integer,
                                                                                                                                   response.binstr))
            response_cmd_num = response[45:40].integer
            if response_type in [4]:
                if response_cmd_num != 0x3f:
                    raise SDIOProtocolError("R4 reserved field [45:40] were not all set to 1, instead they were %x" %response_cmd_num)
            elif response_cmd_num != cmd_num: # Other commands need to have their command number reflected here
                for _ in range(0,4): yield FallingEdge(self.clock)
                raise SDIOProtocolError("Response cmd num error: in response to cmd %d, cmd field had %d" %(cmd_num,response_cmd_num))
            if response_type in [1,1.5]:
                # Could be a busy if 1.5, else it'll be type, let's look at the card status/
                # We only care about a few bits, as specified in the SDIO spec 4.10.8 (page 23)
                card_status = BinaryValue(value=response[39:8].integer,bits=32,bigEndian=False)
                self.log.debug("R1 card status register: %x ('b%s)" %(card_status.integer, card_status.binstr))
                if card_status[31]:
                    for _ in range(0,4): yield FallingEdge(self.clock)
                    raise SDIOResponseError("Card status register bit 31, OUT_OF_RANGE, set in response to cmd%d" %(cmd_num))
                if card_status[23]:
                    for _ in range(0,4): yield FallingEdge(self.clock)
                    raise SDIOResponseError("Card status register bit 23, COM_CRC_ERROR, set in response to cmd%d" %(cmd_num))
                if card_status[22]:
                    for _ in range(0,4): yield FallingEdge(self.clock)
                    raise SDIOResponseError("Card status register bit 22, ILLEGAL_COMMAND (not legal for current state), set in response to cmd%d" %(cmd_num))
                if card_status[19]:
                    for _ in range(0,4): yield FallingEdge(self.clock)
                    raise SDIOResponseError("Card status register bit 19, ERROR (general or unknown error), set in response to cmd%d" %(cmd_num))
                if card_status[12:9].integer != 0xf:
                    for _ in range(0,4): yield FallingEdge(self.clock)
                    raise SDIOResponseError("Card status register CURRENT_STATE != 0xf, which it should be for an SDIO card")
        raise ReturnValue(response)

    @cocotb.coroutine
    def cmd_go_idle(self):
        """
        Send a CMD0, or GO_IDLE_STATE.
        According to section 4.4 of the SDIO spec:
          "Note that in SD mode, CMD0 is only used to indicated entry into SPI mode and shall be supported.
           An I/O only card or the I/O portion of a combo card is not reset by CMD0"

        """
        cmd = sdio_utils.init_cmd(cmd_num=0)
        yield self.phy.acquire_cmd_lock()
        yield self.phy.send_cmd(cmd)
        if self.spi_mode:
            response = yield self.get_cmd_response(cmd)
            self.phy.release_cmd_lock()
            self.log.debug("Response: 'b%s" %response.binstr)
        else:
            self.phy.release_cmd_lock()

    @cocotb.coroutine
    def cmd_send_relative_addr(self):
        """
        Send a CMD3, or SEND_RELATIVE_ADDR.
        Asks the card to publish a new relative address (RCA). Contents are empty.

        """
        cmd = sdio_utils.init_cmd(cmd_num=3)
        yield self.phy.acquire_cmd_lock()
        yield self.phy.send_cmd(cmd)
        response = yield self.get_cmd_response(cmd)
        self.phy.release_cmd_lock()
        self.log.debug("Response: 'b%s" %response.binstr)
        raise ReturnValue(response)

    @cocotb.coroutine
    def cmd_send_if_cond(self):
        """
        Send a CMD8
        """
        cmd = sdio_utils.init_cmd(cmd_num=8)
        # Bits 19:16 are the VHS (table 4-8 SD Spec 6.00)
        cmd[16] = 1 # 19:16 == 3'b001 means 2.7-3.6V voltage, which is right for 2.0
        cmd[15:8] = random.getrandbits(8) # This is an 8-bit pattern which is echoed back, set it to something random
        yield self.phy.acquire_cmd_lock()
        yield self.phy.send_cmd(cmd)
        response = yield self.get_cmd_response(cmd)
        self.phy.release_cmd_lock()
        self.log.debug("Response: 'b%s" %response.binstr)
        raise ReturnValue(response)

    @cocotb.coroutine
    def cmd_send_op_cond(self):
        """
        Send a CMD5
        Similar to the operation of ACMD41 for SD memory cards, it is used to inquire about the
        voltage range needed byt he I/O card.
        """
        cmd = sdio_utils.init_cmd(cmd_num=5)
        # Hardcode to support only around 3v3 (see CMD5 in SDIO spec)
        cmd[8+18] = 1;
        cmd[8+19] = 1;
        cmd[8+20] = 1;
        cmd[8+21] = 1;
        cmd[8+22] = 1;
        yield self.phy.acquire_cmd_lock()
        yield self.phy.send_cmd(cmd)
        response = yield self.get_cmd_response(cmd)
        self.phy.release_cmd_lock()
        self.log.debug("Response: 'b%s" %response.binstr)
        raise ReturnValue(response)


    @cocotb.coroutine
    def cmd_select_card(self, rca=0):
        """
        Send a CMD7, or SELECT_CARD.
        Toggle a card between stand-by and transfer states. Card is selected by its own relative address, and is de-selected by
        any other address.

        """
        cmd = sdio_utils.init_cmd(cmd_num=7)
        cmd[39:24] = rca
        yield self.phy.acquire_cmd_lock()
        yield self.phy.send_cmd(cmd)
        self.log.debug("CMD select card: 'b%s" %cmd.binstr)
        response = yield self.get_cmd_response(cmd)
        self.phy.release_cmd_lock()
        self.log.debug("Response: 'b%s" %response.binstr)
        raise ReturnValue(response)

    @cocotb.coroutine
    def cmd_io_rw_direct(self, rw=0, fn=0, raw=0, addr=0, data=None, timeout_possible=False):
        """
        Send a CMD52, IO_RW_DIRECT command
        It's the simplest form of register access within 128k of space (17-bits address).
        Details in section 5.1 of SDIO spec (page 25)

        args:
        rw - 0: read, 1: write
        raw - read after write
        address - not wider than 17-bits
        data - not wider than 8-bits

        """

        cmd = sdio_utils.init_cmd(cmd_num=52)
        cmd[39] = rw
        cmd[38:36] = fn
        cmd[35] = raw
        cmd[33:17] = addr
        if rw:
            assert(data != None)
            cmd[15:8] = data
        yield self.phy.acquire_cmd_lock()
        yield self.phy.send_cmd(cmd)
        response = yield self.get_cmd_response(cmd,timeout_possible=timeout_possible)
        self.phy.release_cmd_lock()

        if response is "timeout" and timeout_possible:
            raise ReturnValue(response)

        if self.spi_mode:
            response_data = response[7:0].integer
        else:            
            # Inspect the response flags here
            response_flags = BinaryValue(value=response[23:16].integer,bits=8,bigEndian=False)

            if response_flags[7]:
                for _ in range(0,4): yield FallingEdge(self.clock)
                raise SDIOResponseError("IO_RW_DIRECT response flags: COM_CRC_ERROR")
            if response_flags[6]:
                for _ in range(0,4): yield FallingEdge(self.clock)
                raise SDIOResponseError("IO_RW_DIRECT response flags: ILLEGAL_COMMAND")
            if response_flags[3]:
                for _ in range(0,4): yield FallingEdge(self.clock)
                raise SDIOResponseError("IO_RW_DIRECT response flags: ERROR")
            if response_flags[1]:
                for _ in range(0,4): yield FallingEdge(self.clock)
                raise SDIOResponseError("IO_RW_DIRECT response flags: FUNCTION_NUMBER (invalid function number %d)" % fn)
            if response_flags[0]:
                for _ in range(0,4): yield FallingEdge(self.clock)
                raise SDIOResponseError("IO_RW_DIRECT response flags: OUT_OF_RANGE")

            # TODO - this always responds 0 why?!
            self.log.debug("IO_RW_DIRECT response flags, IO_CURRENT_STATE: %d" %(response_flags[5:4].integer))

            response_data = response[15:8].integer

        raise ReturnValue(response_data)

    @cocotb.coroutine
    def cmd_io_rw_extended(self, rw=0, fn=0, block=0, op=0, addr=0, count=0, data=None, blocksize=None, read_wait=None, could_abort=True):
        """
        Send a CMD53, IO_RW_EXTENDED command
        This is a data command and allows reading/writing to multiple address spaces with a single command.
        Details in section 5.3 of SDIO spec (page 28)

        args:
        rw - 0: read, 1: write
        fn - the function to access
        block - whether we're doing a transfer in blocks or bytes
                Block may or may not be supported, SMB bit in CCCR indicates so
        op - 0: multi byte R/W to a fixed address, 1: to an incrementing address
        address - not wider than 17-bits
        count - depending on the mode it has different meanings
                block == 1 ? 0: infinite (keep going until we sent an abort) 1: 1 block, 2: 2 blocks, etc.
                block == 0 ? 0: 512 bytes , 1: 1 byte, 2: 2 bytes etc.
        data - a list of data values (each not wider than 8-bits)

        """
        cmd        = sdio_utils.init_cmd(cmd_num=53)
        cmd[39]    = rw
        cmd[38:36] = fn
        cmd[35]    = block
        cmd[34]    = op
        cmd[33:17] = addr
        cmd[16:8]  = count

        yield self.phy.acquire_cmd_lock()
        yield self.phy.send_cmd(cmd)
        response = yield self.get_cmd_response(cmd)
        self.phy.release_cmd_lock()

        if self.spi_mode:
            pass
        else:
            # Inspect the response flags here
            response_flags = BinaryValue(value=response[23:16].integer,bits=8,bigEndian=False)

            if response_flags[7]:
                for _ in range(0,4): yield FallingEdge(self.clock)
                raise SDIOResponseError("IO_RW_DIRECT response flags: COM_CRC_ERROR")
            if response_flags[6]:
                for _ in range(0,4): yield FallingEdge(self.clock)
                raise SDIOResponseError("IO_RW_DIRECT response flags: ILLEGAL_COMMAND")
            if response_flags[3]:
                for _ in range(0,4): yield FallingEdge(self.clock)
                raise SDIOResponseError("IO_RW_DIRECT response flags: ERROR")
            if response_flags[1]:
                for _ in range(0,4): yield FallingEdge(self.clock)
                raise SDIOResponseError("IO_RW_DIRECT response flags: FUNCTION_NUMBER (invalid function number %d)" % fn)
            if response_flags[0]:
                for _ in range(0,4): yield FallingEdge(self.clock)
                raise SDIOResponseError("IO_RW_DIRECT response flags: OUT_OF_RANGE")

            # TODO - this always responds 0 why?!
            self.log.debug("IO_RW_EXTENDED response flags, IO_CURRENT_STATE: %d" %(response_flags[5:4].integer))

        if block:
            blocks = count
        else:
            blocks = 1

        if rw:
            assert(data != None)
            for blockcount in range(0,blocks):
                # Let's give a random number of bytes of clocks
                random_pad_bytes = random.randint(1,4)
                for _ in range(0,random_pad_bytes*8):
                    yield RisingEdge(self.clock)

                if block:
                    self.log.debug("Writing block %d" %blockcount)
                    # Check if the block write has been aborted
                    if self.phy.data_write_aborted:
                        self.phy.data_write_aborted = False
                        self.log.info("Detected block write aborted after %d blocks" %(blockcount))
                        raise ReturnValue(0)
                bytes_to_write = data[blockcount] if block else data
                # Send data bytes on the data lines
                crc_resp = yield self.phy.data_bus_write(bytes_to_write,could_abort=could_abort,final_block=blockcount+1==blocks)
                # TODO check the CRC response and do something sensible with it
                
        else:
            blockdata = []
            for blockcount in range(0,blocks):
                if block:
                    assert(blocksize != None), "Called with block=1 but blocksize was not passed, please call with blocksize set"
                bytes_to_read = blocksize if block else count
                if block:
                    self.log.debug("Reading block %d" %blockcount)
                data,status = yield self.phy.data_bus_read(bytes_to_read,could_abort=could_abort,final_block=blockcount+1==blocks)
                if status == "aborted" or self.phy.data_read_aborted:
                    self.phy.data_read_aborted = False
                    # We were an aborted read, return what we have
                    raise ReturnValue(data)
                if block:
                    blockdata.append(data)
                    # Wait a little bit before polling for the next data block
                    if self.spi_mode:
                        for _ in range(0,8):
                            yield RisingEdge(self.clock)
                    else:
                        # Wait a few cycles before polling the bus for the next block of data
                        yield FallingEdge(self.clock)
                        yield FallingEdge(self.clock)
                # If we've been passed a read-wait period, do that
                if read_wait:
                    self.phy.read_wait(True)
                    for x in range(read_wait):
                        yield FallingEdge(self.clock)
                    self.phy.read_wait(False)

            if block:
                raise ReturnValue(blockdata)
            else:
                raise ReturnValue(data)

        raise ReturnValue(0)

    @cocotb.coroutine
    def sdio_init(self, reset=False, dump_regs=False, rca_changes=0):
        """
        Run the full SDIO initialization sequence.

        """
        self.log.info("Beginning SDIO device initialization")
        if self.spi_mode:
            pass # Add pre-init test commands here
        else:
            # Send a CMD52 to write to the RES bit of the CCCR, bit 3 of address 6
            reg = yield self.read_reg(fn=0,addr=sdio_utils.get_addr_by_name('I/O abort'),timeout_possible=True)
            if reg != "timeout":
                reg = BinaryValue(value=reg,bits=8,bigEndian=False)
                # Set the reset bit (RES)
                reg[3] = 1
                yield self.write_reg(fn=0,addr=sdio_utils.get_addr_by_name('I/O abort'),data=reg.integer)

                for _ in range(random.randint(8,16)): yield RisingEdge(self.clock)

        
        # Now send a cmd0
        yield self.cmd_go_idle()
        for _ in range(random.randint(8,16)): yield RisingEdge(self.clock)

        yield self.cmd_go_idle()
        for _ in range(random.randint(8,16)): yield RisingEdge(self.clock)

        # CMD8
        yield self.cmd_send_if_cond()
        for _ in range(random.randint(8,16)): yield RisingEdge(self.clock)

        # CMD5
        yield self.cmd_send_op_cond()
        for _ in range(random.randint(8,16)): yield RisingEdge(self.clock)

        if self.spi_mode:
            pass
        else:
            # Now CMD3
            response = yield self.cmd_send_relative_addr()
            self.log.info("SDIO device RCA response: %x ('b%s)" %(response[39:24].integer, response[39:24].binstr))
            self.rca = response[39:24].integer
            # Device should now be in standby state

            # Actually do that again a number of times to check that the RCA changes and we can still init with it
            for loop in range(rca_changes):
                # Reissue CMD3
                response = yield self.cmd_send_relative_addr()
                self.log.info("SDIO device RCA response: %x ('b%s)" %(response[39:24].integer, response[39:24].binstr))
                self.rca = response[39:24].integer

            # Now issue a CMD7 selecting the card
            response = yield self.cmd_select_card(self.rca)
            self.log.debug("SDIO response to select: %x ('b%s)" %(response[39:8].integer, response[39:8].binstr))

        # Read register 8
        reg = yield self.cmd_io_rw_direct(rw=0, fn=0, raw=0, addr=sdio_utils.get_addr_by_name('Card capability'))
        reg = BinaryValue(value=reg,bits=8,bigEndian=False)

        self.sdc = reg[0].integer # Support direct command (CMD52)
        self.smb = reg[1].integer # Support multiple block transfer
        self.srw = reg[2].integer # Support read wait
        self.sbs = reg[3].integer # Support bus control (suspend/resume)
        self.s4mi = reg[4].integer # Support block gap interrupt (card generates interrupts between gaps of 4-bit data)
        self.lsc =  reg[6].integer # Low-speed card, else it's fullspeed
        self.b4ls = reg[7].integer # 4-bit low speed card support

        self.log.info("Card capability register:")
        self.log.info("  SDC - support direct command: %s" %(1 if self.sdc else 0))
        self.log.info("  SMB - support multiple block transfer : %s" %(1 if self.smb else 0))
        self.log.info("  SRW - support read wait: %s" %(1 if self.srw else 0))
        self.log.info("  SBS - support bus control (suspend/resume): %s" %(1 if self.sbs else 0))
        self.log.info("  S4MI - support 4-bit block gap interrupt: %s" %(1 if self.s4mi else 0))
        self.log.info("  LSC - is a low-speed card, if not it's high-speed: %s" %(1 if self.lsc else 0))
        self.log.info("  4BLS - support 4-bit low-speed: %s" %(1 if self.b4ls else 0))

        # Store the function card information structure (CIS) address in variables
        self.fn_cis_addrs[0] = yield self.read_reg(fn=0, addr=sdio_utils.get_addr_by_name('Common CIS pointer byte 0'))
        self.fn_cis_addrs[0] |= (yield self.read_reg(fn=0, addr=sdio_utils.get_addr_by_name('Common CIS pointer byte 1'))) << 8
        self.fn_cis_addrs[0] |= (yield self.read_reg(fn=0, addr=sdio_utils.get_addr_by_name('Common CIS pointer byte 2'))) << 16
        self.log.debug("Function 0 CIS address: 0x%x" %self.fn_cis_addrs[0])

        # Do a data ready to get the CIS, much faster, assume 256 is enough
        self.cis_data = yield self.cmd_io_rw_extended(rw=0, fn=0, block=0, op=0, addr=self.fn_cis_addrs[0], count=256)
        self.log.debug("CIS data: %s" %self.cis_data)

        # Check how many functions we have
        for fn in range(1,8):
            # Read the FBR0 and if it's 0 there's no SDIO function there
            reg = yield self.read_reg(fn=0, addr=sdio_utils._cia_base_addresses['FBR%d'%fn])
            if reg & 0xf:
                self.log.debug("Function %d detected as present" %fn)
                self.fn_count += 1
                self.fn_cis_addrs[fn] = yield self.read_reg(fn=0, addr=sdio_utils._cia_base_addresses['FBR%d'%fn] + sdio_utils.get_addr_by_name('Function CIS pointer byte 0'))
                self.fn_cis_addrs[fn] |= (yield self.read_reg(fn=0, addr=sdio_utils._cia_base_addresses['FBR%d'%fn] + sdio_utils.get_addr_by_name('Function CIS pointer byte 1'))) << 8
                self.fn_cis_addrs[fn] |= (yield self.read_reg(fn=0, addr=sdio_utils._cia_base_addresses['FBR%d'%fn] + sdio_utils.get_addr_by_name('Function CIS pointer byte 2'))) << 16
                self.log.debug("Function %d CIS address: 0x%x" %(fn,self.fn_cis_addrs[fn]))

        if dump_regs:
            yield self.dump_cccrs()
            for fn in range(0,self.fn_count):
                self.dump_cis(self.fn_cis_addrs[fn])
                if fn > 0:
                    yield self.dump_fbrs(fn)

        # Get the function maximum block sizes
        # FN0 - get the upper block size for function 0 from its CIS table
        fn0_cis_tuples = self.parse_cis_tuple_table(self.fn_cis_addrs[0])
        found_size = False
        for tuple in fn0_cis_tuples:
            # See page 64 (section 16.7.3) in the SDIO spec for what I'm doing here
            if tuple[0] == 0x22:
                self.fn_max_blocksizes[0] = (tuple[4] << 8) | tuple[3]
                found_size = True
                break
        if not found_size:
            raise SDIODataError("Unable to determine Function 0's block size from its CIS table")
        else:
            self.log.info("SDIO function 0 max block size is %d bytes" %self.fn_max_blocksizes[0])

        # FN1-7 slightly different as their CIS pointers is in the respective FBRs
        for fn in range(1,self.fn_count):
            fn_cis_tuples = self.parse_cis_tuple_table(self.fn_cis_addrs[fn])
            found_size = False
            for tuple in fn_cis_tuples:
                # See page 65 (section 16.7.4) in the SDIO spec for what I'm doing here
                if tuple[0] == 0x22:
                    self.fn_max_blocksizes[fn] = (tuple[15] << 8) | tuple[14]
                    found_size = True
                    break
            if not found_size:
                raise SDIODataError("Unable to determine Function %d's block size from its CIS table" %fn)
            else:
                self.log.info("SDIO function %d max block size is %d bytes" %(fn,self.fn_max_blocksizes[fn]))

        self.log.info("SDIO Initialized")


    @cocotb.coroutine
    def dump_cccrs(self):
        """
        Print out all of the CCCRs (card common control registers)

        See section 6.9 (page 33) of the SDIO spec.
        """
        self.log.info("CCCRs:")
        for reg in sdio_utils._cccrs:
            val = yield self.read_reg(fn=0, addr=reg['addr'])
            if 'bin' in reg and reg['bin']:
                # Print out in binary format
                self.log.info("0x%02x %-30s: %s" %(reg['addr'], reg['name'], BinaryValue(value=val,bits=8,bigEndian=False).binstr))
            else:
                self.log.info("0x%02x %-30s: %02x" %(reg['addr'], reg['name'], val))

    @cocotb.coroutine
    def dump_fbrs(self, func=0):
        """
        Print out the FBR (function basic registers) for a particular function

        See section 6.9 (page 33) of the SDIO spec.
        """
        self.log.info("FBR for function %d:" % func)
        for reg in sdio_utils._fbrs:
            addr = (func << 8) + reg['addr']
            val = yield self.read_reg(fn=0, addr=addr)
            if 'bin' in reg and reg['bin']:
                # Print out in binary format
                self.log.info("0x%02x %-35s: %s" %(reg['addr'], reg['name'], BinaryValue(value=val,bits=8,bigEndian=False).binstr))
            else:
                self.log.info("0x%02x %-35s: %02x" %(reg['addr'], reg['name'], val))


    def dump_cis(self,addr=0):
        """
        Get the CIS tuples from the address and print them out
        """
        tuples = self.parse_cis_tuple_table(addr)
        if tuples:
            self.log.info("CIS tuples from address %x" %addr)
            for tuple in tuples:
                self.log.info("Tuple type: %02x, tuples: %s" %(tuple[0], ["%02x" %t for t in tuple[2:]])) # Skip the tuple size byte

    def parse_cis_tuple_table(self,addr=0):
        """
        Parse and return all of the tuples as a list of lists
        """
        self.log.debug("Parsing CIS section starting at 0x%x" % addr)
        link = -1
        byte_of_tuple = 0
        return_tuples = []
        current_tuple = []
        cis_offset = addr & 0xff
        for byte in self.cis_data[cis_offset:]:
            if link == 0 and byte_of_tuple > 0:
                # We're back to the beginning of a tuple
                byte_of_tuple = 0
                link -= 1  # Set this back to -1
                return_tuples.append(current_tuple)
                current_tuple = []

            #byte = yield self.read_reg(fn=0, addr=addr)
            current_tuple.append(byte)
            if byte_of_tuple == 0 and byte == 0xff:
                # Finish up
                break

            #self.log.info("Byte @ %04x: %02x (byte_of_tuple=)%d" %(addr,byte,byte_of_tuple))
            if byte_of_tuple == 1:
                if byte == 0:
                    raise SDIODataError("Reading CIS and got 0 for a tuple link value")
                # This is the link value
                link = byte + 1 # + 1 because we subtract 1 immediately at the end

            byte_of_tuple += 1
            addr += 1
            if link:
                link -= 1

            if byte_of_tuple > 100:
                raise SDIODataError("Tuple in CIS too long")
        #raise ReturnValue(return_tuples)
        return return_tuples

    @cocotb.coroutine
    def read_reg(self,fn,addr,timeout_possible=False):
        """
        Read an 8-bit register with the CMD52 guy

        See table 6-1 of the SDIO spec (page 33) for details on some of the CIA registers
        """
        response = yield self.cmd_io_rw_direct(rw=0, fn=fn, addr=addr, timeout_possible=timeout_possible)
        raise ReturnValue(response)

    @cocotb.coroutine
    def write_reg(self,fn,addr,data,timeout_possible=False):
        """
        Write an 8-bit register with the CMD52 guy

        See table 6-1 of the SDIO spec (page 33) for details on some of the CIA registers
        """
        response = yield self.cmd_io_rw_direct(rw=1, fn=fn, addr=addr, data=data, timeout_possible=timeout_possible)
        raise ReturnValue(response)

    @cocotb.coroutine
    def set_bus_width(self,width=4):
        """
        Set bus width of the SDIO device to 4
        """
        if width==4:
            value = BinaryValue(value=0x2,bits=2,bigEndian=False)
        elif width==1:
            value = BinaryValue(value=0x0,bits=2,bigEndian=False)
        else:
            self.log.warning("Bus width %d not supported, ignoring" %width)
            return

        reg_addr = sdio_utils.get_addr_by_name('Bus interface control')
        reg = yield self.read_reg(fn=0,addr=reg_addr)
        reg = BinaryValue(value=reg,bits=8,bigEndian=False)
        # Write the bus width fields
        reg[0] = value[0].integer
        reg[1] = value[1].integer
        yield self.write_reg(fn=0,addr=reg_addr,data=reg.integer)
        # Confirm it
        reg = yield self.read_reg(fn=0,addr=reg_addr)
        assert ((reg & 0x3) == value.integer)

        reg = yield self.read_reg(fn=0,addr=reg_addr)
        self.phy.bus_width = width # Set it in the phy model so it knows what to drive to the device
        self.log.debug("Bus interface control reg: %s" % BinaryValue(value=reg,bits=8,bigEndian=False))

    @cocotb.coroutine
    def enable_fn(self,fn=1):
        """
        Enable a function
        """
        reg_addr = sdio_utils.get_addr_by_name('I/O enables')
        reg = yield self.read_reg(fn=0,addr=reg_addr)
        reg |= 1 << fn
        yield self.write_reg(fn=0,addr=reg_addr,data=reg)

    @cocotb.coroutine
    def send_abort(self,fn=1):
        """
        Send abort to a function by writing tot he ASx bits (2:0) in CCCR reg 6
        """
        reg_addr = sdio_utils.get_addr_by_name('I/O abort')
        reg = yield self.read_reg(fn=0,addr=reg_addr)
        reg &= ~0x7
        reg |= (fn & 0x7)
        yield self.write_reg(fn=0,addr=reg_addr,data=reg)
        self.log.info("Abort CMD52 sent to fn%d at %sns" %(fn,cocotb.utils.get_sim_time(units='ns')))

    @cocotb.coroutine
    def set_block_size(self,fn,blocksize):
        """
        Set the transfer blocksize for a particular function in the CIA registers (CCCRs for FN0, FBRs for FN1-8)

        Only supports functions 0 and 1 for now

        The block size is set by writing the block size to the I/O block size register
        in the FBR (Table 6-3 and 6-4 in the SDIO spec). The block size for function 0 is set
        by writing FN0 Block Size Register in the CCCRs. The block size used and maximum byte count
        per command when block transfers are not being used (block=0) is specified in the CIS (card
        information structure) in the the tuple TPLFE_MAX_BLK_SIZE (section 16.7.4) on a per-function
        basis.

        FBR = function basic registers

        I/O block size is at offset 0xN10-0xN11 (two bytes) within each function's FBRs, which are
        at offset 0xN00 where N = function number, so for function 1, regs 0x110 and 0x111

        """
        if fn==0:
            assert blocksize <= self.fn0_max_blocksize, "Trying to set a block size (%d) for function 0 greater than it supports (%d)" %(blocksize,self.fn0_max_blocksize)
            fn0_blocksize_addr0 = sdio_utils._cia_base_addresses['CCCR'] + sdio_utils.get_addr_by_name('FN0 block size byte 0')
            yield self.write_reg(fn=0,addr=fn0_blocksize_addr0,data=blocksize & 0xff)
            yield self.write_reg(fn=0,addr=fn0_blocksize_addr0 + 1,data=((blocksize) >> 8) & 0xff)
        elif fn < self.fn_count:
            assert blocksize <= self.fn_max_blocksizes[fn], "Trying to set a block size (%d) for function %d greater than it supports (%d)" %(blocksize,fn,self.fn_max_blocksizes[fn])
            fn_blocksize_addr = sdio_utils._cia_base_addresses['FBR%d'%fn] + sdio_utils.get_addr_by_name('Function I/O block size byte 0')
            yield self.write_reg(fn=0,addr=fn_blocksize_addr,data=blocksize & 0xff)
            yield self.write_reg(fn=0,addr=fn_blocksize_addr + 1,data=((blocksize) >> 8) & 0xff)
        else:
            raise Exception("Cannot set block size on function %d as it doesn't exist!" %fn)
            raise ReturnValue(0)
        self.log.info("Block size of function %d set to %d" %(fn, blocksize))

    @cocotb.coroutine
    def soft_reset(self):
        """
        Send abort to a function by writing tot he ASx bits (2:0) in CCCR reg 6
        """
        reg_addr = sdio_utils.get_addr_by_name('I/O abort')
        reg = yield self.write_reg(fn=0,addr=reg_addr,data=1<<3)
        self.log.info("Soft reset CMD52 sent to device at %sns" %(cocotb.utils.get_sim_time(units='ns')))
        # We need to re-init now and so we need to reset some state
        self.init_state()

