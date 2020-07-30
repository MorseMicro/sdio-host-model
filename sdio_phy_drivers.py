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

This module contains two "drivers" which provide physical layer
controls and other useful functions used by the sdio_host module.

The CocotbSDIOHostDriver provides SDIO interface command and data
functions.

The CocotbSPIHostDriver provides the SPI-based version of the SDIO
protocol.

You can use them on their own but they're more useful integrated with
their higher-level protocol drivers in sdio_host.
"""

import cocotb
from cocotb.result import TestFailure, ReturnValue
from cocotb.triggers import FallingEdge, RisingEdge, Event
from cocotb.drivers import BusDriver
from cocotb.log import SimLog
from cocotb.binary import BinaryValue
from cocotb.utils import get_sim_time

import sdio_utils
import sdio_host

import random

class CocotbSDIOHostDriver(BusDriver):
    """
    A SDIO host driver for this testbench, driven directly from Cocotb
    """
    _signals = ["cmd_coco_dir", "cmd_coco_out", "cmd_coco_in",
                "data_coco_dir", "data_coco_out", "data_coco_in"]

    def __init__(self, entity, name, clock):
        BusDriver.__init__(self,entity, name, clock)
        self.bus.cmd_coco_dir.setimmediatevalue(0)
        self.bus.data_coco_dir.setimmediatevalue(0x00)
        self.bus.data_coco_out.setimmediatevalue(0xff)
        self.bus.cmd_coco_out.setimmediatevalue(1)
        self.bus_width = 1

        # Locks implemented with cocotb Events, normal threading.Lock guys don't work
        self.cmd_bus_busy_event = Event("cmd_bus_busy")
        self.cmd_bus_busy = False
        self.data_write_aborted = False
        self.data_read_aborted = False

    @cocotb.coroutine
    def acquire_cmd_lock(self):
        if self.cmd_bus_busy:
            yield self.cmd_bus_busy_event.wait()
        self.cmd_bus_busy_event.clear()
        self.cmd_bus_busy = True

    def release_cmd_lock(self):
        self.cmd_bus_busy = False
        self.cmd_bus_busy_event.set()
        
    @cocotb.coroutine
    def send_cmd(self, cmd):
        """
        Transmit a command, we will calculate the CRC and append it
        """
        crc7 = sdio_utils.crc7_gen(number=cmd[47:8].integer)
        cmd[7:1] = BinaryValue(value=crc7,bits=7,bigEndian=False).integer
        self.log.debug("SDIO host sending CMD%d: 'b%s" %(cmd[45:40].integer, cmd.binstr))
        self.bus.cmd_coco_dir <= 1
        for x in range(47,-1,-1):
            # Now shift this out bit by bit, host sends on falling edge
            yield FallingEdge(self.clock)
            self.bus.cmd_coco_out <= cmd[x].integer
            if x == 0:
                # Deassert the output enable onto the bus on the final bit
                self.bus.cmd_coco_dir <= 0

    @cocotb.coroutine
    def get_cmd_response_bits(self,cmd,timeout=1000,timeout_possible=False):
        """
        Read the response from the bus
        """
        cmd_num = cmd[45:40].integer
        response_type, response_length = sdio_utils.get_response_type(cmd_num)
        response_bit_counter = response_length
        response = BinaryValue(value=0,bits=response_length,bigEndian=False)
        if response_type == None:
            return
        timeout_count = 0
        while timeout_count < timeout:
            yield RisingEdge(self.clock)
            if response_bit_counter == response_length:
                # Response not started yet
                if self.bus.cmd_coco_in.value == 0:
                    # Start bit of response
                    response[response_bit_counter - 1] = int(self.bus.cmd_coco_in.value)
                    response_bit_counter -= 1
            elif response_bit_counter > 0:
                # Shift in the response bits
                response[response_bit_counter - 1] = int(self.bus.cmd_coco_in.value)
                response_bit_counter -= 1
            else:
                break
            timeout_count += 1
        if timeout_count == timeout:
            timeout_message = "Timeout waiting for response to cmd%d: %x ('b%s)" %(cmd_num, cmd.integer, cmd.binstr)
            if timeout_possible:
                self.log.info(timeout_message)
                raise ReturnValue("timeout")
            else:
                raise TestFailure(timeout_message)
        raise ReturnValue(response)

    @cocotb.coroutine
    def data_bus_read(self, count=1, timeout=None, could_abort=False, final_block=False):
        """
        Read the data bytes off the SD bus
        """
        data = []
        # Wait for the start bit
        timeout_count = 0
        timed_out = False
        timens_before = get_sim_time(units='ns') # Value we started at - SDIO timeout is 1ms, wait that long
        while BinaryValue(value=int(self.bus.data_coco_in),bits=8,bigEndian=False)[0] == 1:
            yield RisingEdge(self.clock)
            self.log.debug("Waited a clock for data bus bit 0 to be 0")
            if timeout:
                # Timeout in cycles has been specified
                timeout_count += 1 # I think it's allowed up to a second but in simulation let's just wait an infeasibly long time
                if timeout_count == timeout:
                    timed_out = True
            elif int(get_sim_time(units='ns') - timens_before) >= 1000000:
                # 1ms timeout
                timed_out = True

            if timed_out:
                if could_abort:
                    self.log.info("Timed out waiting for start bit of a block of data. Assuming aborted transfer.")
                    raise ReturnValue((data,"aborted"))
                else:
                    raise sdio_host.SDIODataError("Timeout waiting for start bit of data transfer")

        self.log.debug("SDIO host model data receive start bit detected")
        for byte in range(0,count):
            current_byte = BinaryValue(value=0,bits=8,bigEndian=False)
            if self.bus_width == 4:
                yield RisingEdge(self.clock)
                # WTF can't access this bus input thing without explicitly casting it as a BinaryValue.
                dbus = BinaryValue(value=int(self.bus.data_coco_in),bits=8,bigEndian=False)
                current_byte[7] = dbus[3].integer
                current_byte[6] = dbus[2].integer
                current_byte[5] = dbus[1].integer
                current_byte[4] = dbus[0].integer
                yield RisingEdge(self.clock)
                dbus = BinaryValue(value=int(self.bus.data_coco_in),bits=8,bigEndian=False)
                current_byte[3] = dbus[3].integer
                current_byte[2] = dbus[2].integer
                current_byte[1] = dbus[1].integer
                current_byte[0] = dbus[0].integer
            else:
                # 1-bit mode
                for bit in range(7,-1,-1):
                    yield RisingEdge(self.clock)
                    dbus = BinaryValue(value=int(self.bus.data_coco_in),bits=8,bigEndian=False)
                    current_byte[bit] = dbus[0].integer
            self.log.debug("SDIO host model received byte num %03d: %02x" %(byte,current_byte.integer))
            data.append(current_byte.integer)
            if could_abort and self.data_read_aborted:
                self.log.info("Detected data read aborted after %d bytes" %(byte))
                raise ReturnValue((data,"aborted"))


        # CRC time
        crc0 = BinaryValue(value=0,bits=16,bigEndian=False)
        crc1 = BinaryValue(value=0,bits=16,bigEndian=False)
        crc2 = BinaryValue(value=0,bits=16,bigEndian=False)
        crc3 = BinaryValue(value=0,bits=16,bigEndian=False)
        for bit in range(15,-1,-1):
            yield RisingEdge(self.clock)
            dbus = BinaryValue(value=int(self.bus.data_coco_in),bits=8,bigEndian=False)
            crc0[bit] = dbus[0].integer
            if self.bus_width == 4:
                crc1[bit] = dbus[1].integer
                crc2[bit] = dbus[2].integer
                crc3[bit] = dbus[3].integer
            if could_abort and self.data_read_aborted:
                self.log.info("Detected data read aborted during CRC, already received %d bytes" %(len(data)))
                raise ReturnValue((data,"aborted"))

        if self.bus_width == 4:
            num_bits_to_crc = count * 2
            # Calculate our CRC on the data
            d0,d1,d2,d3 = sdio_utils.crc16_array_prep(self.bus_width, data)
            crc16_d0 = BinaryValue(value=sdio_utils.crc16_gen(d0,num_bits_to_crc),bits=16,bigEndian=False)
            crc16_d1 = BinaryValue(value=sdio_utils.crc16_gen(d1,num_bits_to_crc),bits=16,bigEndian=False)
            crc16_d2 = BinaryValue(value=sdio_utils.crc16_gen(d2,num_bits_to_crc),bits=16,bigEndian=False)
            crc16_d3 = BinaryValue(value=sdio_utils.crc16_gen(d3,num_bits_to_crc),bits=16,bigEndian=False)
            self.log.debug("Received CRC for lane 0: %04x, calculated CRC: %04x" %(crc0.integer, crc16_d0.integer))
            self.log.debug("Received CRC for lane 1: %04x, calculated CRC: %04x" %(crc1.integer, crc16_d1.integer))
            self.log.debug("Received CRC for lane 2: %04x, calculated CRC: %04x" %(crc2.integer, crc16_d2.integer))
            self.log.debug("Received CRC for lane 3: %04x, calculated CRC: %04x" %(crc3.integer, crc16_d3.integer))
            crc_pass = crc0.integer == crc16_d0.integer
            crc_pass &= crc1.integer == crc16_d1.integer
            crc_pass &= crc2.integer == crc16_d2.integer
            crc_pass &= crc3.integer == crc16_d3.integer
        else:
            num_bits_to_crc = count * 8
            crc16_d0 = BinaryValue(value=sdio_utils.crc16_gen(data,num_bits_to_crc),bits=16,bigEndian=False)
            self.log.debug("Received CRC for lane 0: %04x, calculated CRC: %04x" %(crc0.integer, crc16_d0.integer))
            crc_pass = crc0.integer == crc16_d0.integer

        if crc_pass:
            pass
        elif not crc_pass and not self.data_read_aborted:
            raise sdio_host.SDIODataError("Extended RW IO read data CRC error.")
        elif self.data_read_aborted:
            self.log.info("Detected data read aborted during block read, CRC didn't match but that's to be expected")
            raise ReturnValue((data,"aborted"))
        
        raise ReturnValue((data,"okay"))

    def read_wait(self, asserting=True):
        """
        DAT[2] is used to indicate to the device during a multi-block read (CMD53) that we don't
        want the next block of data yet
        """
        if asserting:
            # Deposit the values and set the direction
            self.bus.data_coco_dir[2] <= 1
            self.bus.data_coco_out[2] <= 0
        else:
            self.bus.data_coco_dir[2] <= 0
            self.bus.data_coco_out[2] <= 1
    
    @cocotb.coroutine
    def data_bus_write(self, data=None, timeout=4000, could_abort=False, final_block=False):
        """
        Write out the bytes in data (which should be a list of integers each no bigger than a byte) on the data
        bus and append the CRC.

        It's up to the caller to pass the exact right number of bytes for the command.

        We'll return the block crc response and wait until it's all done being written by the host
        """
        num_bytes = len(data)
        self.log.debug("Writing %d bytes onto data bus" %num_bytes)
        
        self.bus.data_coco_dir[0] <= 1
        if self.bus_width == 4:
            self.bus.data_coco_dir[1] <= 1
            self.bus.data_coco_dir[2] <= 1
            self.bus.data_coco_dir[3] <= 1

        # First the start bit(s)
        yield FallingEdge(self.clock)
        if self.bus_width == 4:
            self.bus.data_coco_out[0] <= 0
            self.bus.data_coco_out[1] <= 0
            self.bus.data_coco_out[2] <= 0
            self.bus.data_coco_out[3] <= 0
        elif self.bus_width == 1:
            self.bus.data_coco_out[0] <= 0

        num_bits_to_crc = 0
        # Next the data
        for byte_num in range(0,len(data)):
            byte = BinaryValue(value=data[byte_num],bits=8,bigEndian=False)
            if self.bus_width == 4:
                # See section 3.6.1/figure 3-9 (page 9) of the SD physical spec for how the data is presented
                yield FallingEdge(self.clock)
                self.log.debug("Data write bit %d, 4-bit MSNibble: %1x" %(num_bits_to_crc, byte[7:4].integer))
                self.bus.data_coco_out <= byte[7:4].integer | (0xf << 4)
                yield FallingEdge(self.clock)
                self.log.debug("Data write bit %d, 4-bit LSNibble: %1x" %(num_bits_to_crc + 1, byte[3:0].integer))
                self.log.debug("    SDIO host data write byte %02d: %02x" %(byte_num,byte))
                self.bus.data_coco_out <= byte[3:0].integer | (0xf << 4)
                num_bits_to_crc += 2
            else:
                self.log.debug("    SDIO host data write byte %02d: %02x" %(byte_num,byte))
                for bit in range(7,-1,-1):
                    yield FallingEdge(self.clock)
                    self.bus.data_coco_out <= byte[bit].integer | (0x7f << 1)
                    num_bits_to_crc += 1
                
        # Now the CRC
        if self.bus_width == 4:
            d0,d1,d2,d3 = sdio_utils.crc16_array_prep(self.bus_width, data)
            crc16_d0 = BinaryValue(value=sdio_utils.crc16_gen(d0,num_bits_to_crc),bits=16,bigEndian=False)
            crc16_d1 = BinaryValue(value=sdio_utils.crc16_gen(d1,num_bits_to_crc),bits=16,bigEndian=False)
            crc16_d2 = BinaryValue(value=sdio_utils.crc16_gen(d2,num_bits_to_crc),bits=16,bigEndian=False)
            crc16_d3 = BinaryValue(value=sdio_utils.crc16_gen(d3,num_bits_to_crc),bits=16,bigEndian=False)
            self.log.debug("input to CRC16, data bytes lane 3 : %s" % [BinaryValue(value=x,bits=8,bigEndian=False).binstr for x in d3])
            self.log.debug("input to CRC16, data bytes lane 2 : %s" % [BinaryValue(value=x,bits=8,bigEndian=False).binstr for x in d2])
            self.log.debug("input to CRC16, data bytes lane 1 : %s" % [BinaryValue(value=x,bits=8,bigEndian=False).binstr for x in d1])
            self.log.debug("input to CRC16, data bytes lane 0 : %s" % [BinaryValue(value=x,bits=8,bigEndian=False).binstr for x in d0])
            self.log.debug("CRC16, data lane 3 : %s (%04x)" %( crc16_d3.binstr, crc16_d3.integer))
            self.log.debug("CRC16, data lane 2 : %s (%04x)" %( crc16_d2.binstr, crc16_d2.integer))
            self.log.debug("CRC16, data lane 1 : %s (%04x)" %( crc16_d1.binstr, crc16_d1.integer))
            self.log.debug("CRC16, data lane 0 : %s (%04x)" %( crc16_d0.binstr, crc16_d0.integer))
            for bit in range(15,-1,-1):
                yield FallingEdge(self.clock)
                self.bus.data_coco_out[0] <= crc16_d0[bit].integer
                self.bus.data_coco_out[1] <= crc16_d1[bit].integer
                self.bus.data_coco_out[2] <= crc16_d2[bit].integer
                self.bus.data_coco_out[3] <= crc16_d3[bit].integer
        else:
            crc16 = BinaryValue(value=sdio_utils.crc16_gen(data,num_bits_to_crc),bits=16,bigEndian=False)
            for bit in range(15,-1,-1):
                yield FallingEdge(self.clock)
                self.bus.data_coco_out[0] = crc16[bit].integer | (0x7f << 1)
                
        yield FallingEdge(self.clock)
        # All CRC bits clocked out now, stop bit
        self.bus.data_coco_out <= 0xf
        self.bus.data_coco_dir <= 1
        yield FallingEdge(self.clock)
        self.bus.data_coco_dir <= 0 # Let bus go high-impedence

        # 2 cycles until CRC response and busy indication
        yield RisingEdge(self.clock)
        yield RisingEdge(self.clock)
        # Go to next rising edge
        yield RisingEdge(self.clock)

        # Now we should expect the start bit
        crc_resp = 0
        dbus = BinaryValue(value=int(self.bus.data_coco_in),bits=8,bigEndian=False)
        if dbus[0].integer != 0:
            self.log.error("Write block CRC response did not start in the correct place.")
        for i in range(5):
            yield RisingEdge(self.clock)
            dbus = BinaryValue(value=int(self.bus.data_coco_in),bits=8,bigEndian=False)
            if i < 3:
                crc_resp |= dbus[0].integer << i
        if crc_resp != 0x2:
            self.log.info("Write block CRC response incorrect, response was %x" %crc_resp)
            raise ReturnValue(crc_resp)

        timeout_count = 0
        dbus = BinaryValue(value=int(self.bus.data_coco_in),bits=8,bigEndian=False)
        while dbus[0].integer == 0:
            yield RisingEdge(self.clock)
            timeout_count += 1
            if timeout_count == timeout:
                self.log.error("Timeout waiting for device to write data")
                break
            dbus = BinaryValue(value=int(self.bus.data_coco_in),bits=8,bigEndian=False)
        raise ReturnValue(crc_resp)
    
class CocotbSPIHostDriver(BusDriver):
    """
    A SPI SDIO host driver for this testbench, driven directly from Cocotb

    SPI->SDIO pin mappings:
     SPI CS -> SDIO D[3]
     SPI MOSI -> SDIO Cmd
     SPI MISO -> SDIO D[0]
     SPI IRQ -> SDIO D[3]

    """
    _signals = ["cmd_coco_dir", "cmd_coco_out", "cmd_coco_in",
                "data_coco_dir", "data_coco_out", "data_coco_in"]

    def __init__(self, entity, name, clock):
        BusDriver.__init__(self,entity, name, clock)
        self.bus.cmd_coco_dir.setimmediatevalue(1) # SPI mode - direction of CMD pin is always host->slave (1)
        self.bus.cmd_coco_out.setimmediatevalue(1)
        self.bus.data_coco_dir.setimmediatevalue(0x08) # D[3] (chip select) is always asserted host->slave
        self.bus.data_coco_out.setimmediatevalue(0x08)
        self.bus_width = 1

        # Locks implemented with cocotb Events, normal threading.Lock guys don't work
        self.cmd_bus_busy_event = Event("cmd_bus_busy")
        self.cmd_bus_busy = False
        self.data_write_aborted = False
        self.data_read_aborted = False

    def set_cs_n(self,val=1):
        """
        SPI CS is active low, assume caller knows this (don't invert in here)
        """
        self.bus.data_coco_out[3] <= val
        
    def set_mosi(self,val=0):
        """
        SPI MOSI assignment
        """
        self.bus.cmd_coco_out <= val

    def get_miso(self):
        """
        SPI MISO read
        """
        return int(BinaryValue(value=int(self.bus.data_coco_in),bits=8,bigEndian=False)[0])
        
    @cocotb.coroutine
    def acquire_cmd_lock(self):
        if self.cmd_bus_busy:
            yield self.cmd_bus_busy_event.wait()
        self.cmd_bus_busy_event.clear()
        self.cmd_bus_busy = True

    def release_cmd_lock(self):
        self.cmd_bus_busy = False
        self.cmd_bus_busy_event.set()
        
    @cocotb.coroutine
    def send_cmd(self, cmd):
        """
        Transmit a command, we will calculate the CRC and append it
        """
        crc7 = sdio_utils.crc7_gen(number=cmd[47:8].integer)
        cmd[7:1] = BinaryValue(value=crc7,bits=7,bigEndian=False).integer
        self.log.debug("SDIO host sending CMD%d: 'b%s" %(cmd[45:40].integer, cmd.binstr))
        self.set_cs_n(0)
        for x in range(47,-1,-1):
            # Now shift this out bit by bit
            #yield FallingEdge(self.clock)
            yield RisingEdge(self.clock)
            self.set_mosi(cmd[x].integer)
            # Think we should probably keep CS asserted after a command, at least until the response
            #if x == 0:
            #    self.set_cs_n(1) # Deassert CS
        yield RisingEdge(self.clock)
        self.set_mosi(1)

    @cocotb.coroutine
    def get_cmd_response_bits(self,cmd,timeout=1000,timeout_possible=False):
        """
        Read the response from the bus, first 8 bits are always an R1 indicating if there were any errors
        """
        cmd_num = cmd[45:40].integer
        response_type, response_length = sdio_utils.get_spi_response_type(cmd_num)
        response_bit_counter = response_length
        response = BinaryValue(value=0,bits=response_length,bigEndian=False)
        if response_type == None:
            return
        timeout_count = 0
        while timeout_count < timeout:
            yield RisingEdge(self.clock)
            if response_bit_counter == response_length:
                # Response not started yet
                if self.get_miso() == 0:
                    # Start bit of response
                    response[response_bit_counter - 1] = self.get_miso()
                    response_bit_counter -= 1
            elif response_bit_counter > 0:
                # Shift in the response bits
                response[response_bit_counter - 1] = self.get_miso()
                response_bit_counter -= 1
            else:
                break
            timeout_count += 1
        if timeout_count == timeout:
            timeout_message = "Timeout waiting for response to SPI cmd%d: %x ('b%s)" %(cmd_num, cmd.integer, cmd.binstr)
            if timeout_possible:
                self.log.info(timeout_message)
                raise ReturnValue("timeout")
            else:
                raise TestFailure(timeout_message)
        if not cmd_num in [53]:
            # De-assert CS if not a bulk data transfer
            self.set_cs_n(1)
            for _ in range(8):
                yield RisingEdge(self.clock)
            
        raise ReturnValue(response)

    @cocotb.coroutine
    def data_bus_read(self, count=1, timeout=None, could_abort=False, final_block=False):
        """
        Read the data bytes off the SD bus
        """
        data = []
        # Wait for the start bit
        timeout_count = 0
        timed_out = False
        timens_before = get_sim_time(units='ns') # Value we started at - SDIO timeout is 1ms, wait that long
        while BinaryValue(value=int(self.bus.data_coco_in),bits=8,bigEndian=False)[0] == 1:
            yield RisingEdge(self.clock)
            self.log.debug("Waited a clock for data bus bit 0 to be 0")
            if timeout:
                # Timeout in cycles has been specified
                timeout_count += 1 # I think it's allowed up to a second but in simulation let's just wait an infeasibly long time
                if timeout_count == timeout:
                    timed_out = True
            elif int(get_sim_time(units='ns') - timens_before) >= 1000000:
                # 1ms timeout
                timed_out = True

            if timed_out:
                if could_abort:
                    self.log.info("Timed out waiting for start bit of a block of data. Assuming aborted transfer.")
                    raise ReturnValue((data,"aborted"))
                else:
                    raise sdio_host.SDIODataError("Timeout waiting for start bit of data transfer")

        self.log.debug("SDIO host model data receive start bit detected")
        for byte in range(0,count):
            current_byte = BinaryValue(value=0,bits=8,bigEndian=False)
            # 1-bit mode
            for bit in range(7,-1,-1):
                yield RisingEdge(self.clock)
                current_byte[bit] = self.get_miso()
            self.log.debug("SDIO host model received byte num %03d: %02x" %(byte,current_byte.integer))
            data.append(current_byte.integer)
            if could_abort and self.data_read_aborted:
                self.log.info("Detected data read aborted after %d bytes" %(byte))
                raise ReturnValue((data,"aborted"))

        # CRC time
        crc0 = BinaryValue(value=0,bits=16,bigEndian=False)
        for bit in range(15,-1,-1):
            yield RisingEdge(self.clock)
            crc0[bit] = self.get_miso()
            if could_abort and self.data_read_aborted:
                self.log.info("Detected data read aborted during CRC, already received %d bytes" %(len(data)))
                raise ReturnValue((data,"aborted"))

        num_bits_to_crc = count * 8
        crc16_d0 = BinaryValue(value=sdio_utils.crc16_gen(data,num_bits_to_crc),bits=16,bigEndian=False)
        self.log.debug("Received CRC for lane 0: %04x, calculated CRC: %04x" %(crc0.integer, crc16_d0.integer))
        crc_pass = crc0.integer == crc16_d0.integer

        if crc_pass:
            pass
        elif not crc_pass and not self.data_read_aborted:
            raise sdio_host.SDIODataError("Extended RW IO SPI read data CRC error.")
        elif self.data_read_aborted:
            self.log.info("Detected data read aborted during block read, CRC didn't match but that's to be expected")
            raise ReturnValue((data,"aborted"))

        if final_block:
            yield RisingEdge(self.clock)
            self.set_cs_n(1)
            for _ in range(8):
                yield RisingEdge(self.clock)
        
        raise ReturnValue((data,"okay"))

    @cocotb.coroutine
    def data_bus_write(self, data=None, timeout=4000, could_abort=False, final_block=False):
        """
        SPI bus block write data (which should be a list of integers each no bigger than a byte) on the data
        bus and append the CRC.

        Prepend this with the appropriate block-write token.

        It's up to the caller to pass the correct number of bytes for the command.

        We'll return the block crc response and wait until it's all done being written by the host
        """
        num_bytes = len(data)
        self.log.debug("Writing %d bytes onto SPI data bus" %num_bytes)
        
        # Start token (as per SD physical spec sect. 7.3.3.2)
        self.set_mosi(1)
        for _ in range(0,7):
            yield RisingEdge(self.clock)
        self.set_mosi(0)
        yield RisingEdge(self.clock)

        num_bits_to_crc = 0
        # Next the data
        for byte_num in range(0,len(data)):
            byte = BinaryValue(value=data[byte_num],bits=8,bigEndian=False)
            self.log.debug("    SPI host data write byte %02d: %02x" %(byte_num,byte))
            for bit in range(7,-1,-1):
                self.set_mosi(byte[bit].integer)
                yield RisingEdge(self.clock)
                num_bits_to_crc += 1

        crc16 = BinaryValue(value=sdio_utils.crc16_gen(data,num_bits_to_crc),bits=16,bigEndian=False)
        self.log.debug("     Block write CRC: %04x" %crc16.integer)
        for bit in range(15,-1,-1):
            self.set_mosi(crc16[bit].integer)
            yield RisingEdge(self.clock)

        # Set MOSI back to idle state
        self.set_mosi(1)
        
        # Write status, read bytes until we see it again
        status = BinaryValue(value=0,bits=8,bigEndian=False)
        write_done = False
        timeout_count = 0
        while not write_done:
            for bit in range(7,-1,-1):
                yield RisingEdge(self.clock)
                status[bit] = self.get_miso()
            if status[4] == 0:
                assert (status[0] == 1), "SPI write data block response token malformed, bit 0 wasn't 1 (%02x)" % status.integer
                write_done = True
                # We have a status:
                if status[3:1].integer == 0x2:
                    # Data accepted
                    self.log.debug("SPI block write accepted")
                elif status[3:1].integer == 0x5:
                    self.log.error("SPI block data write rejectd due to incorrect CRC")
            timeout_count += 8
            if timeout_count >= timeout:
                self.log.error("Timeout waiting for device to write data")
                break

        # Now wait for write busy to be deasserted
        timeout_count = 0
        write_busy = BinaryValue(value=0,bits=8,bigEndian=False)
        while True:
            # Read bytes from the bus until we get 1 again
            for bit in range(7,-1,-1):
                yield RisingEdge(self.clock)
                write_busy[bit] = self.get_miso()
            if write_busy[0] == 1:
                break
            timeout_count += 8
            if timeout_count >= timeout:
                self.log.error("Timeout waiting for device to write data")
                break

        if final_block:
            yield RisingEdge(self.clock)
            self.set_cs_n(1)
            for _ in range(8):
                yield RisingEdge(self.clock)

        raise ReturnValue(status[3:1].integer)
