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

This file provides a bunch of generic and cocotb-dependent functions which help with SDIO driving

"""
from cocotb.log import SimLog
from cocotb.binary import BinaryValue

log = SimLog("sdio_utils")

# General SDIO definitions
_cccrs = [{'name': 'CCCR/SDIO revision',        'addr': 0},
          {'name': 'SD spec revision',          'addr': 1},
          {'name': 'I/O enables',               'addr': 2, 'bin': True}, # 'bin': True will make the dumper print it in binary instead of hex
          {'name': 'I/O ready',                 'addr': 3, 'bin': True},
          {'name': 'Int enable',                'addr': 4, 'bin': True},
          {'name': 'Int pending',               'addr': 5, 'bin': True},
          {'name': 'I/O abort',                 'addr': 6, 'bin': True},
          {'name': 'Bus interface control',     'addr': 7, 'bin': True},
          {'name': 'Card capability',           'addr': 8, 'bin': True},
          {'name': 'Common CIS pointer byte 0', 'addr': 9},
          {'name': 'Common CIS pointer byte 1', 'addr': 10},
          {'name': 'Common CIS pointer byte 2', 'addr': 11},
          {'name': 'Bus suspend',               'addr': 12, 'bin': True},
          {'name': 'Function select',           'addr': 13},
          {'name': 'Exec flags',                'addr': 14, 'bin': True},
          {'name': 'Ready flags',               'addr': 15, 'bin': True},
          {'name': 'FN0 block size byte 0',     'addr': 16},
          {'name': 'FN0 block size byte 1',     'addr': 17},
          {'name': 'Power control',             'addr': 18, 'bin': True},
          {'name': 'Bus speed select',          'addr': 19, 'bin': True},
          {'name': 'UHS-I support',             'addr': 20},
          {'name': 'Driver strength',           'addr': 21, 'bin': True},
          {'name': 'Interrupt extension',       'addr': 22, 'bin': True}]

_cia_base_addresses =  {'CCCR': 0x0,
                       'FBR1' : 0x100,
                       'FBR2' : 0x200,
                       'FBR3' : 0x300,
                       'FBR4' : 0x400,
                       'FBR5' : 0x500,
                       'FBR6' : 0x600,
                       'FBR7' : 0x700,
                       #...
                       'CIS'  :0x1000}

# Function basic registers
_fbrs = [{'name': 'Standard function code',           'addr': 0},
         {'name': 'Standard function code extended ', 'addr': 1},
         {'name': 'Power state support',              'addr': 2, 'bin': True},
         {'name': 'Function CIS pointer byte 0',      'addr': 9,},
         {'name': 'Function CIS pointer byte 1',      'addr': 10,},
         {'name': 'Function CIS pointer byte 2',      'addr': 11,},
         {'name': 'Function CSA pointer byte 0',      'addr': 12,},
         {'name': 'Function CSA pointer byte 1',      'addr': 13,},
         {'name': 'Function CSA pointer byte 2',      'addr': 14,},
         {'name': 'Function I/O block size byte 0',   'addr': 16,},
         {'name': 'Function I/O block size byte 1',   'addr': 17,}]


def get_addr_by_name(name=""):
    regaddr = [x['addr'] for x in _cccrs if x['name'] == name]
    if regaddr:
        return regaddr[0]
    regaddr = [x['addr'] for x in _fbrs if x['name'] == name]
    if not regaddr:
        raise Exception("Tried to get register %s address but it doesn't exist in the CCCR regs" % name)
    return regaddr[0]

def init_cmd( cmd_num=0):
    """
    Initialize a BinaryValue to a command without the fields filled in.
    Page 81, section 4.7.2 of SD physical layer spec has the description of commands.

    They're 48-bits big. The spec numbers the bits going out *first* as the highest
    in the field descriptions.

    Also section 4.2.2 on the SDIO spec (page 18) says there are a bunch of commands
    not supported by the SDIO devices, check for them.
    """
    if cmd_num in [12,16,2,4,9,10,13,17,18,24]:
        log.warning("Command %d is not supported on SDIO, sending anyway but what are you doing?!" %cmd_num)

    cmd = BinaryValue(bits=48,bigEndian=False)
    cmd[47] = 0 # Start value
    cmd[46] = 1 # Direction , 1 = towards device, 0 = towards host
    cmd[45:40] = BinaryValue(value=cmd_num, bits=6, bigEndian=False).integer
    cmd[0] = 1 # Stop bit
    return cmd

def get_response_type(cmd_num):
    """
    Encode the table on page 86 of the SD phy spec (section 4.7.4) 
    describing what responses we should get from each command"""
    length = 48 # Default length of a response
    if cmd_num in [0,4,15]:
        # No response expected
        rv = None
    if cmd_num in [11,13,16,17,18,19,23,55,56]:
        # Response type 1
        rv = 1
    if cmd_num in [7,12,20]:
        # Response type 1b, means it could also be a busy
        rv = 1.5
    if cmd_num in [2,9,10]:
        # Reponse type 2, CID/CSD register, not on SDIO but here for completeness
        length = 136
        rv = 2
    if cmd_num in [4,5]:
        rv = 4
    if cmd_num in [52,53]:
        rv = 5
    if cmd_num in [3]:
        rv = 6
    if cmd_num in [8]:
        rv = 7
    log.debug("Cmd %d expects response type R%s" %(cmd_num,rv))
    return (rv, length)

def get_spi_response_type(cmd_num):
    """
    Encode section 7.3.2.1 of the SD physical spec 2.0
    SPI mode commands and their responses, but for IO-only
    """
    length = 8 # Default length of a response
    resp_type = 1
    if cmd_num in [8]:
        # CMD8 gets R7
        resp_type = 7
        length = 40
    if cmd_num in [5]:
        # CMD5 gets a R4 back in SPI mode
        resp_type = 4
        length = 40
    if cmd_num in [52,53]:
        resp_type = 5
        length = 16
        
    log.debug("Cmd %d expects response type R%s" %(cmd_num,resp_type))
    return (resp_type, length)

def crc7_gen( number, bit_count=40):
    """
    Generate the CRC for the end of command bits
    """
    byte_index = 0
    crc = 0
    regval = 0
    byte_val = 0
    byte_index = 0
    for i in range (bit_count, 0, -8):
        shift_val = i - 8
        byte_val = (number >> shift_val) & 0xFF
        for bit_index in range (8):
            regval = regval << 1
            if ((byte_val ^ regval) & 0x80) > 0:
                regval = regval ^ 9
            byte_val = (byte_val << 1) & 0xFF
        regval = (regval & 0x7F)
    log.debug("CRC7 calc - Number:\t\t\t\t0x%09X" % number)
    log.debug("CRC7 calc - CRC:\t\t\t0x%02X" % regval)
    return regval

def crc16_array_prep( width, data):
    """
    Data bus shifting is described in Figure 3-9, pg9 of the SD physical spec.
    In wide-bus mode (4-bits) the CRC is calculated per line so
    arrange the bytes so they go MSbit-LSbit for each line.
    Eg: 
    D3: B0b7 B0b3 B1b7 B1b3 ...
    D2: B0b6 B0b2 B1b6 B1b2 ...
    D1: B0b5 B0b1 B1b5 B1b1 ...
    D0: B0b4 B0b0 B1b4 B1b0 ...
    So that for the CRC for D3 we have bytes which looke like:
    D3B0 = {B0b7,B0b3,B1b7,B1b3,B2b7,B2b3,B3b7,B3b3}
    D2B0 = {B0b6,B0b2,B1b6,B1b2,B2b6,B2b2,B3b6,B3b2}
    D1B0 = {B0b5,B0b1,B1b5,B1b1,B2b5,B2b1,B3b5,B3b1}
    D0B0 = {B0b4,B0b0,B1b4,B1b0,B2b4,B2b0,B3b4,B3b0}
    etc.
    Return the 4 byte arrays
    """
    if width != 4:
        raise Exception("Error, crc16_array_prep called but data width is not 4-bits on the SDIO interface")

    def get_bits_and_shift(byte,bit1,bit0,shift):
            """
            Return {byte[bit1],byte[bit0]} << shift
            """
            bit1 = ((byte & (1 << bit1)) >> bit1) << 1
            bit0 = ((byte & (1 << bit0)) >> bit0)
            return (bit1 | bit0) << shift

    D0=[]
    D1=[]
    D2=[]
    D3=[]
    B0=0
    B1=0
    B2=0
    B3=0
    for byte_num in range(0,len(data)):
        if (byte_num % 4 == 0) and byte_num > 0:
            # Append the bytes and reset them
            D0.append(B0)
            D1.append(B1)
            D2.append(B2)
            D3.append(B3)
            B0=0
            B1=0
            B2=0
            B3=0
        # Get the next byte    
        B = data[byte_num]
        if byte_num % 4 == 0:
            B0 |= get_bits_and_shift(B,4,0,6)
            B1 |= get_bits_and_shift(B,5,1,6)
            B2 |= get_bits_and_shift(B,6,2,6)
            B3 |= get_bits_and_shift(B,7,3,6)
        if byte_num % 4 == 1:
            B0 |= get_bits_and_shift(B,4,0,4)
            B1 |= get_bits_and_shift(B,5,1,4)
            B2 |= get_bits_and_shift(B,6,2,4)
            B3 |= get_bits_and_shift(B,7,3,4)
        if byte_num % 4 == 2:
            B0 |= get_bits_and_shift(B,4,0,2)
            B1 |= get_bits_and_shift(B,5,1,2)
            B2 |= get_bits_and_shift(B,6,2,2)
            B3 |= get_bits_and_shift(B,7,3,2)
        if byte_num % 4 == 3:
            B0 |= get_bits_and_shift(B,4,0,0)
            B1 |= get_bits_and_shift(B,5,1,0)
            B2 |= get_bits_and_shift(B,6,2,0)
            B3 |= get_bits_and_shift(B,7,3,0)
    # Always append what is left over
    D0.append(B0)
    D1.append(B1)
    D2.append(B2)
    D3.append(B3)
    return (D0,D1,D2,D3)

def crc16_gen( data, num_bits):
    """
    16-bit CRC algorithm
    """
    crc = 0
    round = 0
    log.debug("crc16_gen: calculating SDIO data CRC for %d bits' worth across %d bytes" %(num_bits,len(data)))
    assert((num_bits / 8) <= len(data))
    for byte in data:
        # Let's go from MSb first
        for bit in range(7,-1,-1):
            val = ((byte >> bit) ^ (crc >> 15)) & 0x1
            crc = crc ^ ((val << 4) | (val << 11))
            crc = (crc << 1) | val
            num_bits -= 1
            round += 1
            log.debug("crc16_gen round %d of %d: %04x" %(round, num_bits, crc & 0xffff))
            if num_bits == 0:
                break
        if num_bits == 0:
            break
    return (crc & 0xffff)
