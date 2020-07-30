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

The following is example code showing setup and initialization of the SDIO host
model module and testbench.

"""

import cocotb
from cocotb.result import TestFailure, TestSuccess, ReturnValue
from cocotb.clock import Clock
from cocotb.triggers import Timer, FallingEdge, RisingEdge, Event, Join
from cocotb.binary import BinaryValue
from cocotb.utils import get_sim_time
import logging
from cocotb.log import SimLog

import sdio_utils
import sdio_phy_drivers
import sdio_host
from sdio_host import SDIOHost

# Objects we'll use to store the clock generation state
clk_sdio = None
clk_sdio_gen = None

@cocotb.coroutine
def set_sdio_clock(dut,sdio_clk_period=20.0,sdio_clk_units="ns",stop=False):
    """
    Stop and restart the clocks for the device
    """
    global clk_sdio
    global clk_sdio_gen
    if clk_sdio_gen:
        # Kill the clock if it's started as we'll re-create it
        clk_sdio_gen.kill()
    yield Timer(1,units="us")
    if stop:
        dut.log.info("SDIO clock stopped")
        raise ReturnValue(0)
    clk_sdio = Clock(dut.sdio_clk, sdio_clk_period, units=sdio_clk_units)
    clk_sdio_gen = cocotb.fork(clk_sdio.start())
    dut.log.info("SDIO clock period set to %s%s" %(sdio_clk_period,sdio_clk_units))


@cocotb.coroutine
def sdio_test_setup(dut):
    """
    Create the clocks, drivers and testbench instances, pass them back
    """
    # Initialize all SDIO bus controls
    dut.sd_cmd_coco_dir.setimmediatevalue(0)
    dut.sd_data_coco_dir.setimmediatevalue(0x00)
    dut.sd_data_coco_out.setimmediatevalue(0xff)
    dut.sd_cmd_coco_out.setimmediatevalue(1)
    dut.sdio_clk.setimmediatevalue(0)
    
    # 400kHz for initialization
    #yield set_clocks(dut,sdio_clk_period=2500.0)
    # Actually, that's too slow, go straight to 50MHz for now
    yield set_sdio_clock(dut,sdio_clk_period=20.0)

    phy = sdio_phy_drivers.CocotbSDIOHostDriver(dut,"sd",dut.sdio_clk)
    drv = SDIOHost(clock=dut.sdio_clk,phy=phy)
    raise ReturnValue(drv)

@cocotb.test(skip = False)
def sdio_init(dut):
    """
    Description:
        Test initializing the SDIO device interface
    """
    drv = yield sdio_test_setup(dut)
    yield Timer(1,units='us')
    yield drv.sdio_init(dump_regs=True)
    raise TestSuccess()

@cocotb.test(skip = False)
def sdio_do_writes(dut):
    """
    Description:
        A simple function demonstrating the write feature
    """
    drv = yield sdio_test_setup(dut)
    yield Timer(1,units='us')
    # Just demonstrate doing a CMD52 write without initializing, set timeout_possible so it doesn't bail out
    yield drv.cmd_io_rw_direct(rw=1,fn=0,addr=0xabcd,data=0x5a,timeout_possible=True)
    # or do a write via the nice write_reg function
    yield drv.write_reg(fn=0,addr=0x1111,data=0x96, timeout_possible=True)
    # Now demonstrate doing a block write
    yield drv.cmd_io_rw_extended(rw=1,fn=1,block=0,op=1,addr=0,data=[x for x in range(0,256)])
    raise TestSuccess()
