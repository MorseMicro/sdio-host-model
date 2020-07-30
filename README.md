# Cocotb SDIO host model

These Python modules implement an SDIO 2.0-era host model, providing the basics required for device initialization and data transfers via CMD52 and CMD53. There's some other useful bits and pieces in here too, but mostly it provides the ability to communicate with an SDIO device under simulation in Cocotb.

## Organization

The SDIO specification talks about a protocol over the SD bus physical layer, which could either be the usual SD-card looking setup or SPI. So that's how these modules are split up, the `sdio_host.py` provides the upper layers, implementing commands and responses, and the `sdio_phy_drivers.py` implements the physical layers for either SDIO or SPI buses.

`sdio_utils.py` is a place for things which can get used between both, or even in the higher layers of cocotb.

### Cocotb

Cocotb is a most excellent Python framework which provides a neat interface into HDL simulators via their binary API, VPI, etc.

http://cocotb.org/

# Using

Simply put, you'll need some signals in your testbench you'll pass to the phy module when it's created, and then you'll pass your phy module to the `SDIOHost` module when you create it. With that you can send arbitrary commands and data, or use the CMD52 and CMD53 `cmd_io_rw_direct` and `cmd_io_rw_extended` functions, respectively, to do individual byte and block data transfers.

## Example

The following is a concise version of what's in the `example/` directory for you to run.

You'll need Icarus Verilog installed to run the example, if you want to.

To run it, do:

```
cd example
git submodule init
git submodule update
make sim
```

It will fail, beacuse there's no SDIO device module for the initialization to complete with, but it'll generate a VCD showing the initial commands the host tries to send.

The following assumes in your Verilog testbench you have the following signals attached to your DUT with an SDIO device within. All signals prefixed with `pad_` are the actual chip's pad IOs.

```
    ////////////////////////////////////////////////////////////////////////////////
    // SDIO
    ////////////////////////////////////////////////////////////////////////////////
    reg  sdio_clk;
    reg  sd_cmd_coco_dir;
    wire sd_cmd_coco_in;
    reg  sd_cmd_coco_out;
    reg [3:0] sd_data_coco_dir;
    wire [3:0] sd_data_coco_in;
    reg [3:0]  sd_data_coco_out;

    // SDIO command pins
    assign pad_SDIO_CMD = sd_cmd_coco_dir ? sd_cmd_coco_out : 1'bZ;
    assign sd_cmd_coco_in = pad_SDIO_CMD;
    pullup(pad_SDIO_CMD);

    // X-warner on the SDIO command bus (when we're out of reset)
    always @(negedge sdio_clk)
      if (dut.rst === 1'b0) // TODO - hook this up to a reset in your system so you don't get X warnings before the chip is out of reset
        if (sd_cmd_coco_in === 1'bx)
          $display("%-0t %m: SDIO command bus is X, this may result in CRC errors", $time/1000);

    // SDIO data pins - drivers from cocotb and pullup the bus
    assign pad_SDIO_D0 = sd_data_coco_dir[0] ? sd_data_coco_out[0] : 1'bZ;
    assign pad_SDIO_D1 = sd_data_coco_dir[1] ? sd_data_coco_out[1] : 1'bZ;
    assign pad_SDIO_D2 = sd_data_coco_dir[2] ? sd_data_coco_out[2] : 1'bZ;
    assign pad_SDIO_D3 = sd_data_coco_dir[3] ? sd_data_coco_out[3] : 1'bZ;

    generate
    begin
        pullup(pad_SDIO_D0);
        pullup(pad_SDIO_D1);
        pullup(pad_SDIO_D2);
        pullup(pad_SDIO_D3);
    end
    endgenerate

    assign sd_data_coco_in  = {pad_SDIO_D3,pad_SDIO_D2,pad_SDIO_D1,pad_SDIO_D0};
    assign pad_SDIO_CLK  = sdio_clk;
```

and then in your Cocotb test module you'll have:

```
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

```



