/*
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
 
 Example testbench, demonstrating SDIO host driver hookup suitable for use with
 host driver model.

*/
`timescale 1ps/1ps

module tb
  (
    inout pad_SDIO_CLK,
    inout pad_SDIO_CMD,
    inout pad_SDIO_D0,
    inout pad_SDIO_D1,
    inout pad_SDIO_D2,
    inout pad_SDIO_D3
   );
  
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

  initial
    begin
      $dumpfile("sdio_example.vcd");
      $dumpvars(0,tb);
 end

endmodule // tb
