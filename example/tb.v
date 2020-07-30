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
