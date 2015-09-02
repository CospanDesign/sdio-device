/*
Distributed under the MIT license.
Copyright (c) 2011 Dave McCoy (dave.mccoy@cospandesign.com)

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
*/



//sd_host_platform_cocotb.v
`timescale 1 ns/1 ps


module sd_dev_platform_cocotb (

input               clk,
input               rst,

//SD Stack Interface
output              o_locked,
output  reg         o_out_clk,
output              o_out_clk_x2,

input               i_sd_cmd_dir,
output              o_sd_cmd_in,
input               i_sd_cmd_out,


input               i_sd_data_dir,
output        [7:0] o_sd_data_in,
input         [7:0] i_sd_data_out,

input               i_phy_out_clk,
inout               io_phy_sd_cmd,
inout         [3:0] io_phy_sd_data

);

assign  o_out_clk_x2 = clk;
assign  o_locked  =  1;

assign  io_phy_sd_cmd = i_sd_cmd_dir  ? i_sd_cmd_out : 1'hZ;
assign  o_sd_cmd_in   = io_phy_sd_cmd;

assign  io_phy_sd_data= i_sd_data_dir ? i_sd_data_out: 8'hZ;
assign  o_sd_data_in  = io_phy_sd_data;



always @ (posedge clk) begin
  if (rst) begin
    o_out_clk     <=  0;
    o_phy_out_clk <=  0;
  end
  else begin
    o_out_clk     <= ~o_out_clk;
    o_phy_out_clk <= ~o_phy_out_clk;
  end
end
endmodule

