/*
Distributed under the MIT license.
Copyright (c) 2015 Dave McCoy (dave.mccoy@cospandesign.com)

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

/*
 * Author:
 * Description:
 *
 * Changes:
 */

module sdio_data_phy (
  input                   clk,
  input                   clk_x2,
  input                   rst,

  //Configuration
  input                   i_ddr_en,
  input                   i_spi_phy,
  input                   i_sd1_phy,
  input                   i_sd4_phy,

  //Data Interface
  input                   i_activate,
  input                   i_write_flag,
  input           [9:0]   i_data_count,

  output  reg             o_data_wr_stb,
  output          [7:0]   o_data_wr_data,
  input                   i_data_rd_stb,
  input           [7:0]   i_data_rd_data,
  output  reg             o_data_hst_rdy, //Host May not be ready
  input                   i_data_com_rdy,

  //FPGA Platform Interface
  output  reg             o_sdio_data_dir,
  input           [7:0]   i_sdio_data_in,
  output  reg     [7:0]   o_sdio_data_out
);

//local parameters
localparam      IDLE      = 4'h0;
localparam      START     = 4'h1;
localparam      WRITE     = 4'h2;
localparam      READ      = 4'h3;
localparam      CRC       = 4'h4;
localparam      FINISHED  = 4'h5;

localparam      PROCESS_CRC = 4'h1;

//registes/wires
reg               [3:0]   state;
reg               [3:0]   crc_state;

reg               [9:0]   data_count;
//wire                      edge_toggle;
reg                       posedge_clk;
reg                       edge_toggle;
reg               [3:0]   crc_bit;
wire              [15:0]  crc_out [0:3];
reg                       crc_rst;
wire                      sdio_data [0:3];
wire                      sdio_data0;
wire                      sdio_data1;
wire                      sdio_data2;
wire                      sdio_data3;

wire                      capture_crc;
reg                       enable_crc;


wire              [15:0]  crc_out0;
wire              [15:0]  crc_out1;
wire              [15:0]  crc_out2;
wire              [15:0]  crc_out3;



reg                       prev_clk_edge;
wire                      posege_clk;

integer                   i;
//submodules
genvar g;
generate
for (g = 0; g < 4; g = g + 1) begin : data_crc
crc16 crc (
  .clk                (clk_x2                       ),
  .rst                (crc_rst                      ),
  .en                 (enable_crc                   ),
  //.bit                (i_sdio_data_in[sdio_data[g]] ),
  .bit                (sdio_data[g]                 ),
  .crc                (crc_out[g]                   )
);
end
endgenerate


//asynchronous logic
assign  crc_out0      = crc_out[0];
assign  crc_out1      = crc_out[1];
assign  crc_out2      = crc_out[2];
assign  crc_out3      = crc_out[3];

assign  sdio_data0    = sdio_data[0];
assign  sdio_data1    = sdio_data[1];
assign  sdio_data2    = sdio_data[2];
assign  sdio_data3    = sdio_data[3];

assign  sdio_data[0]  = i_write_flag ?
                          edge_toggle ? i_sdio_data_in [4'h4] : i_sdio_data_in [4'h0]:
                          edge_toggle ? o_sdio_data_out[4'h4] : o_sdio_data_out[4'h0];
assign  sdio_data[1]  = i_write_flag ?
                          edge_toggle ? i_sdio_data_in [4'h5] : i_sdio_data_in [4'h1]:
                          edge_toggle ? o_sdio_data_out[4'h5] : o_sdio_data_out[4'h1];
assign  sdio_data[2]  = i_write_flag ?
                          edge_toggle ? i_sdio_data_in [4'h6] : i_sdio_data_in [4'h2]:
                          edge_toggle ? o_sdio_data_out[4'h6] : o_sdio_data_out[4'h2];
assign  sdio_data[3]  = i_write_flag ?
                          edge_toggle ? i_sdio_data_in [4'h7] : i_sdio_data_in [4'h3]:
                          edge_toggle ? o_sdio_data_out[4'h7] : o_sdio_data_out[4'h3];

//assign  posedge_clk   = clk_x2 & !prev_clk_edge;
//assign  edge_toggle   = !posedge_clk;
assign  capture_crc   = ((state == READ) || (state == WRITE));
assign  o_data_wr_data = o_sdio_data_dir ? 8'h00 : i_sdio_data_in;

//synchronous logic
always @ (posedge clk_x2) begin
  if (rst) begin
    crc_rst                   <=  1;
    crc_state                 <=  IDLE;
    enable_crc                <=  0;
  end
  else begin
    case (crc_state)
      IDLE: begin
        crc_rst               <=  1;
        if (capture_crc && posedge_clk) begin
          crc_rst             <=  0;
          crc_state           <=  PROCESS_CRC;
          enable_crc          <=  1;
        end
      end
      PROCESS_CRC: begin
        //if (!capture_crc && !posedge_clk) begin
        //if (!capture_crc && !posedge_clk) begin
        if (!capture_crc) begin
          crc_state           <=  FINISHED;
          enable_crc          <=  0;
        end
      end
      FINISHED: begin
        if (state == IDLE) begin
          crc_state           <=  IDLE;
        end
      end
    endcase
  end
end

always @ (posedge clk_x2) begin
  if (rst) begin
    edge_toggle               <=  0;
  end
  else begin
    if (clk) begin
      posedge_clk             <=  1;
      edge_toggle             <=  1;
    end
    else begin
      posedge_clk             <=  0;
      edge_toggle             <=  0;
    end
  end
end

always @ (posedge clk) begin
  o_data_wr_stb               <=  0;
  if (rst) begin
    state                     <=  IDLE;
    o_data_hst_rdy            <=  0;
    data_count                <=  0;
    o_sdio_data_dir           <=  0;
    o_sdio_data_out           <=  0;
  end
  else begin
    case (state)
      IDLE: begin
        data_count            <=  0;
        o_sdio_data_dir       <=  0;
        if (i_activate) begin
          state               <=  START;
        end
      end
      START: begin
        /*
        if (i_spi_phy) begin
          $display ("sdio_data_phy: SPI Mode not supported yet !");
          state               <=  FINISHED;
        end
        else if (i_sd1_phy) begin
          $display ("sdio_data_phy: SD1 Mode not supported yet!");
          state               <=  FINISHED;
        end
        else begin
        */
          //$display ("sdio_data_phy: SD4 Transaction Started!");
          if (i_write_flag) begin
            if (i_sdio_data_in[0] == 0) begin
              state           <=  WRITE;
            end
            else begin
            end
          end
          else begin
            if (i_data_com_rdy && i_sdio_data_in[2]) begin
              o_data_hst_rdy   <=  1;
              //Both the data bus is ready and the host has not issued the wait signal
              o_sdio_data_dir   <=  1;
              o_sdio_data_out   <=  8'h00;  //Issue two '0' values on each of the data lines to start a transaction
              state             <=  READ;
            end
          end
        //end
      end
      WRITE: begin
        if (data_count < i_data_count) begin
          data_count            <= data_count + 1;
        end
        else begin
          state                 <=  CRC;
        end
      end
      READ: begin
      end
      CRC: begin
      end
      FINISHED: begin
        o_sdio_data_dir   <=  0;
      end
      default: begin
      end
    endcase

    if (!i_activate) begin
      state               <=  IDLE;
    end
  end
end


endmodule
