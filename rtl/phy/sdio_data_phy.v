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

`define CRC_COUNT 8

module sdio_data_phy (
  input                   clk,
  input                   rst,
  input                   i_interrupt,

  //Configuration
  input                   i_ddr_en,
  input                   i_spi_phy,
  input                   i_sd1_phy,
  input                   i_sd4_phy,

  //Data Interface
  input                   i_activate,
  output  reg             o_finished,
  input                   i_write_flag,
  input           [12:0]  i_data_count,

  output  reg             o_data_wr_stb,
  output          [7:0]   o_data_wr_data,
  input                   i_data_rd_stb,
  input           [7:0]   i_data_rd_data,
  output  reg             o_data_hst_rdy, //Host May not be ready
  input                   i_data_com_rdy,

  output  reg             o_data_crc_good,

  output  reg [15:0]      o_crc0_rmt,
  output  reg [15:0]      o_crc1_rmt,
  output  reg [15:0]      o_crc2_rmt,
  output  reg [15:0]      o_crc3_rmt,

  output  reg [15:0]      o_crc0_gen,
  output  reg [15:0]      o_crc1_gen,
  output  reg [15:0]      o_crc2_gen,
  output  reg [15:0]      o_crc3_gen,



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
localparam      WRITE_CRC = 4'h5;
localparam      FINISHED  = 4'h6;

localparam      PROCESS_CRC = 4'h1;

//registes/wires
reg               [3:0]   state;
reg               [3:0]   crc_state;

reg               [12:0]  data_count;
wire                      data_crc_good;
reg               [3:0]   crc_bit;
wire              [15:0]  crc_out [0:3];

reg                       crc_rst;
wire                      crc_main_rst;
wire                      crc_main_en;
reg               [15:0]  host_crc [0:3];
wire              [15:0]  host_crc0;
wire              [15:0]  host_crc1;
wire              [15:0]  host_crc2;
wire              [15:0]  host_crc3;

wire              [15:0]  crc_out0;
wire              [15:0]  crc_out1;
wire              [15:0]  crc_out2;
wire              [15:0]  crc_out3;

reg               [7:0]   read_data;

reg               [7:0]   crc_data;
wire                      sdio_data1;
wire                      sdio_data2;
wire                      sdio_data3;

reg                       crc_enable;

reg                       prev_clk_edge;
wire                      posege_clk;

reg                       buffered_read_stb;
reg               [7:0]   buffered_read_data;
reg                       local_rst = 1;

integer                   i;
//submodules
genvar gv;
generate
for (gv = 0; gv < 4; gv = gv + 1) begin : data_crc
crc16_2bit crc(
  .clk                (clk                          ),
  .rst                (crc_main_rst                 ),
  .en                 (crc_main_en                  ),
  .bit1               (crc_data[7 - gv]             ),
  .bit0               (crc_data[7 - gv - 4]         ),
  .crc                (crc_out[gv]                  )
);

end
endgenerate

//asynchronous logic
assign  crc_main_rst  = i_data_rd_stb ? 1'b0 : crc_rst;
assign  crc_main_en   = i_data_rd_stb ? 1'b1 : crc_enable;

assign  crc_out0      = crc_out[0];
assign  crc_out1      = crc_out[1];
assign  crc_out2      = crc_out[2];
assign  crc_out3      = crc_out[3];

assign  host_crc0     = host_crc[0];
assign  host_crc1     = host_crc[1];
assign  host_crc2     = host_crc[2];
assign  host_crc3     = host_crc[3];

assign  o_data_wr_data= o_sdio_data_dir ? 8'h00 : i_sdio_data_in;
assign  data_crc_good =  ( (host_crc[0] == crc_out[0]) &&
                           (host_crc[1] == crc_out[1]) &&
                           (host_crc[2] == crc_out[2]) &&
                           (host_crc[3] == crc_out[3]));

//Synchronous Logic
always @ (posedge clk)begin
  if (rst | local_rst) begin
    buffered_read_stb         <=  1'b0;
    buffered_read_data        <=  8'h00;
  end
  else begin
    buffered_read_stb         <=  i_data_rd_stb;
    buffered_read_data        <=  i_data_rd_data;
  end
end

always @ (posedge clk) begin
  if (rst | local_rst) begin
    o_sdio_data_out           <=  8'hFF;
  end
  else begin
    o_sdio_data_out           <=  read_data;
  end
end

always @ (posedge clk) begin
  o_data_wr_stb               <=  0;
  if (rst | local_rst) begin
    local_rst                 <=  0;
    o_data_crc_good           <=  0;
    state                     <=  IDLE;
    o_data_hst_rdy            <=  0;
    data_count                <=  0;
    o_sdio_data_dir           <=  0;
    read_data                 <=  0;
    o_finished                <=  0;

    o_crc0_rmt                <=  0;
    o_crc1_rmt                <=  0;
    o_crc2_rmt                <=  0;
    o_crc3_rmt                <=  0;

    o_crc0_gen                <=  0;
    o_crc1_gen                <=  0;
    o_crc2_gen                <=  0;
    o_crc3_gen                <=  0;

    //CRC Controller
    crc_rst                   <=  1;
    crc_data                  <=  0;
    crc_enable                <=  0;

    for (i = 0; i < 4; i = i + 1) begin
      host_crc[i]             <=  0;
    end
  end
  else begin
    read_data                 <=  i_data_rd_data;
    case (state)
      IDLE: begin
        o_finished            <=  0;
        data_count            <=  0;
        crc_rst               <=  1;
        crc_data              <=  0;
        if (i_interrupt) begin
          o_sdio_data_dir     <=  1;
          read_data           <=  8'hFD;
        end
        else begin
          o_sdio_data_dir     <=  0;
          read_data           <=  8'hFF;
        end
        o_data_hst_rdy        <=  0;
        if (i_activate) begin
          crc_rst             <=  0;
          o_sdio_data_dir     <=  0;
          o_data_crc_good     <=  0;
          for (i = 0; i < 4; i = i + 1) begin
            host_crc[i]       <=  0;
          end
          state               <=  START;
        end
      end
      START: begin
        read_data             <=  8'hFF;
        //$display ("sdio_data_phy: SD4 Transaction Started!");
        if (i_write_flag) begin
          o_data_hst_rdy      <= 1;
          if (i_sdio_data_in[0] == 0) begin
            crc_enable        <=  1;
            state             <=  WRITE;
          end
          else begin
          end
        end
        else begin
          if (i_sdio_data_in[2]) begin
            o_data_hst_rdy      <=  1;
            if (i_data_com_rdy) begin
              crc_enable        <=  1;
              //Both the data bus is ready and the host has not issued the wait signal
              o_sdio_data_dir   <=  1;
              state             <=  READ;
            end
          end
        end
        if (!i_activate) begin
          state               <=  IDLE;
        end
      end
      WRITE: begin
        o_data_wr_stb           <=  1;
        if (data_count < i_data_count) begin
          data_count            <= data_count + 1;
          crc_data              <=  i_sdio_data_in;
        end
        else begin
          crc_enable            <= 0;
          o_data_wr_stb         <= 0;
          state                 <= CRC;
          data_count            <= 0;
          //data_count            <=  data_count + 1;
          host_crc[0]           <=  {host_crc[0][13:0], i_sdio_data_in[7], i_sdio_data_in[3]};
          host_crc[1]           <=  {host_crc[1][13:0], i_sdio_data_in[6], i_sdio_data_in[2]};
          host_crc[2]           <=  {host_crc[2][13:0], i_sdio_data_in[5], i_sdio_data_in[1]};
          host_crc[3]           <=  {host_crc[3][13:0], i_sdio_data_in[4], i_sdio_data_in[0]};
        end
        //Cancel a Transaction
        if (!i_activate) begin
          state                 <=  IDLE;
        end
      end
      READ: begin
        //Cancel a Transaction
        if (i_data_rd_stb) begin
          host_crc[0]           <=  crc_out[0];
          host_crc[1]           <=  crc_out[1];
          host_crc[2]           <=  crc_out[2];
          host_crc[3]           <=  crc_out[3];
        end
        if (!i_activate) begin
          state                 <=  IDLE;
        end
        if (data_count < i_data_count) begin
          if (!buffered_read_stb) begin
            read_data           <=  8'hFF;
          end

          if (i_data_rd_stb && !buffered_read_stb) begin
            read_data           <=  8'h00;
          end
          if (i_data_rd_stb) begin
            crc_data            <=  i_data_rd_data;
          end
          //Is there a read strobe?
          if (buffered_read_stb) begin
            //It's okay if we start capturing the CRC when data is 0, it will not modify the outcome

            //Is this the first byte?
            read_data           <=  buffered_read_data;
            data_count          <=  data_count + 1;
          end
          if (data_count == i_data_count - 1) begin
            crc_enable          <=  0;
          end
      
        end
        else begin
          crc_enable            <=  0;
          state                 <=  WRITE_CRC;
          data_count            <=  0;
          read_data             <=  {crc_out0[15], crc_out1[15], crc_out2[15], crc_out3[15],
                                     crc_out0[14], crc_out1[14], crc_out2[14], crc_out3[14]};

          host_crc[0]           <=  {crc_out[0][13:0], 2'b00};
          host_crc[1]           <=  {crc_out[1][13:0], 2'b00};
          host_crc[2]           <=  {crc_out[2][13:0], 2'b00};
          host_crc[3]           <=  {crc_out[3][13:0], 2'b00};

          o_crc0_gen            <=  crc_out[0];
          o_crc1_gen            <=  crc_out[1];
          o_crc2_gen            <=  crc_out[2];
          o_crc3_gen            <=  crc_out[3];

        end
      end
      CRC: begin
        if (data_count < (`CRC_COUNT - 1)) begin
          data_count            <=  data_count + 1;
          host_crc[0]           <=  {host_crc[0][13:0], i_sdio_data_in[7], i_sdio_data_in[3]};
          host_crc[1]           <=  {host_crc[1][13:0], i_sdio_data_in[6], i_sdio_data_in[2]};
          host_crc[2]           <=  {host_crc[2][13:0], i_sdio_data_in[5], i_sdio_data_in[1]};
          host_crc[3]           <=  {host_crc[3][13:0], i_sdio_data_in[4], i_sdio_data_in[0]};
        end
        else begin
          o_crc0_rmt            <=  host_crc[0];
          o_crc1_rmt            <=  host_crc[1];
          o_crc2_rmt            <=  host_crc[2];
          o_crc3_rmt            <=  host_crc[3];

          o_crc0_gen            <=  crc_out[0];
          o_crc1_gen            <=  crc_out[1];
          o_crc2_gen            <=  crc_out[2];
          o_crc3_gen            <=  crc_out[3];

          state                 <=  FINISHED;
        end
      end
      WRITE_CRC: begin
        if (data_count < `CRC_COUNT) begin
          data_count            <=  data_count + 1;
          read_data             <=  {host_crc[0][15], host_crc[1][15], host_crc[2][15], host_crc[3][15],
                                     host_crc[0][14], host_crc[1][14], host_crc[2][14], host_crc[3][14]};
          host_crc[0]           <=  {host_crc[0][13:0], 2'b00};
          host_crc[1]           <=  {host_crc[1][13:0], 2'b00};
          host_crc[2]           <=  {host_crc[2][13:0], 2'b00};
          host_crc[3]           <=  {host_crc[3][13:0], 2'b00};
        end
        else begin
          read_data             <=  8'hFF;
          state                 <=  FINISHED;
        end
      end
      FINISHED: begin
        o_finished              <=  1;
        o_data_hst_rdy          <=  0;
        read_data               <=  8'hFF;
        o_sdio_data_dir         <=  0;
        o_data_crc_good         <=  data_crc_good;
        if (!i_activate) begin
          state                 <=  IDLE;
        end
      end
      default: begin
        local_rst               <=  1;
        if (!i_activate) begin
          state                 <=  IDLE;
        end
      end
    endcase
  end
end
endmodule
