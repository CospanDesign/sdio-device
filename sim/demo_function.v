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
 * Author: Demo Function
 * Description: Demonstrate all possible characteristics of a function
 *  including
 *      Single Byte Write
 *      Single Byte Read
 *      Multiple Byte Write
 *      Multiple Byte Read
 *      Block Write
 *      Block Read
 *      Incrementing Address Write
 *      Non Incrementing Address Write
 *      Incrementing Address Read
 *      Non Incrementing Address Read
 *
 *      Test Interrupt
 *          Enable
 *          Interrupt Action
 *          Interrupt Pending
 *
 *      Read Wait
 *          System Response to a read wait
 *
 *  Note: All function configuration needs to be done inside
 *      sdio_configuration.json file
 *
 *  Parameters:
 *      FUN_NUM: The function index to respond to, e.g. 1 for function 1
 *
 * Changes:
 */

`include "sdio_cia_defines.v"

module demo_function #(
  parameter                 FUNC_NUM = 1
)(
  input                     clk,
  input                     sdio_clk,
  input                     rst,

  //Configuration
  input                     i_csa_en,
  input         [3:0]       i_pwr_mode,
  input         [15:0]      i_block_size,           //Maximum Size of block transfer
  input                     i_enable,               //Host says they want to use the function
  output                    o_ready,                //Function is ready to accept data/command
  input                     i_abort,                //Host asks to abort a transaction
  output                    o_execution_status,     //Currently Executing a Command
  output                    o_ready_for_data,       //Function is ready to receive data
  output                    o_busy,                 //Current status of the function

  //Transaction Configuration
  input                     i_activate,             //Activate for a data transfer
  output    reg             o_finished,             //Function asserts this signal when it is finished
  input                     i_inc_addr,             //Function will increment address for each conecutive r/w
  input                     i_block_mode,           //0 = Transfering bytes at a time, 1 = transfering at a block

  //Transaction
  input                     i_write_flag,           //Read = 0, Write = 1
  input                     i_rd_after_wr,          //Read the value after write, only valid for 1 byte
  input         [17:0]      i_addr,                 //Address within Function
  input         [7:0]       i_write_data,           //Data to write
  output    reg [7:0]       o_read_data,            //Data to read
  output    reg             o_data_rdy,             //Function is ready to receive up to block size of data
  input                     i_data_stb,             //Host is sending a byte of data
  input                     i_host_rdy,             //Host is ready to receive data from function
  input         [12:0]      i_data_count,           //Number of bytes or blocks to read/write
  output    reg             o_data_stb,             //SDIO sends a piece of data to host

  //Out of band signaling
//  output                    o_read_wait,            //When asserted function is not ready to receive more data
  output                    o_interrupt,            //Launch off an interrupt

  //Test Stimulate
//  input                     i_request_read_wait,    //Used for Simulation to test read wait
  input                     i_request_interrupt     //Used for simulation to test interrupt

);
//local parameters
localparam      IDLE        =   4'h0;
localparam      WRITE       =   4'h1;
localparam      READ_START  =   4'h2;
localparam      READ        =   4'h3;
localparam      FINISHED    =   4'h4;

//registes/wires
reg             [3:0]       state;
reg             [11:0]      mem_addr;
reg                         write_stb;
reg                         mem_write_stb;
reg             [31:0]      mem_write_data;
wire            [31:0]      mem_read_data;
reg             [12:0]      data_count;
reg             [1:0]       byte_count;
reg             [31:0]      block_data_count;
wire                        count_finished;
reg             [31:0]      address;

reg             [31:0]      write_data;
reg             [31:0]      read_data;

//submodules
blk_mem #(
    .DATA_WIDTH             (32             ),  //More Challenging but this is going to be more like final applications
    .ADDRESS_WIDTH          (12             ),  //4096
    .INC_NUM_PATTERN        (1              )   //Incrementing data pattern to read and modify
)mem(
    .clka                   (sdio_clk       ),
    .wea                    (mem_write_stb  ),
    .addra                  (mem_addr       ),
    .dina                   (mem_write_data ),
    //.clkb                   (blk            ),
    .clkb                   (sdio_clk       ),
    .addrb                  (mem_addr       ),
    .doutb                  (mem_read_data  )
);
//asynchronous logic
assign  o_busy              =   state != IDLE;
//assign  count_finished      =   i_block_mode ?  (block_data_count >= i_data_count) :
//                                                (data_count       >= i_data_count);
assign  count_finished      =   (data_count >= i_data_count);
//assign  o_read_wait         =   (`SRW && i_request_read_wait);
assign  o_interrupt         =   i_request_interrupt;
assign  o_ready             =   i_enable;
assign  o_execution_status  =   o_busy;
assign  o_ready_for_data    =   1'b1;
//assign  mem_write_data      =   {write_data[23:0], i_write_data};

//synchronous logic
/*
always @ (*) begin
  if(rst) begin
    o_read_data             <=  8'h0;
  end
  else begin
    case (byte_count)
      2'b00:
        o_read_data         <=  mem_read_data[7:0];
      2'b01:
        o_read_data         <=  mem_read_data[15:8];
      2'b10:
        o_read_data         <=  mem_read_data[23:16];
      2'b11:
        o_read_data         <=  mem_read_data[31:24];
    endcase
  end
end
*/

//Counter
//XXX: This may need to be a mealy state machine
/*
always @ (posedge sdio_clk) begin
  if (rst) begin
    data_count              <=  0;
    block_data_count        <=  0;
  end
  else begin
    if (state == IDLE) begin
        data_count          <=  0;
        block_data_count    <=  0;
    end
    else if ((state == WRITE) || (state == READ)) begin
      if (i_data_stb || o_data_stb) begin
        if (i_block_mode) begin
          if (data_count < i_block_size) begin
            data_count            <=  data_count + 13'h1;
          end
          else begin
            if (block_data_count  < data_count) begin
              block_data_count    <=  block_data_count + 1;
              data_count          <=  13'h0;
            end
          end
        end
        else begin
          data_count              <=  data_count + 13'h1;
        end
      end
    end
  end
end
*/

//Main State Machine
always @ (posedge sdio_clk) begin
  mem_write_stb             <=  0;
  write_stb                 <=  0;
  o_data_stb                <=  0;
  if (rst) begin
    byte_count              <=  0;
    //mem_write_data          <=  0;
    o_finished              <=  0;
    o_data_rdy              <=  0;
    write_data              <=  0;
    address                 <=  0;
    mem_addr                <=  0;
    o_read_data             <=  0;
    read_data               <=  0;

    state                   <=  IDLE;
    data_count              <=  0;
    block_data_count        <=  0;
    mem_write_data          <=  0;
  end
  else begin
    if (write_stb) begin
      mem_write_stb         <=  1;
      address               <=  address + 1;
    end
    case (state)
      IDLE: begin
        byte_count          <=  0;
        o_data_rdy          <=  0;
        o_finished          <=  0;
        mem_addr            <=  0;
        data_count          <=  0;
        if (i_activate)begin
          address           <=  i_addr[12:2];
          mem_addr          <=  i_addr[12:2];
          o_data_rdy        <=  1;
          if (i_write_flag) begin
            state           <=  WRITE;
          end
          else begin
            state           <=  READ_START;
          end
        end
      end
      WRITE: begin
        if (data_count < i_data_count) begin
          if (i_data_stb) begin
            //Shift Data In
            write_data          <= {write_data[23:0], i_write_data};
            case (byte_count)
              0: begin
              end
              1: begin
              end
              2: begin
                mem_addr          <= address;
              end
              3: begin
                write_stb         <=  1;
                data_count        <= data_count + 1;
                mem_write_data    <= {write_data[23:0], i_write_data};
              end
            endcase
            byte_count          <= byte_count + 2'b01;
          end
        end
        else begin
          state             <= FINISHED;
        end
      end
      READ_START: begin
        if (i_host_rdy) begin
          read_data         <=  mem_read_data;
          o_read_data       <=  mem_read_data[31:24];
          byte_count        <=  0;
          //o_data_stb        <=  1;
          state             <=  READ;
        end
      end
      READ: begin
        o_data_stb        <= 1;
        case (byte_count)
          0: begin
            o_read_data     <=  read_data[31:24];
            mem_addr        <=  mem_addr + 1;
          end
          1: begin
            o_read_data     <=  read_data[23:16];
          end
          2: begin
            o_read_data     <=  read_data[15:8];
            data_count      <=  data_count + 1;
          end
          3: begin
            o_read_data     <=  read_data[7:0];
            //mem_addr        <=  address;
            //address         <=  address + 1;
            if (data_count < i_data_count[12:2]) begin
              read_data     <=  mem_read_data;
            end
            else begin
              state         <= FINISHED;
            end
          end

        endcase
        byte_count        <=  byte_count + 2'b01;
      end
      FINISHED: begin
        o_data_rdy          <=  0;
        o_finished          <=  1;
        if (!i_activate) begin
          state             <=  IDLE;
        end
      end
      default: begin
          state             <=  IDLE;
      end
    endcase
  end
end


endmodule
