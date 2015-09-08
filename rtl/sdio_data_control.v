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
 * Author: David McCoy  (dave.mccoy@cospandesign.com)
 * Description: Arbitrates access to the command bus data interface and the
 *  Data Bus interface
 *
 * Changes:
 *  2015.09.07: Initial Commit
 */

module sdio_data_control (
  input                     rst,
  input           [3:0]     i_func_sel,
  input                     i_mem_sel,
  input                     i_cmd_bus_sel,  /* If this is high we can only read/write one byte */

  //Command Bus Interface
  input                     i_cmd_wr_stb,
  input           [7:0]     i_cmd_wr_data,
  output                    o_cmd_rd_stb,
  output          [7:0]     o_cmd_rd_data,
  input                     i_cmd_hst_rdy,  /* CMD -> Func: Ready for receive data */
  output                    o_cmd_com_rdy,
  input                     i_cmd_activate,
  output                    o_cmd_finished,

  //Phy Data Bus Inteface
  input                     i_data_phy_wr_stb,
  input           [7:0]     i_data_phy_wr_data,
  output                    o_data_phy_rd_stb,
  output          [7:0]     o_data_phy_rd_data,
  input                     i_data_phy_hst_rdy,  /* DATA PHY -> Func: Ready for receive data */
  output                    o_data_phy_com_rdy,
  input                     i_data_phy_activate,
  output                    o_data_phy_finished,

  //CIA Interface
  output  reg               o_cia_wr_stb,
  output  reg     [7:0]     o_cia_wr_data,
  input                     i_cia_rd_stb,
  input           [7:0]     i_cia_rd_data,
  output  reg               o_cia_hst_rdy,
  input                     i_cia_com_rdy,
  output  reg               o_cia_activate,
  input                     i_cia_finished,

  //Function 1 Interface
  output  reg               o_func1_wr_stb,
  output  reg     [7:0]     o_func1_wr_data,
  input                     i_func1_rd_stb,
  input           [7:0]     i_func1_rd_data,
  output  reg               o_func1_hst_rdy,
  input                     i_func1_com_rdy,
  output  reg               o_func1_activate,
  input                     i_func1_finished,

  //Function 2 Interface
  output  reg               o_func2_wr_stb,
  output  reg     [7:0]     o_func2_wr_data,
  input                     i_func2_rd_stb,
  input           [7:0]     i_func2_rd_data,
  output  reg               o_func2_hst_rdy,
  input                     i_func2_com_rdy,
  output  reg               o_func2_activate,
  input                     i_func2_finished,

  //Function 3 Interface
  output  reg               o_func3_wr_stb,
  output  reg     [7:0]     o_func3_wr_data,
  input                     i_func3_rd_stb,
  input           [7:0]     i_func3_rd_data,
  output  reg               o_func3_hst_rdy,
  input                     i_func3_com_rdy,
  output  reg               o_func3_activate,
  input                     i_func3_finished,

  //Function 4 Interface
  output  reg               o_func4_wr_stb,
  output  reg     [7:0]     o_func4_wr_data,
  input                     i_func4_rd_stb,
  input           [7:0]     i_func4_rd_data,
  output  reg               o_func4_hst_rdy,
  input                     i_func4_com_rdy,
  output  reg               o_func4_activate,
  input                     i_func4_finished,

  //Function 5 Interface
  output  reg               o_func5_wr_stb,
  output  reg     [7:0]     o_func5_wr_data,
  input                     i_func5_rd_stb,
  input           [7:0]     i_func5_rd_data,
  output  reg               o_func5_hst_rdy,
  input                     i_func5_com_rdy,
  output  reg               o_func5_activate,
  input                     i_func5_finished,

  //Function 6 Interface
  output  reg               o_func6_wr_stb,
  output  reg     [7:0]     o_func6_wr_data,
  input                     i_func6_rd_stb,
  input           [7:0]     i_func6_rd_data,
  output  reg               o_func6_hst_rdy,
  input                     i_func6_com_rdy,
  output  reg               o_func6_activate,
  input                     i_func6_finished,

  //Function 7 Interface
  output  reg               o_func7_wr_stb,
  output  reg     [7:0]     o_func7_wr_data,
  input                     i_func7_rd_stb,
  input           [7:0]     i_func7_rd_data,
  output  reg               o_func7_hst_rdy,
  input                     i_func7_com_rdy,
  output  reg               o_func7_activate,
  input                     i_func7_finished,

  //Memory Interface
  output  reg               o_mem_wr_stb,
  output  reg     [7:0]     o_mem_wr_data,
  input                     i_mem_rd_stb,
  input           [7:0]     i_mem_rd_data,
  output  reg               o_mem_hst_rdy,
  input                     i_mem_com_rdy,
  output  reg               o_mem_activate,
  input                     i_mem_finished

);
//local parameters
//registes/wires
reg                           rd_stb;
reg   [7:0]                   rd_data;
wire                          wr_stb;
wire  [7:0]                   wr_data;
wire                          hst_rdy;
reg                           com_rdy;
wire  [3:0]                   func_select;
wire                          activate;
reg                           finished;

//submodules
//asynchronous logic
assign  o_cmd_rd_stb        = i_cmd_bus_sel ? rd_stb : 1'b0;
assign  o_cmd_rd_data       = i_cmd_bus_sel ? rd_data : 8'h00;
assign  o_cmd_com_rdy       = i_cmd_bus_sel ? com_rdy : 1'b0;
assign  o_cmd_finished      = i_cmd_bus_sel ? finished : 1'b0;

assign  o_data_phy_rd_stb   = !i_cmd_bus_sel ? rd_stb : 1'b0;
assign  o_data_phy_rd_data  = !i_cmd_bus_sel ? rd_data : 8'h00;
assign  o_data_com_rdy      = !i_cmd_bus_sel ? com_rdy : 1'b0;

assign  wr_stb              = i_cmd_bus_sel ? i_cmd_wr_stb : i_data_phy_wr_stb;
assign  wr_data             = i_cmd_bus_sel ? i_cmd_wr_data : i_data_phy_wr_data;
assign  hst_rdy             = i_cmd_bus_sel ? i_cmd_hst_rdy : i_data_phy_hst_rdy;
assign  activate            = i_cmd_bus_sel ? i_cmd_activate : i_data_phy_activate;

assign  func_select         = i_mem_sel   ? 4'h8 : {1'b0, i_func_sel};

//Multiplexer: Cmd Layer, Data Phy Layer -> Func Layer
always @ (*) begin
  if (rst) begin
    o_cia_wr_stb             = 0;
    o_cia_wr_data            = 0;
    o_cia_hst_rdy            = 0;

    o_func1_wr_stb           = 0;
    o_func1_wr_data          = 0;
    o_func1_hst_rdy          = 0;

    o_func2_wr_stb           = 0;
    o_func2_wr_data          = 0;
    o_func2_hst_rdy          = 0;

    o_func3_wr_stb           = 0;
    o_func3_wr_data          = 0;
    o_func3_hst_rdy          = 0;

    o_func4_wr_stb           = 0;
    o_func4_wr_data          = 0;
    o_func4_hst_rdy          = 0;

    o_func5_wr_stb           = 0;
    o_func5_wr_data          = 0;
    o_func5_hst_rdy          = 0;

    o_func6_wr_stb           = 0;
    o_func6_wr_data          = 0;
    o_func6_hst_rdy          = 0;

    o_func7_wr_stb           = 0;
    o_func7_wr_data          = 0;
    o_func7_hst_rdy          = 0;

    o_mem_wr_stb             = 0;
    o_mem_wr_data            = 0;
    o_mem_hst_rdy            = 0;
  end
  else begin
    //If nothing overrides these values, just set them to zero
    o_cia_wr_stb             = 0;
    o_cia_wr_data            = 0;
    o_cia_hst_rdy            = 0;
    o_cia_activate           = 0;

    o_func1_wr_stb           = 0;
    o_func1_wr_data          = 0;
    o_func1_hst_rdy          = 0;
    o_func1_activate         = 0;

    o_func2_wr_stb           = 0;
    o_func2_wr_data          = 0;
    o_func2_hst_rdy          = 0;
    o_func2_activate         = 0;

    o_func3_wr_stb           = 0;
    o_func3_wr_data          = 0;
    o_func3_hst_rdy          = 0;
    o_func3_activate         = 0;

    o_func4_wr_stb           = 0;
    o_func4_wr_data          = 0;
    o_func4_hst_rdy          = 0;
    o_func4_activate         = 0;

    o_func5_wr_stb           = 0;
    o_func5_wr_data          = 0;
    o_func5_hst_rdy          = 0;
    o_func5_activate         = 0;

    o_func6_wr_stb           = 0;
    o_func6_wr_data          = 0;
    o_func6_hst_rdy          = 0;
    o_func6_activate         = 0;

    o_func7_wr_stb           = 0;
    o_func7_wr_data          = 0;
    o_func7_hst_rdy          = 0;
    o_func7_activate         = 0;

    o_mem_wr_stb             = 0;
    o_mem_wr_data            = 0;
    o_mem_hst_rdy            = 0;
    o_mem_activate           = 0;

    case (func_select)
      0: begin
        o_cia_wr_stb         = wr_stb;
        o_cia_wr_data        = wr_data;
        o_cia_hst_rdy        = hst_rdy;
        o_cia_activate       = activate;
      end                    
      1: begin               
        o_func1_wr_stb       = wr_stb;
        o_func1_wr_data      = wr_data;
        o_func1_hst_rdy      = hst_rdy;
        o_func1_activate     = activate;
      end                    
      2: begin               
        o_func2_wr_stb       = wr_stb;
        o_func2_wr_data      = wr_data;
        o_func2_hst_rdy      = hst_rdy;
        o_func2_activate     = activate;
      end                    
      3: begin               
        o_func3_wr_stb       = wr_stb;
        o_func3_wr_data      = wr_data;
        o_func3_hst_rdy      = hst_rdy;
        o_func3_activate     = activate;
      end                    
      4: begin               
        o_func4_wr_stb       = wr_stb;
        o_func4_wr_data      = wr_data;
        o_func4_hst_rdy      = hst_rdy;
        o_func4_activate     = activate;
      end                    
      5: begin               
        o_func5_wr_stb       = wr_stb;
        o_func5_wr_data      = wr_data;
        o_func5_hst_rdy      = hst_rdy;
        o_func5_activate     = activate;
      end                    
      6: begin               
        o_func6_wr_stb       = wr_stb;
        o_func6_wr_data      = wr_data;
        o_func6_hst_rdy      = hst_rdy;
        o_func6_activate     = activate;
      end                    
      7: begin               
        o_func7_wr_stb       = wr_stb;
        o_func7_wr_data      = wr_data;
        o_func7_hst_rdy      = hst_rdy;
        o_func7_activate     = activate;
      end                    
      8: begin               
        o_mem_wr_stb         = wr_stb;
        o_mem_wr_data        = wr_data;
        o_mem_hst_rdy        = hst_rdy;
        o_mem_activate     = activate;
      end
      default: begin
      end
    endcase
  end
end

//Multiplexer: Func -> Cmd Layer, Data Phy Layer
always @ (*) begin
  if (rst) begin
    rd_stb                  = 0;
    rd_data                 = 0;
    com_rdy                 = 0;

  end
  else begin
    case (func_select)
      0: begin
        rd_stb            = i_cia_rd_stb;
        rd_data           = i_cia_rd_data;
        com_rdy           = i_cia_com_rdy;
        finished          = i_cia_finished;
      end
      1: begin
        rd_stb            = i_func1_rd_stb;
        rd_data           = i_func1_rd_data;
        com_rdy           = i_func1_com_rdy;
        finished          = i_func1_finished;
      end
      2: begin
        rd_stb            = i_func2_rd_stb;
        rd_data           = i_func2_rd_data;
        com_rdy           = i_func2_com_rdy;
        finished          = i_func2_finished;
      end
      3: begin
        rd_stb            = i_func3_rd_stb;
        rd_data           = i_func3_rd_data;
        com_rdy           = i_func3_com_rdy;
        finished          = i_func3_finished;
      end
      4: begin
        rd_stb            = i_func4_rd_stb;
        rd_data           = i_func4_rd_data;
        com_rdy           = i_func4_com_rdy;
        finished          = i_func4_finished;
      end
      5: begin
        rd_stb            = i_func5_rd_stb;
        rd_data           = i_func5_rd_data;
        com_rdy           = i_func5_com_rdy;
        finished          = i_func5_finished;
      end
      6: begin
        rd_stb            = i_func6_rd_stb;
        rd_data           = i_func6_rd_data;
        com_rdy           = i_func6_com_rdy;
        finished          = i_func6_finished;
      end
      7: begin
        rd_stb            = i_func7_rd_stb;
        rd_data           = i_func7_rd_data;
        com_rdy           = i_func7_com_rdy;
        finished          = i_func7_finished;
      end
      8: begin
        rd_stb            = i_mem_rd_stb;
        rd_data           = i_mem_rd_data;
        com_rdy           = i_mem_com_rdy;
        finished          = i_mem_finished;
      end
      default: begin
        rd_stb            = 1'b0;
        rd_data           = 8'h0;
        com_rdy           = 1'b0;
      end
    endcase
  end
end

//synchronous logic

endmodule
