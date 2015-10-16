///////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2009 Xilinx, Inc.
// This design is confidential and proprietary of Xilinx, All Rights Reserved.
///////////////////////////////////////////////////////////////////////////////
//   ____  ____
//  /   /\/   /
// /___/  \  /   Vendor: Xilinx
// \   \   \/    Version: 1.0
//  \   \        Filename: serdes_1_to_n_clk_ddr_s8_diff.v
//  /   /        Date Last Modified:  November 5 2009
// /___/   /\    Date Created: September 1 2009
// \   \  /  \
//  \___\/\___\
//
//Device:   Spartan 6
//Purpose:    1-bit generic 1:n DDR clock receiver module for serdes factors
//    from 2 to 8 with differential inputs
//    Instantiates necessary BUFIO2 clock buffers
//Reference:
//
//Revision History:
//    Rev 1.0 - First created (nicks)
///////////////////////////////////////////////////////////////////////////////
//
//  Disclaimer:
//
//    This disclaimer is not a license and does not grant any rights to the materials
//              distributed herewith. Except as otherwise provided in a valid license issued to you
//              by Xilinx, and to the maximum extent permitted by applicable law:
//              (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND WITH ALL FAULTS,
//              AND XILINX HEREBY DISCLAIMS ALL WARRANTIES AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY,
//              INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-INFRINGEMENT, OR
//              FITNESS FOR ANY PARTICULAR PURPOSE; and (2) Xilinx shall not be liable (whether in contract
//              or tort, including negligence, or under any other theory of liability) for any loss or damage
//              of any kind or nature related to, arising under or in connection with these materials,
//              including for any direct, or any indirect, special, incidental, or consequential loss
//              or damage (including loss of data, profits, goodwill, or any type of loss or damage suffered
//              as a result of any action brought by a third party) even if such damage or loss was
//              reasonably foreseeable or Xilinx had been advised of the possibility of the same.
//
//  Critical Applications:
//
//    Xilinx products are not designed or intended to be fail-safe, or for use in any application
//    requiring fail-safe performance, such as life-support or safety devices or systems,
//    Class III medical devices, nuclear facilities, applications related to the deployment of airbags,
//    or any other applications that could lead to death, personal injury, or severe property or
//    environmental damage (individually and collectively, "Critical Applications"). Customer assumes
//    the sole risk and liability of any use of Xilinx products in Critical Applications, subject only
//    to applicable laws and regulations governing limitations on product liability.
//
//  THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS PART OF THIS FILE AT ALL TIMES.
//
//////////////////////////////////////////////////////////////////////////////



/*
 * Author:
 * Description:
 *
 * Changes:
 */

`define SWAP_CLK  0
module sd_dev_platform_spartan6 #(
  parameter                 OUTPUT_DELAY  = 63,
  parameter                 INPUT_DELAY   = 63
)(
  input                     rst,
  input                     clk,

  //SD Stack Interfface
  output                    o_locked,

  output                    o_sd_clk,
  output                    o_sd_clk_x2,

  output  reg               o_posedge_stb,

  input                     i_sd_cmd_dir,
  input                     i_sd_cmd_out,
  output                    o_sd_cmd_in,

  input                     i_sd_data_dir,
  input           [7:0]     i_sd_data_out,
  output          [7:0]     o_sd_data_in,

  input                     i_phy_clk,
  inout                     io_phy_sd_cmd,
  inout           [3:0]     io_phy_sd_data
);

//local parameters
//registes/wires
wire                [7:0]   sd_data_out;
wire                        pll_sd_clk;

wire                        ddr_clk_predelay;
wire                        ddr_clk_delay;

wire                        ddr_clk;

wire                        sd_cmd_tristate_dly;
wire                        sd_cmd_out_delay;
wire                        sd_cmd_in_delay;

wire                [3:0]   pin_data_out;
wire                [3:0]   pin_data_in;
wire                [3:0]   pin_data_tristate;

wire                [3:0]   pin_data_out_delay;
wire                [3:0]   pin_data_in_predelay;
wire                [3:0]   pin_data_tristate_predelay;
wire                        serdes_strobe;

wire                        phy_clk_s;
wire                        pll_phy_clk;
wire                        clkfb;
wire                        fb_locked;
wire                        serdes_clk_fb;

`ifdef SIMULATION
pullup (io_phy_sd_data[0]);
pullup (io_phy_sd_data[1]);
pullup (io_phy_sd_data[2]);
pullup (io_phy_sd_data[3]);
`endif

assign    fb_locked       = o_locked && pll_locked;
//assign    o_posedge_stb   = serdes_strobe;
//submodules

//Read in the clock
wire  buf_clk;
wire  buf_phy_clk_p;
wire  buf_phy_clk_n;
wire  predelay_clk_p;
wire  predelay_clk_n;

IBUFG input_clock_buffer_p(
  .I                    (i_phy_clk            ),
  .O                    (buf_phy_clk_p        )
);

assign  predelay_clk_p  = buf_phy_clk_p ^ `SWAP_CLK;

IODELAY2 # (
  .DATA_RATE            ("SDR"                ),
  .SIM_TAPDELAY_VALUE   (49                   ),
  .IDELAY_VALUE         (0                    ),
  .IDELAY2_VALUE        (0                    ),
  .ODELAY_VALUE         (0                    ),
  .IDELAY_MODE          ("NORMAL"             ),
  .SERDES_MODE          ("NONE"               ),
  .IDELAY_TYPE          ("FIXED"              ),
  .COUNTER_WRAPAROUND   ("STAY_AT_LIMIT"      ),
  .DELAY_SRC            ("IDATAIN"            )
) clock_iodelay (
  .IDATAIN              (predelay_clk_p       ),
  .TOUT                 (                     ),
  .DOUT                 (                     ),
  .T                    (1'b1                 ),
  .ODATAIN              (1'b0                 ),
  .DATAOUT              (clock_delay          ),
  .DATAOUT2             (                     ),
  .IOCLK0               (1'b0                 ),
  .IOCLK1               (1'b0                 ),
  .CLK                  (1'b0                 ),
  .CAL                  (1'b0                 ),
  .INC                  (1'b0                 ),
  .CE                   (1'b0                 ),
  .RST                  (1'b0                 ),
  .BUSY                 (                     )
);

BUFIO2 #(
  .DIVIDE               (1                    ),
  .I_INVERT             ("FALSE"              ),
  .DIVIDE_BYPASS        ("FALSE"              ),
  .USE_DOUBLER          ("FALSE"              )
) bufio2_clk (
  .I                    (clock_delay          ),
  .IOCLK                (pll_serdes_clk       ),
  .DIVCLK               (pll_sd_clk           ),
  .SERDESSTROBE         (serdes_strobe        )
);

BUFIO2 #(
  .DIVIDE               (1                    ),
  .I_INVERT             ("TRUE"               ),
  .DIVIDE_BYPASS        ("FALSE"              ),
  .USE_DOUBLER          ("FALSE"              )
) bufio2_clk_n (
  .I                    (clock_delay          ),
  .IOCLK                (pll_serdes_clk_n     ),
  .DIVCLK               (pll_sd_clk_n         ),
  .SERDESSTROBE         (serdes_strobe_n      )
);


/*
BUFIO2_2CLK #(
//  .DIVIDE               (1                    )
//  .I_INVERT             ("FALSE"              ),
//  .DIVIDE_BYPASS        ("FALSE"              ),
//  .USE_DOUBLER          ("FALSE"               )
) bufio2_clk (
  .I                    (clock_delay          ),
  .IOCLK                (pll_serdes_clk       ),
  .DIVCLK               (pll_sd_clk           ),
  .SERDESSTROBE         (serdes_strobe        )
);
*/




//Clock will be used to drive both the output and the internal state machine
BUFG sd_clk_bufg(
  .I                    (pll_sd_clk           ),
  .O                    (o_sd_clk             )
);
BUFG sd_clk_x2_bufg(
  .I                    (pll_serdes_clk       ),
  .O                    (o_sd_clk_x2          )
);

ISERDES2 #(
  .DATA_RATE            ("SDR"                ),  //Because we are using a PLL to generate a high speed clock we use single data rate
  .DATA_WIDTH           (2                    ),
  .BITSLIP_ENABLE       ("FALSE"              ),
  .SERDES_MODE          ("MASTER"             ),
  .INTERFACE_TYPE       ("RETIMED"            )
) clkfb_serdes_m (
  .RST                  (!fb_locked           ),
  .D                    (phy_clk_m            ),
  .CE0                  (1'b1                 ),
  .CLK0                 (serdes_clk           ),
  .CLK1                 (1'b0                 ),
  .IOCE                 (serdes_strobe        ),
  .CLKDIV               (o_sd_clk             ),
  .SHIFTIN              (                     ),
  .SHIFTOUT             (                     ),
  .FABRICOUT            (                     ),
  .CFB0                 (feedback             ),
  .CFB1                 (                     ),
  .DFB                  (data_clk_pre         ),
  .INCDEC               (                     ),
  .VALID                (                     ),
  .BITSLIP              (1'b0                 ),
  //Actual Data
  .Q1                   (                     ),
  .Q2                   (                     )
);

/*
ISERDES2 #(
  .BITSLIP_ENABLE       ("FALSE"              ),
  .DATA_RATE            ("SDR"                ),  //Because we are using a PLL to generate a high speed clock we use single data rate
  .DATA_WIDTH           (2                    ),
  .SERDES_MODE          ("SLAVE"              ),
  .INTERFACE_TYPE       ("RETIMED"            )
) clkfb_serdes_s (

  .RST                  (!fb_locked           ),
  .D                    (phy_clk_s            ),
  //.D                    (phy_clk_m            ),
  .CE0                  (1'b1                 ),
  .CLK0                 (serdes_clk           ),
  .CLK1                 (1'b0                 ),
  .IOCE                 (serdes_strobe        ),
  .CLKDIV               (o_sd_clk             ),
  .SHIFTIN              (                     ),
  .SHIFTOUT             (                     ),
  .FABRICOUT            (                     ),
  .CFB0                 (feedback             ),
  .CFB1                 (                     ),
  //.DFB                  (feedback             ),
  .DFB                  (data_clk_pre         ),
  .INCDEC               (                     ),
  .VALID                (                     ),
  .BITSLIP              (1'b0                 ),
  //Actual Data
  .Q1                   (                     ),
  .Q2                   (                     )
);
*/

BUFPLL #(
  .DIVIDE               (2                    ),
  .ENABLE_SYNC          ("TRUE"               )
) clk_buff_pll(
  .GCLK                 (o_sd_clk             ),
  .LOCKED               (pll_locked           ),
  .PLLIN                (pll_serdes_clk       ),
  .LOCK                 (o_locked             ),
  .IOCLK                (serdes_clk           ),
  .SERDESSTROBE         (serdes_strobe        )
);

//Control Line
IOBUF
#(
  .IOSTANDARD           ("LVCMOS33"           )
)
cmd_iobuf(
  .T                    (sd_cmd_tristate_dly  ),
  .O                    (sd_cmd_in_delay      ),
  .I                    (sd_cmd_out_delay     ),
  .IO                   (io_phy_sd_cmd        )
);

`ifdef SIMULATION
pullup (io_phy_sd_cmd);
`endif

IODELAY2 #(
  .DATA_RATE            ("SDR"                ),
  .IDELAY_VALUE         (INPUT_DELAY          ),
  .ODELAY_VALUE         (OUTPUT_DELAY         ),
  .IDELAY_TYPE          ("FIXED"              ),
  .COUNTER_WRAPAROUND   ("STAY_AT_LIMIT"      ),
  .DELAY_SRC            ("IO"                 ),
  .SERDES_MODE          ("NONE"               ),
  .SIM_TAPDELAY_VALUE   (75                   )
) cmd_delay (
  .T                    (!i_sd_cmd_dir        ),
  .ODATAIN              (i_sd_cmd_out         ),
  //.DATAOUT              (o_sd_cmd_in          ),
  .DATAOUT2             (o_sd_cmd_in          ),
  //FPGA Fabric
  //IOB
  .TOUT                 (sd_cmd_tristate_dly  ),
  .IDATAIN              (sd_cmd_in_delay      ),
  .DOUT                 (sd_cmd_out_delay     ),

  .IOCLK0               (1'b0                 ),  //XXX: This one is not SERDESized.. Do I need to add a clock??
  .IOCLK1               (1'b0                 ),
  .CLK                  (1'b0                 ),
  .CAL                  (1'b0                 ),
  .INC                  (1'b0                 ),
  .CE                   (1'b0                 ),
  .BUSY                 (                     ),
  .RST                  (1'b0                 )
);

//DATA Lines
genvar pcnt;
generate
for (pcnt = 0; pcnt < 4; pcnt = pcnt + 1) begin: sgen
IOBUF #(
  .IOSTANDARD           ("LVCMOS33"             )
) io_data_buffer (
  .T                    (pin_data_tristate[pcnt]),

  .I                    (pin_data_out[pcnt]     ),
  .O                    (pin_data_in[pcnt]      ),

  .IO                   (io_phy_sd_data[pcnt]   )
);

IODELAY2 #(
  .DATA_RATE            ("SDR"                ),
  .IDELAY_VALUE         (INPUT_DELAY          ),
  .ODELAY_VALUE         (OUTPUT_DELAY         ),
  .IDELAY_TYPE          ("FIXED"              ),
  .COUNTER_WRAPAROUND   ("STAY_AT_LIMIT"      ),
  .DELAY_SRC            ("IO"                 ),
  .SERDES_MODE          ("NONE"               ),
  .SIM_TAPDELAY_VALUE   (75                   )
)sd_data_delay(
  //IOSerdes
  .T                    (pin_data_tristate_predelay[pcnt] ),
  .ODATAIN              (pin_data_in_predelay[pcnt]       ),
  .DATAOUT              (pin_data_out_delay[pcnt]         ),
  //To/From IO Buffer
  .TOUT                 (pin_data_tristate[pcnt]          ),
  .IDATAIN              (pin_data_in[pcnt]                ),
  .DOUT                 (pin_data_out[pcnt]               ),
  .DATAOUT2             (                     ),
  .IOCLK0               (1'b0                 ),  //This one is not SERDESized.. Do I need to add a clock??
  .IOCLK1               (1'b0                 ),
  .CLK                  (1'b0                 ),
  .CAL                  (1'b0                 ),
  .INC                  (1'b0                 ),
  .CE                   (1'b0                 ),
  .BUSY                 (                     ),
  .RST                  (1'b0                 )
);

ISERDES2 #(
  .BITSLIP_ENABLE       ("FALSE"              ),
  .DATA_RATE            ("SDR"                ),  //Because we are using a PLL to generate a high speed clock we use single data rate
  .DATA_WIDTH           (2                    ),
  .SERDES_MODE          ("NONE"               ),
  .INTERFACE_TYPE       ("NETWORKING"         )
) data_in_serdes (

  .RST                  (rst                  ),
  .SHIFTIN              (                     ),
  .SHIFTOUT             (                     ),
  .FABRICOUT            (                     ),
  .CFB0                 (                     ),
  .CFB1                 (                     ),
  .DFB                  (                     ),
  .INCDEC               (                     ),
  .VALID                (                     ),
  .BITSLIP              (1'b0                 ),
  .CE0                  (1'b1                 ),
  .CLK0                 (serdes_clk           ),
  .CLK1                 (1'b0                 ),
  .CLKDIV               (o_sd_clk             ),
  .IOCE                 (serdes_strobe        ),

  //Actual Data
  .D                    (pin_data_out_delay[pcnt]),
  .Q1                   (o_sd_data_in[pcnt]     ),
  .Q2                   (o_sd_data_in[pcnt + 4] )
);

OSERDES2 #(
  .DATA_RATE_OQ         ("SDR"                ),
  .DATA_RATE_OT         ("SDR"                ),
  .TRAIN_PATTERN        (0                    ),
  .DATA_WIDTH           (2                    ),
  .SERDES_MODE          ("NONE"               ),
  .OUTPUT_MODE          ("SINGLE_ENDED"       )
) data_out_serdes (
  .D1                   (sd_data_out[pcnt + 4]),
  .D2                   (sd_data_out[pcnt]    ),
  .T1                   (!i_sd_data_dir       ),
  .T2                   (!i_sd_data_dir       ),
  .SHIFTIN1             (1'b1                 ),
  .SHIFTIN2             (1'b1                 ),
  .SHIFTIN3             (1'b1                 ),
  .SHIFTIN4             (1'b1                 ),
  .SHIFTOUT1            (                     ),
  .SHIFTOUT2            (                     ),
  .SHIFTOUT3            (                     ),
  .SHIFTOUT4            (                     ),
  .TRAIN                (1'b0                 ),
  .OCE                  (1'b1                 ),
  .CLK0                 (serdes_clk           ),
  .CLK1                 (1'b0                 ),
  .CLKDIV               (o_sd_clk             ),

  .OQ                   (pin_data_in_predelay[pcnt]   ),
  .TQ                   (pin_data_tristate_predelay[pcnt]),
  .IOCE                 (serdes_strobe        ),
  .TCE                  (1'b1                 ),
  .RST                  (rst                  )
);

end
endgenerate

//asynchronous logic
assign  sd_data_out = i_sd_data_out;

always @ (posedge o_sd_clk_x2) begin
  o_posedge_stb     <=  serdes_strobe;
end

//Synchronous Logic
endmodule
