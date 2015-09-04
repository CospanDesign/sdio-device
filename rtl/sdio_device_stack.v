/*
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
 * Author: David McCoy
 *
 * NOTE: Add Pullups to all the signals, this will tell the host we are
 *    an SD Device, not a SPI Device
 * Description: SDIO Stack
 *  Manages the entire SDIO communication flow, in the end users should
 *  Write into the stack and it should arrive on the other side.
 *
 *  Data Link Layer:
 *    Sends and receive commands and responses from the physical layer.
 *    Manages all register read and writes that will be used to configure
 *    the entire stack.
 *
 *  Phy Layer:
 *    Sends and receives streams of bits with the host and manages CRC
 *    generation and analysis. The bottom part of this layer is connected
 *    to the physical pins of the FPGA and the top is connected to the data
 *    link layer
 *
 * Changes:
 *  2015.08.09: Inital Commit
 */

module sdio_device_stack (

  input               sdio_clk,
  input               rst,

  //Function Configuration
  output  [15:0]      o_block_size,           //Maximum Size of block transfer
  output  [7:0]       o_func_enable,          //Bitmask Function Enable
  input   [7:0]       i_func_ready,           //Bitmask Function is Ready
  output  [2:0]       o_func_abort,
  output  [7:0]       o_func_int_en,
  input   [7:0]       i_func_int_pending,
  input   [7:0]       i_func_exec_status,

  // Function Interface From CIA
  output              o_fbr1_csa_en,
  output  [3:0]       o_fbr1_pwr_mode,
  output  [15:0]      o_fbr1_block_size,

  output              o_fbr2_csa_en,
  output  [3:0]       o_fbr2_pwr_mode,
  output  [15:0]      o_fbr2_block_size,

  output              o_fbr3_csa_en,
  output  [3:0]       o_fbr3_pwr_mode,
  output  [15:0]      o_fbr3_block_size,

  output              o_fbr4_csa_en,
  output  [3:0]       o_fbr4_pwr_mode,
  output  [15:0]      o_fbr4_block_size,

  output              o_fbr5_csa_en,
  output  [3:0]       o_fbr5_pwr_mode,
  output  [15:0]      o_fbr5_block_size,

  output              o_fbr6_csa_en,
  output  [3:0]       o_fbr6_pwr_mode,
  output  [15:0]      o_fbr6_block_size,

  output              o_fbr7_csa_en,
  output  [3:0]       o_fbr7_pwr_mode,
  output  [15:0]      o_fbr7_block_size,


  //Function Bus
  output              o_func_activate,
  input               i_func_finished,

  output              o_func_inc_addr,
  output              o_func_block_mode,

  output  [3:0]       o_func_num,
  output              o_func_write_flag,
  output              o_func_rd_after_wr,
  output  [17:0]      o_func_addr,
  output  [7:0]       o_func_write_data,
  input   [7:0]       i_func_read_data,
  input               o_func_data_rdy,
  output              i_func_host_rdy,
  output  [17:0]      o_func_data_count,

  input   [7:0]       i_interrupt,

  //FPGA Interface
  input               i_sdio_clk,
/*
  inout               io_sdio_cmd,
  inout   [3:0]       io_sdio_data,
*/


  output              o_sd_cmd_dir,
  input               i_sd_cmd_in,
  output              o_sd_cmd_out,

  output              o_sd_data_dir,
  output  [7:0]       o_sd_data_out,
  input   [7:0]       i_sd_data_in

);

//local parameters
//registes/wires
wire        [3:0]   sdio_state;
wire                sdio_cmd_in;
wire                sdio_cmd_out;
wire                sdio_cmd_dir;

wire        [3:0]   sdio_data_in;
wire        [3:0]   sdio_data_out;
wire                sdio_data_dir;

//Phy Configuration
wire                spi_phy;
wire                sd1_phy;
wire                sd4_phy;

//Phy Interface
wire                cmd_phy_idle;
wire                cmd_stb;
wire                cmd_crc_stb;
wire        [5:0]   cmd;
wire        [31:0]  cmd_arg;

wire        [39:0]  rsps;
wire        [7:0]   rsps_len;

wire                interrupt;
wire                read_wait;
wire                chip_select_n;

//Function Level
wire                func_activate;
wire                func_inc_addr;
wire                func_block_mode;

wire                tunning_block;


//Submodules
sdio_card_control card_controller (
  .sdio_clk                 (sdio_clk               ),/* Run from the SDIO Clock */
  .rst                      (rst                    ),

  .o_func_activate          (func_activate          ),/* CMD -> FUNC: Start a function layer transaction */
  .i_func_finished          (func_finished          ),/* FUNC -> CMD: Function has finished */
  .o_func_inc_addr          (func_inc_addr          ),/* CMD -> FUNC: Increment the address after every read/write */
  .o_func_block_mode        (func_block_mode        ),/* CMD -> FUNC: This is a block level transfer, not byte */

  .o_func_num               (o_func_num             ),/* CMD -> FUNC: Function Number to activate */
  .o_func_write_flag        (o_func_write_flag      ),/* CMD -> FUNC: We are writing */
  .o_func_rd_after_wr       (o_func_rd_after_wr     ),/* CMD -> FUNC: Read the value after a write */
  .o_func_addr              (o_func_addr            ),/* CMD -> FUNC: Address we are talking to */
  .o_func_write_data        (o_func_write_data      ),/* CMD -> FUNC: Data to Write */
  .i_func_read_data         (i_func_read_data       ),/* FUNC -> CMD: Read Data */
  .i_func_data_rdy          (i_func_data_rdy        ),/* FUNC -> CMD: Function is ready for write data */
  .o_func_host_rdy          (o_func_host_rdy        ),/* CMD -> FUNC: Host is ready to read data */
  .o_func_data_count        (o_func_data_count      ),/* CMD -> FUNC: number of data bytes/blocks to read/write */

  .o_tunning_block          (tunning_block          ),

  // Function Interface From CIA
  .o_fbr1_csa_en            (o_fbr1_csa_en          ),
  .o_fbr1_pwr_mode          (o_fbr1_pwr_mode        ),
  .o_fbr1_block_size        (o_fbr1_block_size      ),

  .o_fbr2_csa_en            (o_fbr2_csa_en          ),
  .o_fbr2_pwr_mode          (o_fbr2_pwr_mode        ),
  .o_fbr2_block_size        (o_fbr2_block_size      ),

  .o_fbr3_csa_en            (o_fbr3_csa_en          ),
  .o_fbr3_pwr_mode          (o_fbr3_pwr_mode        ),
  .o_fbr3_block_size        (o_fbr3_block_size      ),

  .o_fbr4_csa_en            (o_fbr4_csa_en          ),
  .o_fbr4_pwr_mode          (o_fbr4_pwr_mode        ),
  .o_fbr4_block_size        (o_fbr4_block_size      ),

  .o_fbr5_csa_en            (o_fbr5_csa_en          ),
  .o_fbr5_pwr_mode          (o_fbr5_pwr_mode        ),
  .o_fbr5_block_size        (o_fbr5_block_size      ),

  .o_fbr6_csa_en            (o_fbr6_csa_en          ),
  .o_fbr6_pwr_mode          (o_fbr6_pwr_mode        ),
  .o_fbr6_block_size        (o_fbr6_block_size      ),

  .o_fbr7_csa_en            (o_fbr7_csa_en          ),
  .o_fbr7_pwr_mode          (o_fbr7_pwr_mode        ),
  .o_fbr7_block_size        (o_fbr7_block_size      ),

  .i_cmd_phy_idle           (cmd_phy_idle           ),/* PHY -> CMD: Command portion of phy layer is IDLE */
  .i_cmd_stb                (cmd_stb                ),/* PHY -> CMD: Command signal strobe */
  .i_cmd_crc_good_stb       (cmd_crc_good_stb       ),/* PHY -> CMD: CRC is good */
  .i_cmd                    (cmd                    ),/* PHY -> CMD: Command */
  .i_cmd_arg                (cmd_arg                ),/* PHY -> CMD: Command Arg */

  .i_chip_select_n          (chip_select_n          ),/* Chip Select used to determine if this is a SPI host */

  .o_rsps                   (rsps                   ),/* Response Generated by this layer*/
  .o_rsps_len               (rsps_len               ),/* Length of response*/
  .o_rsps_stb               (rsps_stb               )
);

sdio_device_phy phy(
  .rst                      (rst                    ),

  //Configuration
  .i_spi_phy                  (spi_phy                ),/* Flag: SPI PHY (not supported now) */
  .i_sd1_phy                  (sd1_phy                ),/* Flag: SD  PHY with one data lane */
  .i_sd4_phy                  (sd4_phy                ),/* Flag: SD  PHY with four data lanes */

  .o_cmd_phy_idle             (cmd_phy_idle           ),/* PHY -> CMD: Command portion of phy layer is IDLE */

  //Data Link Interface
  .o_cmd_stb                  (cmd_stb                ),/* PHY -> CMD: Command signal strobe */
  .o_cmd_crc_good_stb         (cmd_crc_good_stb       ),/* PHY -> CMD: CRC is good */
  .o_cmd                      (cmd                    ),/* PHY -> CMD: Command */
  .o_cmd_arg                  (cmd_arg                ),/* PHY -> CMD: Command Arg */
  .i_rsps_stb                 (rsps_stb               ),/* CMD -> PHY: Response initiate */
  .i_rsps                     (rsps                   ),/* CMD -> PHY: Response Value */
  .i_rsps_len                 (rsps_len               ),/* CMD -> PHY: Response Length */

  .i_interrupt                (interrupt              ),/* Interrupt */
  .i_read_wait                (read_wait              ),/* SDIO Device is busy working on generated a read */

  //FPGA Interface
  .i_sdio_clk                 (sdio_clk               ),

  .o_sdio_cmd_dir             (o_sd_cmd_dir           ),
  .i_sdio_cmd_in              (i_sd_cmd_in            ),
  .o_sdio_cmd_out             (o_sd_cmd_out           ),

  .o_sdio_data_dir            (o_sd_data_dir          ),
  .i_sdio_data_in             (i_sd_data_in           ),
  .o_sdio_data_out            (o_sd_data_out          )

);


//asynchronous logic
/*
assign  sdio_cmd      = sdio_cmd_dir  ? sdio_cmd_out  : sdio_cmd_in;
assign  sdio_data     = sdio_data_dir ? sdio_data_out : sdio_data_in;
assign  chip_select_n = sdio_data[3];
*/

//synchronous logic

endmodule
