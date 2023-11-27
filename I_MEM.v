`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07.02.2022 20:55:43
// Design Name: 
// Module Name: I_MEM
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

`include "RISC-V_DEFINES.vh"

module I_MEM(
    input i_clk,
    input [`DATA_LEN-1:0] i_addr,
    output [`DATA_LEN-1:0] o_dout                  
);

    localparam I_MEM_ADDR_SIZE = $clog2(`I_MEM_SIZE);
    wire [I_MEM_ADDR_SIZE-1:0] w_addr = i_addr[I_MEM_ADDR_SIZE-1:0];

    reg [7:0] ram_single_port[`I_MEM_SIZE-1:0];

    initial begin
        $readmemh("Instruction.mem", ram_single_port);
    end

    assign o_dout = {ram_single_port[w_addr + 2'd3], ram_single_port[w_addr + 2'd2],
                     ram_single_port[w_addr + 2'd1], ram_single_port[w_addr]};

endmodule
