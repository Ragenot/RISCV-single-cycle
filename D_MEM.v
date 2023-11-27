`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07.02.2022 20:55:43
// Design Name: 
// Module Name: D_MEM
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

module D_MEM(
    input i_clk,
    input i_we,
    input i_re,
    input [2:0] i_funct3,
    input [`DATA_LEN-1:0] i_addr,
    input [`DATA_LEN-1:0] i_din,
    output reg [`DATA_LEN-1:0] o_dout                  
);
    localparam D_MEM_ADDR_SIZE = $clog2(`D_MEM_SIZE);
    wire [D_MEM_ADDR_SIZE-1:0] w_addr = i_addr[D_MEM_ADDR_SIZE-1:0];

    reg [7:0] ram_dual_port[`D_MEM_SIZE-1:0];
    

    wire [`DATA_LEN-1:0] w_dout =  {ram_dual_port[w_addr + 2'd3], ram_dual_port[w_addr + 2'd2],
                                    ram_dual_port[w_addr + 2'd1], ram_dual_port[w_addr]};

    initial begin
        $readmemh("Data.mem", ram_dual_port);
    end

    // write process
    always @(posedge i_clk)
    begin
        if (i_we) begin
            case (i_funct3)
            `FUNCT3_SB_C: begin
                //ram_dual_port[w_addr] <= {w_dout[31:8], i_din[7:0]};
                ram_dual_port[w_addr] <= i_din[7:0];     
            end 
            `FUNCT3_SH_C: begin
                //ram_dual_port[w_addr] <= {w_dout[31:16], i_din[15:0]}; 
                ram_dual_port[w_addr] <= i_din[7:0]; 
                ram_dual_port[w_addr + 2'd1] <= i_din[15:8]; 
            end 
            `FUNCT3_SW_C: begin
                //ram_dual_port[w_addr] <= i_din; 
                ram_dual_port[w_addr] <= i_din[7:0]; 
                ram_dual_port[w_addr + 2'd1] <= i_din[15:8]; 
                ram_dual_port[w_addr + 2'd2] <= i_din[23:16]; 
                ram_dual_port[w_addr + 2'd3] <= i_din[31:24]; 
            end 
            default: begin
                //ram_dual_port[w_addr] <= 31'b0;    
            end
            endcase
        end
    end
    
    // read process
    always @(*)
    begin
        if (i_re) begin
            case (i_funct3)
            `FUNCT3_LB_C: begin
                o_dout = {{24{w_dout[7]}}, w_dout[7:0]};
            end 
            `FUNCT3_LH_C: begin
                o_dout = {{16{w_dout[15]}}, w_dout[15:0]};
            end 
            `FUNCT3_LW_C: begin
                o_dout = w_dout;
            end 
            `FUNCT3_LBU_C: begin
                o_dout = {24'b0, w_dout[7:0]};  
            end 
            `FUNCT3_LHU_C: begin
                o_dout = {16'b0, w_dout[15:0]};     
            end 
            default: begin
                o_dout = 31'b0;    
            end
            endcase
        end
    end

endmodule