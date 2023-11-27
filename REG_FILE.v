`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07.02.2022 20:55:43
// Design Name: 
// Module Name: REG_FILE
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

module REG_FILE(
    // Clock & Reset    
    input i_clk,
    input i_rstn,
    // Inputs
    input i_write_enable,
    
    input [`REG_FILE_ADDR_LEN-1:0] i_addr_write,
    input [`REG_FILE_ADDR_LEN-1:0] i_addr_read_1,
    input [`REG_FILE_ADDR_LEN-1:0] i_addr_read_2, 
    
    input [`DATA_LEN-1:0] i_data,
    // Outputs
    output reg [`DATA_LEN-1:0] o_data_1,
    output reg [`DATA_LEN-1:0] o_data_2 
);

    integer i; 
    reg [`DATA_LEN-1:0] r_reg_file [`REG_FILE_LEN-1:0];    
       
    always @(posedge i_clk) begin
        if(!i_rstn) begin
            for(i = 0; i < `REG_FILE_LEN; i=i+1) begin
                r_reg_file[i] = `DATA_LEN'b0;    
            end    
        end
        else if(i_write_enable) begin
            if(i_addr_write != `REG_FILE_ADDR_LEN'b0) begin
                r_reg_file[i_addr_write] = i_data; 
            end   
        end 
    end   
    
    always @(*) begin
        o_data_1 = r_reg_file[i_addr_read_1];
        o_data_2 = r_reg_file[i_addr_read_2];        
    end
     
endmodule
