`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07.02.2022 21:10:00
// Design Name: 
// Module Name: DECODER_IMM
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

module DECODER_IMM(
    input [`DATA_LEN-1:0] i_cur_instruction,
    
    output reg [`DATA_LEN-1:0] o_immediate 
);
    wire [6:0] w_opcode = i_cur_instruction[`INSTR_OPCODE_MSB_C:`INSTR_OPCODE_LSB_C];
    
    always @(*) begin
        case (w_opcode)
            // I-immediate: ALU-immediate, loads, jump-and-link with registers
            `OPCODE_JALR_C, `OPCODE_LOAD_C, `OPCODE_ALUI_C : begin
                o_immediate[31:11] = {21{i_cur_instruction[31]}};
                o_immediate[10:0]  = i_cur_instruction[30:20];     
            end 
            // S-immediate: store 
            `OPCODE_STORE_C: begin
                o_immediate[31:11] = {21{i_cur_instruction[31]}};
                o_immediate[10:5]  = i_cur_instruction[30:25]; 
                o_immediate[4:0]   = i_cur_instruction[11:7];         
            end  
            // B-immediate: conditional branches
            `OPCODE_BRANCH_C: begin
                o_immediate[31:12] = {20{i_cur_instruction[31]}};
                o_immediate[11]    = i_cur_instruction[7]; 
                o_immediate[10:5]  = i_cur_instruction[30:25]; 
                o_immediate[4:1]   = i_cur_instruction[11:8];
                o_immediate[0]     = 1'b0;     
            end  
            // U-immediate: lui, auipc
            `OPCODE_LUI_C, `OPCODE_AUIPC_C: begin
                o_immediate[31:12] = i_cur_instruction[31:12];
                o_immediate[11:0]  = 12'b0;
            end  
            // J-immediate: unconditional jumps
            `OPCODE_JAL_C: begin
                o_immediate[31:20] = {12{i_cur_instruction[31]}};
                o_immediate[19:12] = i_cur_instruction[19:12]; 
                o_immediate[11]    = i_cur_instruction[20]; 
                o_immediate[10:1]  = i_cur_instruction[30:21];
                o_immediate[0]     = 1'b0;          
            end  
            default: begin
                o_immediate = `DATA_LEN'b0;
            end 
        endcase 
    end    
endmodule
