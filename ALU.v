`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07.02.2022 20:55:43
// Design Name: 
// Module Name: ALU
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

module ALU(
    input [`DATA_LEN-1:0] i_op1,
    input [`DATA_LEN-1:0] i_op2,
    input [`CTRL_SIZE-1:0] i_control_vector,
    output reg [`DATA_LEN-1:0] o_result
);

    wire signed [`DATA_LEN:0] w_extend_op1 = (i_control_vector[`CTRL_ALU_UNSIGNED_BIT]) ? {1'b0, i_op1} : {i_op1[`DATA_LEN-1], i_op1};
    wire signed [`DATA_LEN:0] w_extend_op2 = (i_control_vector[`CTRL_ALU_UNSIGNED_BIT]) ? {1'b0, i_op2} : {i_op2[`DATA_LEN-1], i_op2};    
    
    wire w_lt = (w_extend_op1 < w_extend_op2);
    wire w_eq = (w_extend_op1 == w_extend_op2);
    
    // Shift Right Arithmetic
    wire [`DATA_LEN:0] w_sr = w_extend_op1 >>> w_extend_op2[4:0]; 


    wire [3:0] alu_opcode = i_control_vector[3:0];
    always @(*) begin
        case(alu_opcode)
        `ALU_OPCODE_ADD: begin
            o_result = i_op1 + i_op2;
        end    
        `ALU_OPCODE_SUB: begin
            o_result = i_op1 - i_op2;
        end  
        `ALU_OPCODE_XOR: begin
            o_result = i_op1 ^ i_op2;
        end  
        `ALU_OPCODE_OR: begin
            o_result = i_op1 | i_op2;
        end  
        `ALU_OPCODE_AND: begin
            o_result = i_op1 & i_op2;
        end  
        `ALU_OPCODE_SLL: begin
            o_result = i_op1 << i_op2[4:0];
        end  
        `ALU_OPCODE_SR: begin
            o_result = w_sr[`DATA_LEN-1:0];
        end  
        `ALU_OPCODE_MOVE_OP2: begin
            o_result = i_op2;
        end   
        `ALU_OPCODE_EQ: begin
            o_result = {`DATA_LEN'b0, w_eq};
        end  
        `ALU_OPCODE_NEQ: begin
            o_result = {`DATA_LEN'b0, ~w_eq};
        end  
        `ALU_OPCODE_LT: begin
            o_result = {`DATA_LEN'b0, w_lt};
        end  
        `ALU_OPCODE_GE: begin
            o_result = {`DATA_LEN'b0, ~w_lt};
        end  
        default: begin
            o_result = 'd0;
        end  
        endcase    
    end
endmodule


/* Previous verison of ALU
module ALU(
    input [`DATA_LEN-1:0] i_op1,
    input [`DATA_LEN-1:0] i_op2,
    input [`CTRL_SIZE-1:0] i_control_vector,
    output [`DATA_LEN-1:0] o_result
);

    wire signed [`DATA_LEN:0] w_extend_op1 = (i_control_vector[`CTRL_ALU_UNSIGNED_BIT]) ? {1'b0, i_op1} : {i_op1[`DATA_LEN-1], i_op1};
    wire signed [`DATA_LEN:0] w_extend_op2 = (i_control_vector[`CTRL_ALU_UNSIGNED_BIT]) ? {1'b0, i_op2} : {i_op2[`DATA_LEN-1], i_op2};    
    
    wire w_lt = (w_extend_op1 < w_extend_op2);
    wire w_eq = (w_extend_op1 == w_extend_op2);
    
    // Shift Right Arithmetic
    wire [`DATA_LEN:0] w_sr = w_extend_op1 >>> w_extend_op2[4:0]; 

    wire [`DATA_LEN-1:0] w0 = (i_control_vector[`CTRL_ALU_ADD]) ? i_op1 + i_op2 : 0;
    wire [`DATA_LEN-1:0] w1 = (i_control_vector[`CTRL_ALU_SUB]) ? i_op1 - i_op2 : 0;
    wire [`DATA_LEN-1:0] w2 = (i_control_vector[`CTRL_ALU_XOR]) ? i_op1 ^ i_op2 : 0;
    wire [`DATA_LEN-1:0] w3 = (i_control_vector[`CTRL_ALU_OR])  ? i_op1 | i_op2 : 0;
    wire [`DATA_LEN-1:0] w4 = (i_control_vector[`CTRL_ALU_AND]) ? i_op1 & i_op2 : 0;
    wire [`DATA_LEN-1:0] w5 = (i_control_vector[`CTRL_ALU_SLL]) ? i_op1 << i_op2[4:0] : 0;
    wire [`DATA_LEN-1:0] w6 = (i_control_vector[`CTRL_ALU_SR]) ?  w_sr[`DATA_LEN-1:0] : 0;
    wire [`DATA_LEN-1:0] w7 = (i_control_vector[`CTRL_ALU_MOVE_OP2]) ? i_op2 : 0;
    wire [`DATA_LEN-1:0] w8 = (i_control_vector[`CTRL_ALU_BEQ]) ? {`DATA_LEN'b0, w_eq} : 0;
    wire [`DATA_LEN-1:0] w9 = (i_control_vector[`CTRL_ALU_BNE]) ? {`DATA_LEN'b0, ~w_eq} : 0;
    wire [`DATA_LEN-1:0] w10 = (i_control_vector[`CTRL_ALU_BLT]) ? {`DATA_LEN'b0, w_lt} : 0;
    wire [`DATA_LEN-1:0] w11 = (i_control_vector[`CTRL_ALU_BGE]) ? {`DATA_LEN'b0, ~w_lt} : 0;   
    assign o_result = w0 ^ w1 ^ w2 ^ w3 ^ w4 ^ w5 ^ w6 ^ w7 ^ w8 ^ w9 ^ w10 ^ w11;
endmodule
*/