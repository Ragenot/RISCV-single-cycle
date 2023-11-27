`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08.02.2022 11:30:38
// Design Name: 
// Module Name: CORE
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

module CORE(
    // Clock & Reset    
    input i_clk,
    input i_rstn
);

    wire [`DATA_LEN-1:0] w_cur_instruction;
    wire [`CTRL_SIZE-1:0] w_control_vector;
    reg [`DATA_LEN-1:0] r_pc;
    
    wire [`REG_FILE_ADDR_LEN-1:0] w_rd_addr  = w_cur_instruction[`INSTR_RD_MSB_C:`INSTR_RD_LSB_C];
    wire [`REG_FILE_ADDR_LEN-1:0] w_rs1_addr = w_cur_instruction[`INSTR_RS1_MSB_C:`INSTR_RS1_LSB_C];
    wire [`REG_FILE_ADDR_LEN-1:0] w_rs2_addr = w_cur_instruction[`INSTR_RS2_MSB_C:`INSTR_RS2_LSB_C];
    wire [2:0] w_funct3 = w_cur_instruction[`INSTR_FUNCT3_MSB_C:`INSTR_FUNCT3_LSB_C];
    
    wire [`DATA_LEN-1:0] w_imm;
    wire [`DATA_LEN-1:0] w_rs1;
    wire [`DATA_LEN-1:0] w_rs2; 
    wire [`DATA_LEN-1:0] w_d_mem_dout;
    wire [`DATA_LEN-1:0] w_alu_result;
    wire [`DATA_LEN-1:0] w_alu_op1 = (w_control_vector[`CTRL_ALU_OP1_SEL_BIT]) ? r_pc  : w_rs1;
    wire [`DATA_LEN-1:0] w_alu_op2 = (w_control_vector[`CTRL_ALU_OP2_SEL_BIT]) ? w_imm : w_rs2; 
    wire [`DATA_LEN-1:0] w_reg_file_din = (w_control_vector[`CTRL_REG_FILE_SEL_MSB:`CTRL_REG_FILE_SEL_LSB] == 2'b10) ? w_next_linear_pc :
                                          (w_control_vector[`CTRL_REG_FILE_SEL_MSB:`CTRL_REG_FILE_SEL_LSB] == 2'b01) ? w_d_mem_dout : w_alu_result;
    
    
    wire [`DATA_LEN-1:0] w_next_linear_pc = r_pc + 31'd4;
    wire [`DATA_LEN-1:0] w_pc_op1 = (w_control_vector[`CTRL_BRANCH_SEL_BIT]) ? w_rs1 : r_pc; 
    wire [`DATA_LEN-1:0] w_pc_op2 = w_imm;
    wire [`DATA_LEN-1:0] w_next_jump_pc = w_pc_op1 + w_pc_op2;
        
    always @(posedge i_clk) begin
        if(!i_rstn) begin
            r_pc <= `PC_START_ADDR;
        end
        else begin
            if((w_control_vector[`CTRL_BRANCH_BIT] & w_alu_result[0]) | w_control_vector[`CTRL_JUMP_BIT]) begin
                r_pc <= w_next_jump_pc;   
            end
            else begin
                r_pc <= w_next_linear_pc;
            end    
        end 
    end

//=========================================
//  INSTANCES
//========================================= 
    ALU ALU_RV32I(
        .i_op1              (w_alu_op1),
        .i_op2              (w_alu_op2),
        .i_control_vector   (w_control_vector),
        .o_result           (w_alu_result)
    );

    REG_FILE REG_FILE_RV32I(
        // Clock & Reset    
        .i_clk              (i_clk),
        .i_rstn             (i_rstn),
        // Inputs
        .i_write_enable     (w_control_vector[`CTRL_REG_FILE_WR_BIT]),
        
        .i_addr_write       (w_rd_addr),
        .i_addr_read_1      (w_rs1_addr),
        .i_addr_read_2      (w_rs2_addr), 
        
        .i_data             (w_reg_file_din),
        // Outputs
        .o_data_1           (w_rs1),
        .o_data_2           (w_rs2)
    );

    DECODER_CONTROL DECODER_CONTROL_RV32I(
        .i_cur_instruction  (w_cur_instruction),
        .o_control_vector   (w_control_vector)
    );

    DECODER_IMM DECODER_IMM_RV32I(
        .i_cur_instruction  (w_cur_instruction),
        .o_immediate        (w_imm)
    );
    
    D_MEM D_MEM_RV32I(
        .i_clk              (i_clk),
        .i_we               (w_control_vector[`CTRL_D_MEM_WR_BIT]),
        .i_re               (w_control_vector[`CTRL_D_MEM_RD_BIT]),
        .i_funct3           (w_funct3),
        .i_addr             (w_alu_result),
        .i_din              (w_rs2),
        .o_dout             (w_d_mem_dout)
    );
    
    I_MEM I_MEM_RV32I(
        .i_clk              (i_clk),
        .i_addr             (r_pc),
        .o_dout             (w_cur_instruction)
    );
    
    
endmodule
