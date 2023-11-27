`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07.02.2022 21:10:00
// Design Name: 
// Module Name: DECODER_CONTROL
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

module DECODER_CONTROL(
    input [`DATA_LEN-1:0] i_cur_instruction,
    output reg [`CTRL_SIZE-1:0] o_control_vector
);

    wire [6:0] w_opcode = i_cur_instruction[`INSTR_OPCODE_MSB_C:`INSTR_OPCODE_LSB_C];
    wire [2:0] w_funct3 = i_cur_instruction[`INSTR_FUNCT3_MSB_C:`INSTR_FUNCT3_LSB_C];
    wire [7:0] w_funct7 = i_cur_instruction[`INSTR_FUNCT7_MSB_C:`INSTR_FUNCT7_LSB_C];
    
    always @(*) begin
        o_control_vector = `CTRL_SIZE'b0;
        case (w_opcode)
            // I-immediate: ALU-immediate
            `OPCODE_ALU_C, `OPCODE_ALUI_C : begin
                o_control_vector[`CTRL_ALU_OP2_SEL_BIT] = ~w_opcode[5]; // 0 - OPCODE_ALUI_C
                                                                        // 1 - OPCODE_ALU_C 
                o_control_vector[`CTRL_REG_FILE_WR_BIT] = 1'b1;
                case(w_funct3)   
                `FUNCT3_ADDSUB_C: begin 
                    if(~w_opcode[5]) begin // ADDI
                        o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_ADD; // ADD op    
                    end
                    else begin // ADD/SUB
                        if (w_funct7[6]) begin //SUB
                            o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_SUB; // SUB op 
                        end
                        else begin // ADD
                            o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_ADD; // ADD op 
                        end
                    end
                end
                `FUNCT3_SLL_C: begin 
                    o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_SLL; // SLL op
                end
                `FUNCT3_SLT_C: begin 
                    o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_LT; // LT op    
                end
                `FUNCT3_SLTU_C: begin 
                    o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_LT; // LT op  
                    o_control_vector[`CTRL_ALU_UNSIGNED_BIT] = 1'b1;
                end
                `FUNCT3_SR_C: begin 
                    if(w_funct7[6]) begin // SRA[I]
                        o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_SR; // SRA op    
                    end
                    else begin // SRL[I]
                        o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_SR; // SRL op
                        o_control_vector[`CTRL_ALU_UNSIGNED_BIT] = 1'b1;
                    end   
                end
                `FUNCT3_XOR_C: begin 
                    o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_XOR; // XOR op
                end
                `FUNCT3_OR_C: begin 
                    o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_OR; // OR op
                end
                `FUNCT3_AND_C: begin 
                    o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_AND; // AND op
                end
                default: begin
                
                end
                endcase    
            end 
            // I-immediate: load
            `OPCODE_LOAD_C : begin 
                o_control_vector[`CTRL_REG_FILE_WR_BIT]  = 1'b1; // WE REG_FILE
                o_control_vector[`CTRL_D_MEM_RD_BIT]     = 1'b1; // RE D_MEM 
                o_control_vector[`CTRL_REG_FILE_SEL_MSB:`CTRL_REG_FILE_SEL_LSB] = 2'b01; // REG_FILE_DIN = D_MEM
                o_control_vector[`CTRL_ALU_OP2_SEL_BIT]  = 1'b1; // IMM
                o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_ADD; // ADD op
            end
            // S-immediate: store 
            `OPCODE_STORE_C: begin
                o_control_vector[`CTRL_D_MEM_WR_BIT]    = 1'b1; // WE D_MEM
                o_control_vector[`CTRL_ALU_OP2_SEL_BIT] = 1'b1; // IMM   
                o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_ADD; // ADD op             
            end  
            // B-immediate: conditional branches
            // I-immediate: jump-and-link with registers
            // J-immediate: unconditional jumps
            `OPCODE_BRANCH_C, `OPCODE_JAL_C, `OPCODE_JALR_C: begin
                if(w_opcode == `OPCODE_BRANCH_C) begin
                    o_control_vector[`CTRL_BRANCH_BIT] = 1'b1;
                    case (w_funct3) 
                    `FUNCT3_BEQ_C: begin 
                        o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_EQ;
                    end
                    `FUNCT3_BNE_C: begin
                        o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_NEQ;
                    end
                    `FUNCT3_BLT_C: begin
                        o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_LT;
                    end
                    `FUNCT3_BGE_C: begin
                        o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_GE;
                    end 
                    `FUNCT3_BLTU_C: begin
                        o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_LT;
                        o_control_vector[`CTRL_ALU_UNSIGNED_BIT] = 1'b1;
                    end
                    `FUNCT3_BGEU_C: begin
                        o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_GE;
                        o_control_vector[`CTRL_ALU_UNSIGNED_BIT] = 1'b1;
                    end
                    endcase   
                end
                else begin // JAL/JALR
                    o_control_vector[`CTRL_REG_FILE_WR_BIT] = 1'b1; // WE REG_FILE
                    o_control_vector[`CTRL_REG_FILE_SEL_MSB:`CTRL_REG_FILE_SEL_LSB] = 2'b10; // REG_FILE_DIN = PC + 4  
                    o_control_vector[`CTRL_JUMP_BIT] = 1'b1;
                    if(w_opcode == `OPCODE_JALR_C) begin
                        o_control_vector[`CTRL_BRANCH_SEL_BIT] = 1'b1;
                    end 
                end    
            end  
            // U-immediate: lui, auipc
            `OPCODE_LUI_C, `OPCODE_AUIPC_C: begin
                o_control_vector[`CTRL_REG_FILE_WR_BIT] = 1'b1;
                o_control_vector[`CTRL_ALU_OP2_SEL_BIT] = 1'b1;    
                if(w_opcode == `OPCODE_LUI_C) begin // LUI
                    o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_MOVE_OP2;     
                end
                else begin // AUIPC
                    o_control_vector[`CTRL_ALU_VECTOR_MSB:`CTRL_ALU_VECTOR_LSB] = `ALU_OPCODE_ADD;   
                    o_control_vector[`CTRL_ALU_OP1_SEL_BIT] = 1'b1;     
                end
            end  
            default: begin   
            end 
        endcase    
    end
    
endmodule
