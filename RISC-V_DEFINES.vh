//=========================================
//  REG_FILE_DEFINES
//=========================================
`define DATA_LEN            32
`define INSTRUCTION_LEN     32
`define REG_FILE_LEN        32
`define REG_FILE_ADDR_LEN   5 
//=========================================
//  I_MEM and D_MEM DEFINES  
//=========================================
`define I_MEM_SIZE          8192 //in bytes
`define D_MEM_SIZE          8192 //in bytes
`define PC_START_ADDR       `DATA_LEN'b0
//=========================================
//  RISC-V Control Layout
//=========================================         
`define CTRL_ALU_VECTOR_LSB     0
`define CTRL_ALU_VECTOR_MSB     3
`define CTRL_D_MEM_RD_BIT       `CTRL_ALU_VECTOR_MSB + 1   // Loads
`define CTRL_D_MEM_WR_BIT       `CTRL_D_MEM_RD_BIT + 1     // Stores
`define CTRL_BRANCH_BIT         `CTRL_D_MEM_WR_BIT + 1     // Branches
`define CTRL_JUMP_BIT           `CTRL_BRANCH_BIT + 1     // Jal/Jalr
`define CTRL_BRANCH_SEL_BIT     `CTRL_JUMP_BIT + 1       // 0 - PC+IMM 1 - RS1+IMM only for JALR
`define CTRL_REG_FILE_WR_BIT    `CTRL_BRANCH_SEL_BIT + 1   // Store RD
`define CTRL_REG_FILE_SEL_LSB   `CTRL_REG_FILE_WR_BIT + 1  // 00 - ALU 01 - D_MEM 
`define CTRL_REG_FILE_SEL_MSB   `CTRL_REG_FILE_SEL_LSB + 1 // 10 - PC+4 for JAL/JALR
`define CTRL_ALU_OP1_SEL_BIT    `CTRL_REG_FILE_SEL_MSB + 1  // 0 - RS1 1 - PC - only for UIPC
`define CTRL_ALU_OP2_SEL_BIT    `CTRL_ALU_OP1_SEL_BIT + 1  // 0 - RS2 1 - IMM
`define CTRL_ALU_UNSIGNED_BIT   `CTRL_ALU_OP2_SEL_BIT + 1  // 0 - Signed 1 - Unsigned
//`define CTRL_ALU_INVERSE_BIT    `CTRL_ALU_UNSIGNED_BIT + 1   // 0 - non 1 - inverse ops

`define CTRL_SIZE               `CTRL_ALU_UNSIGNED_BIT + 1
//=========================================
//  ALU Opcode Layout
//========================================= 
`define ALU_OPCODE_NOP      4'b0000 // NOP                 
`define ALU_OPCODE_ADD      4'b0001 // ADD
`define ALU_OPCODE_SUB      4'b0010 // SUB
`define ALU_OPCODE_XOR      4'b0011 // XOR
`define ALU_OPCODE_OR       4'b0100 // OR
`define ALU_OPCODE_AND      4'b0101 // AND
`define ALU_OPCODE_SLL      4'b0110 // SHIFT LOGIC LEFT
`define ALU_OPCODE_SR       4'b0111 // SHIFT RIGHT [L, A]
`define ALU_OPCODE_MOVE_OP2 4'b1000 // OP2 || only for LUI
`define ALU_OPCODE_EQ       4'b1001 // EQUALITY
`define ALU_OPCODE_NEQ      4'b1010 // NON EQUALITY
`define ALU_OPCODE_LT       4'b1011 // LESS THEN
`define ALU_OPCODE_GE       4'b1100 // GREATER OR EQUAL THEN

//=========================================
//  RISC-V Opcode Layout
//========================================= 
`define INSTR_OPCODE_LSB_C  0 
`define INSTR_OPCODE_MSB_C  6 
`define INSTR_RD_LSB_C      7
`define INSTR_RD_MSB_C      11
`define INSTR_FUNCT3_LSB_C  12
`define INSTR_FUNCT3_MSB_C  14
`define INSTR_RS1_LSB_C     15
`define INSTR_RS1_MSB_C     19
`define INSTR_RS2_LSB_C     20
`define INSTR_RS2_MSB_C     24
`define INSTR_FUNCT7_LSB_C  25
`define INSTR_FUNCT7_MSB_C  31

//=========================================
//  RISC-V Opcodes
//========================================= 
// alu
`define OPCODE_LUI_C        7'b0110111 // 7'h37
`define OPCODE_AUIPC_C      7'b0010111 // 7'h17
`define OPCODE_ALUI_C       7'b0010011 // 7'h13
`define OPCODE_ALU_C        7'b0110011 // 7'h33
// control flow  
`define OPCODE_JAL_C        7'b1101111 // 7'h6F
`define OPCODE_JALR_C       7'b1100111 // 7'h67
`define OPCODE_BRANCH_C     7'b1100011 // 7'h63
// memory access 
`define OPCODE_LOAD_C       7'b0000011 // 7'h03
`define OPCODE_STORE_C      7'b0100011 // 7'h23
// system/csr  
`define OPCODE_FENCE_C      7'b0001111 // 7'h0F
`define OPCODE_SYSCSR_C     7'b1110011 // 7'h73
// atomic memory access (A) 
`define OPCODE_ATOMIC_C     7'b0101111 // 7'h2F
// floating point operations 
`define OPCODE_FOP_C        7'b1010011 // 7'h53
//=========================================
//  RISC-V funct3 opcodes
//========================================= 
// branch
`define FUNCT3_BEQ_C        3'b000        
`define FUNCT3_BNE_C        3'b001
`define FUNCT3_BLT_C        3'b100
`define FUNCT3_BGE_C        3'b101
`define FUNCT3_BLTU_C       3'b110
`define FUNCT3_BGEU_C       3'b111
// load 
`define FUNCT3_LB_C         3'b000        
`define FUNCT3_LH_C         3'b001
`define FUNCT3_LW_C         3'b010
`define FUNCT3_LBU_C        3'b100
`define FUNCT3_LHU_C        3'b101
// store 
`define FUNCT3_SB_C         3'b000        
`define FUNCT3_SH_C         3'b001
`define FUNCT3_SW_C         3'b010
// alu 
`define FUNCT3_ADDSUB_C     3'b000 
`define FUNCT3_SLL_C        3'b001 
`define FUNCT3_SLT_C        3'b010 
`define FUNCT3_SLTU_C       3'b011 
`define FUNCT3_SR_C         3'b101 
`define FUNCT3_XOR_C        3'b100 
`define FUNCT3_OR_C         3'b110
`define FUNCT3_AND_C        3'b111

 