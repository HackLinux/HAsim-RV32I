
// isa_decode_functions

// This file contains functions which decode a rv32i instruction.

// First some helper functions


// 7 bits of an instruction are the opcode. [change to 7 bit]
typedef Bit#(7) RV32I_OPCODE;

// 3 bits of funct3
typedef Bit#(3) RV32I_FUNCT3;

// 7 bits of funct7
typedef Bit#(7) RV32I_FUNCT7;

// 12 bits of funct12
typedef Bit#(12) RV32I_FUNCT12;

// LUI - Load upper immediate
RV32I_OPCODE rv32iLUI = 7'b0110111;

// AUIPC - Add upper immediate to pc
RV32I_OPCODE rv32iAUIPC = 7'b0010111;

//JAL - Jump and link
RV32I_OPCODE rv32iJAL = 7'b1101111;

//JALR - Jump and link register
RV32I_OPCODE rv32iJALR = 7'b1100111;

// Conditional branches - take funct3 [14:12]
RV32I_OPCODE rv32iBRANCH = 7'b1100011;

	// BEQ - Branch equal
	RV32I_FUNCT3 rv32iF3BEQ = 3'b000;

	// BNE - Branch not equal
	RV32I_FUNCT3 rv32iF3BNE = 3'b001;

	// BLT - Branch less than
	RV32I_FUNCT3 rv32iF3BLT = 3'b100;

	// BGT - Branch greater than
	RV32I_FUNCT3 rv32iF3BGE = 3'b101;

	// BLTU - Branch less than unsigned
	RV32I_FUNCT3 rv32iF3BLTU = 3'b110;

	// BGEU - Branch greaer or equal unsigned
	RV32I_FUNCT3 rv32iF3BGEU = 3'b111;

// Load Instructions
RV32I_OPCODE rv32iLOAD = 7'b0000011;

	// LB - LB are defined analogously for 8-bit values
	RV32I_FUNCT3 rv32iF3LB = 3'b000;

	// LH - Loads a 16-bit value from memory
	RV32I_FUNCT3 rv32iF3LH = 3'b001;

	// LW - Loads a 32-bit value from memory
	RV32I_FUNCT3 rv32iF3LW = 3'b010;

	// LBU - LBU are defined analogously for 8-bit values
	RV32I_FUNCT3 rv32iF3LBU = 3'b100;

	// LHU - LHU loads a 16-bit value from memory but then zero extends to 32-bits before storing in rd
	RV32I_FUNCT3 rv32iF3LHU = 3'b101;

// Store Instructions
RV32I_OPCODE rv32iSTORE = 7'b0100011;

	// SB - Store 8-bit values from the low bits of register rs2 to memory
	RV32I_FUNCT3 rv32iF3SB = 3'b000;

	// SH - Store 16-bit values from the low bits of register rs2 to memory
	RV32I_FUNCT3 rv32iF3SH = 3'b001;

	// SW - Store 32-bit values from the low bits of register rs2 to memory
	RV32I_FUNCT3 rv32iF3SW = 3'b010;

// Integer Register-Immediate Operations
RV32I_OPCODE rv32iOPIMM = 7'b0010011;

	// ADDI - Add immediate
	RV32I_FUNCT3 rv32iF3ADDI = 3'b000;

	// SLTI - set less than immediate
	RV32I_FUNCT3 rv32iF3SLTI = 3'b010;

	// SLTIU - set less than immediate unsigned
	RV32I_FUNCT3 rv32iF3SLTIU = 3'b011;

	// XORI - XOR on register rs1 and the sign-extended 12-bit immediate
	RV32I_FUNCT3 rv32iF3XORI = 3'b100;

	// ORI - OR on register rs1 and the sign-extended 12-bit immediate
	RV32I_FUNCT3 rv32iF3ORI = 3'b110;

	// ANDI - AND on register rs1 and the sign-extended 12-bit immediate
	RV32I_FUNCT3 rv32iF3ANDI = 3'b111;

	// SLLI - logical left shift immediate
	RV32I_FUNCT3 rv32iF3SLLI = 3'b001;

	// SRLI-SRAI
	RV32I_FUNCT3 rv32iF3_SRLISRAI = 3'b101;

		// SRLI - logical right shift immediate
		RV32I_FUNCT7 rv32iF7SRLI = 7'b0000000;

		// SRAI - arithmetic right shift immediate
		RV32I_FUNCT7 rv32iF7SRAI = 7'b0100000;

// Integer Register-Register Operations
RV32I_OPCODE rv32iOP = 7'b0110011;

	// ADD-SUB
 	RV32I_FUNCT3 rv32iF3_ADDSUB = 3'b000;
		
		// ADD
		RV32I_FUNCT7 rv32iF7ADD = 7'b0000000;

		// SUB
		RV32I_FUNCT7 rv32iF7SUB = 7'b0100000;

	// SLL
 	RV32I_FUNCT3 rv32iF3SLL = 3'b001;

	// SLT
 	RV32I_FUNCT3 rv32iF3SLT = 3'b010;

	// SLTU
 	RV32I_FUNCT3 rv32iF3SLTU = 3'b011;

	// XOR
 	RV32I_FUNCT3 rv32iF3XOR = 3'b100;

	// SRL-SRA
 	RV32I_FUNCT3 rv32iF3_SRLSRA = 3'b101;
		
		// SRL
		RV32I_FUNCT7 rv32iF7SRL = 7'b0000000;

		// SRA
		RV32I_FUNCT7 rv32iF7SRA = 7'b0100000;

	// OR
 	RV32I_FUNCT3 rv32iF3OR = 3'b110;

	// AND
 	RV32I_FUNCT3 rv32iF3AND = 3'b111;

// Memory Model
RV32I_OPCODE rv32iMEM = 7'b0001111;

	// FENCE - instruction provides finer-grain memory and I/O orderings
	RV32I_FUNCT3 rv32iF3FENCE = 3'b000;	

	// FENCE.I - instruction is used to synchronize the instruction and data streams
	RV32I_FUNCT3 rv32iF3FENCEI = 3'b001;

// System Instructions
RV32I_OPCODE rv32iSYS = 7'b1110011;

	// SCALL - SBREAK
	RV32I_FUNCT3 rv32iF3PRIV = 3'b000;
		
		// SCALL
		RV32I_FUNCT7 rv32iF7SCALL = 7'b0000000;

		// SBREAK
		RV32I_FUNCT7 rv32iF7SBREAK = 7'b0000001;
	
	// Timers and Counters	
	RV32I_FUNCT3 rv32iF3CSRRS = 3'b010;

		// RDCYCLE
		RV32I_FUNCT12 rv32iF12RDCYCLE = 12'b110000000000;

		// RDCYCLEH
		RV32I_FUNCT12 rv32iF12RDCYCLEH = 12'b110010000000;

		// RDTIME
		RV32I_FUNCT12 rv32iF12RDTIME = 12'b110000000001;

		// RDTIMEH
		RV32I_FUNCT12 rv32iF12RDTIMEH = 12'b110010000001;

		// RDINSTRET
		RV32I_FUNCT12 rv32iF12RDINSTRET = 12'b110000000010;

		// RDINSTRETH
		RV32I_FUNCT12 rv32iF12RDINSTRETH = 12'b110010000010;
	


// isaGetSrc

// Given a risc-v 32i instruction, return the nth source register.
// Or return Invalid if there is no such source for this instruction.
// Implemented using the helper functions below.

function Maybe#(ISA_REG_INDEX) isaGetSrc(ISA_INSTRUCTION i, Integer n);

    case (n)
      0: return rv32iGetSrc1(i);
      1: return rv32iGetSrc2(i);
    endcase

endfunction

// rv32iGetSrc1

// Given an instruction, return source 1 (rs)
// The RISC-V ISA keeps the source (rs1 and rs2) and destination (rd) registers
// at the same position in all formats to simplify decoding

function Maybe#(ISA_REG_INDEX) rv32iGetSrc1(ISA_INSTRUCTION i);
    
    RV32I_OPCODE op = i[6:0];

    case (op)
        // LUI has no src1
        rv32iLUI: return tagged Invalid;
	// AUIPC has no src1
	rv32iAUIPC: return tagged Invalid;
	// JAL has no src1
        rv32iJAL: return tagged Invalid;

        // Memory Model have no src1
        rv32iMEM:   return tagged Invalid;

	//System Instructions have no src1 
        rv32iSYS:   return tagged Invalid;

        // for all case
        default:     return tagged Valid i[19:15];
    endcase

endfunction

// rv32iGetSrc2

// Given an instruction, return source 2 (rt)
// The RISC-V ISA keeps the source (rs1 and rs2) and destination (rd) registers
// at the same position in all formats to simplify decoding

function Maybe#(ISA_REG_INDEX) rv32iGetSrc2(ISA_INSTRUCTION i);
    
    RV32I_OPCODE op = i[6:0];

    case (op)
	// LUI has no src2
        rv32iLUI: return tagged Invalid;
	// AUIPC has no src2
	rv32iAUIPC: return tagged Invalid;
	// JAL has no src2
        rv32iJAL: return tagged Invalid;
	// JALR has no src2
        rv32iJALR: return tagged Invalid;
	
	// Load Instructions have no src2
	rv32iLOAD: return tagged Invalid;
	
	// Integer Register-Immediate Operations have no src2
	rv32iOPIMM: return tagged Invalid;

        // Memory Model have no src2
        rv32iMEM:   return tagged Invalid;

	//System Instructions have no src2
        rv32iSYS:   return tagged Invalid;
	
        default: return tagged Valid i[24:20];
    endcase

endfunction


// isaGetDst

// Given an instruction, return the destination register (rd).
// No RV32I instruction has more than one dest.


function Maybe#(ISA_REG_INDEX) isaGetDst(ISA_INSTRUCTION i, Integer n);

    Maybe#(ISA_REG_INDEX) retval = tagged Invalid;

    retval = case (n)
                 0      : return rv32iGetDst1(i);
                 default: return tagged Invalid;
             endcase;

    return case (retval) matches
               tagged Valid 0: return tagged Invalid;
               default       : return retval;
           endcase;

endfunction

// rv32iGetDst1

// The RISC-V ISA keeps the source (rs1 and rs2) and destination (rd) registers
// at the same position in all formats to simplify decoding

function Maybe#(ISA_REG_INDEX) rv32iGetDst1(ISA_INSTRUCTION i);

    RV32I_OPCODE op = i[6:0];

    case (op)
        // These have no destinations

	// Conditional branches, Store Instructions, Memory Model
	rv32iBRANCH,
        rv32iSTORE,
	rv32iMEM: return tagged Invalid;
	rv32iSYS:
	begin
		RV32I_FUNCT3 sys_op = i[14:12];
		case (sys_op)
			// System instruction have no destination (SCALL-SBREAK)
			rv32iF3PRIV: return tagged Invalid; 
			// Timers and Counters have destination
			rv32iF3CSRRS: return tagged Valid i[11:7];
		endcase

	end
    
        // The rest of RV32I instructions
        default: return tagged Valid i[11:7];
    endcase


endfunction


// isaGetNumDsts

// Given an instruction, return the number of destination registers (rd).
// Most have one dest. 

function Integer isaGetNumDsts(ISA_INSTRUCTION i);

    RV32I_OPCODE op = i[6:0];

    case (op)
        // These have no destinations

	// Conditional branches, Store Instructions, Memory Model
	rv32iBRANCH,
        rv32iSTORE,
	rv32iMEM: return 0;
	rv32iSYS:
	begin
		RV32I_FUNCT3 sys_op = i[14:12];
		case (sys_op)
			// System instruction have no destination (SCALL-SBREAK)
			rv32iF3PRIV: return 0; 
			// Timers and Counters have destination
			rv32iF3CSRRS: return 1;
		endcase

	end
    
        // The rest of RV32I instructions
        default: return 1;
    endcase

endfunction


// isaIsLoad

// Only LW is a load.

function Bool isaIsLoad(ISA_INSTRUCTION i);

	RV32I_OPCODE op = i[6:0];
	case (op)
		rv32iLOAD:
		begin
			RV32I_FUNCT3 lw_op = i[14:12];
			return lw_op == rv32iF3LW;
		end
		default: return False;

	endcase

endfunction

// ???????????????????????????
// isaIsLoadLocked

// Returns true if the given instruction is a load locked.

function Bool isaIsLoadLocked(ISA_INSTRUCTION i) = False;


// isaIsStore

// Only SW is a store.

function Bool isaIsStore(ISA_INSTRUCTION i);

	RV32I_OPCODE op = i[6:0];
	case (op)
		rv32iSTORE:
		begin
			RV32I_FUNCT3 sw_op = i[14:12];
			return sw_op == rv32iF3SW;
		end
		default: return False;

	endcase

endfunction


// isaIsStoreCond

// Returns destination register if the given instruction is a store conditional.

function Maybe#(Integer) isaIsStoreCond(ISA_INSTRUCTION i) = tagged Invalid;


// isIsBranch

// All branches and jumps should be snapshotted.

function Bool isaIsBranch(ISA_INSTRUCTION i);

    RV32I_OPCODE op = i[6:0];

    case (op)
        // Conditional branches, J, JAL, branch
        rv32iJAL,
	rv32iJALR,
	rv32iBRANCH: return True;
      
        // The rest of RISC-V instructions are not branches.
        default: return False;

    endcase

endfunction


// isaDrainBefore

// No RISC-V instructions require draining.

function Bool isaDrainBefore(ISA_INSTRUCTION i);

    return False;

endfunction


// isaDrainAfter

// No RISC-V instructions require draining.

function Bool isaDrainAfter(ISA_INSTRUCTION i);

    return False;

endfunction


// isaLoadType

// Returns MEMOP_Word.

function ISA_MEMOP_TYPE isaLoadType(ISA_INSTRUCTION i);

    return MEMOP_Word;

endfunction


// isaStoreType

// Returns MEMOP_Word.

function ISA_MEMOP_TYPE isaStoreType(ISA_INSTRUCTION i);

    return MEMOP_Word;

endfunction


// isaEmulateInstruction

function Bool isaEmulateInstruction(ISA_INSTRUCTION i);
    return False;
endfunction

function Bool isBranchImm(ISA_INSTRUCTION inst);
    RV32I_OPCODE op = inst[6:0];

    if(op == rv32iBRANCH)
        return True;
    else
        return False;
endfunction


function Bool isJumpImm(ISA_INSTRUCTION inst);
    RV32I_OPCODE op = inst[6:0];
    return (op == rv32iJAL || op == rv32iJALR);
endfunction

// ????????????????????????????

function ISA_ADDRESS predPcBranchImm(ISA_ADDRESS addr, ISA_INSTRUCTION inst);
	return addr + (signExtend({inst[31], inst[7], inst[30:25], inst[11:8]}) << 2);
endfunction

function ISA_ADDRESS predPcJumpImm(ISA_ADDRESS addr, ISA_INSTRUCTION inst);
    //return {(addr + 4)[31:28], inst[25:0], 2'b00};
	return addr+ (signExtend({inst[31], inst[19:12], inst[20], inst[30:21]}) << 2);
	
endfunction
    

//
// Generate masks of destinations written at different pipeline stages
//

//
// Load's first destination is written at load.
//
function ISA_INST_DSTS_MASK isaWrittenAtLD(ISA_INSTRUCTION i);
    ISA_INST_DSTS_MASK mask = replicate(False);

    if (isaIsLoad(i))
    begin
        mask[0] = True;
    end

    return mask;
endfunction

//
// RISC-v writes nothing no registers at store.
//
function ISA_INST_DSTS_MASK isaWrittenAtST(ISA_INSTRUCTION i) = replicate(False);

//
// Everything else is written at execute.  The mask here doesn't bother to decode the
// instruction.  It must be used in combination with the valid bits returned during
// a full decode.
//
function ISA_INST_DSTS_MASK isaWrittenAtEXE(ISA_INSTRUCTION i);
    ISA_INST_DSTS_MASK all_valid = replicate(True);

    let mask_ld = isaWrittenAtLD(i);
    let mask_st = isaWrittenAtST(i);

    // Return everything not written at load or store.
    return unpack(pack(all_valid) ^ (pack(mask_ld) | pack(mask_st)));
endfunction
