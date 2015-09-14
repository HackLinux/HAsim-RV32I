
// isa_datapath_template

// The RISC-V datapath.

// 23/07/2015 - Cong Minh thanh


// ***** Imports *****

import Vector::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/common_services.bsh"

`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/funcp_interface.bsh"
`include "asim/provides/isa_emulator.bsh"

`include "asim/rrr/remote_client_stub_ISA_REGOP_EMULATOR.bsh"
`include "asim/rrr/remote_client_stub_ISA_DP_DEBUG.bsh"
`include "asim/dict/PARAMS_HASIM_ISA_DATAPATH.bsh"

// ***** Modules *****

// mkISA_Datapath

// The datapath module itself.

module [HASIM_MODULE] mkISA_Datapath 
  //interface:
              ();

    // ***** Dynamic parameters *****
    PARAMETER_NODE paramNode <- mkDynamicParameterNode();
    Param#(1) debugISADP <- mkDynamicParameter(`PARAMS_HASIM_ISA_DATAPATH_DEBUG_ISA_DP, paramNode);
    function Bool enableISADebug = (debugISADP != 0);

    // Debug RRR client
    ClientStub_ISA_DP_DEBUG debugClient <- mkClientStub_ISA_DP_DEBUG();

    // ***** Soft Connections *****

    // Connection to the functional partition.
    
    Connection_Server#(FUNCP_ISA_DATAPATH_REQ, 
                       FUNCP_ISA_DATAPATH_RSP) link_fp <- mkConnection_Server("isa_datapath");

    Connection_Receive#(FUNCP_ISA_DATAPATH_SRCVALS) link_fp_srcvals <- mkConnection_Receive("isa_datapath_srcvals");
    Connection_Send#(FUNCP_ISA_WRITEBACK) link_fp_writeback <- mkConnection_Send("isa_datapath_writeback");

    // Emulation RRR Stubs (not used for RV32I)
    ClientStub_ISA_REGOP_EMULATOR emulClient <- mkClientStub_ISA_REGOP_EMULATOR();

    // ***** Debugging Log *****
    
    // This logfile is available for debugging during Bluesim simulation.
    // It has no affect on the FPGA.
    
    let debug_log <- mkReg(InvalidFile);


    // ***** Rules ******

    
    // openLog
    
    // Opens the logfile for writing. The filename is an AWB parameter.

    rule open_log (debug_log == InvalidFile);

        let fd <- $fopen(`HASIM_ISA_DP_LOGFILE, "w");

        if (fd == InvalidFile)
        begin
          $display("ERROR: ISA: Datapath: Could not create logfile %s", `HASIM_ISA_DP_LOGFILE);
          $finish(1);
        end

        debug_log <= fd;

    endrule


    // datapathExec

    // Execute RISC-V instructions. This is currently unpipelined combinational logic.
    // Parameters: 
    //    * Instruction
    //    * Address (program counter)
    //    * List of source values from registers
    // Returns: 
    //    * Instruction result (given to timing partition)
    //    * Effective address (unused if not a load/store)
    //    * List of values to writeback

    rule datapathExec (True);

        // Get the request from the functional partition.
        let req  = link_fp.getReq();
        link_fp.deq();
        let inst = req.instruction;
        let addr = req.instAddress;
        
        // Register input values come directory from the physial register file
        let reqSrcs = link_fp_srcvals.receive();
        link_fp_srcvals.deq();
        let srcs = reqSrcs.srcValues;

        // Some convenient variables to return.

        FUNCP_ISA_EXECUTION_RESULT timep_result = RNop;
        ISA_ADDRESS effective_addr = 0;
        Maybe#(ISA_VALUE) writeback = tagged Invalid;
        
        // Calculate a sign-extended immediate used by many operations.
 //       ISA_VALUE sign_ext_imm = signExtend(inst[15:0]);
        
        // Calculate a zero-extended offset used by many logical operations.
//       ISA_VALUE zero_ext_imm = zeroExtend(inst[15:0]);

        // Calculate a sign-extended address used by loads/stores.
 //       ISA_ADDRESS sign_ext_offset = signExtend(inst[15:0]);
        
        // A shifted offset used by many branches.
 //       ISA_ADDRESS shifted_ext_offset = sign_ext_offset << 2;

        // Calculate the PC + 4 for many branch instructions.
        ISA_ADDRESS branch_not_taken_dest = addr + 4;
        


	// J-immediate encodes
	ISA_ADDRESS decoded_imm_uj = signExtend({inst[31], inst[19:12], inst[20], inst[30:21], 1'b0});
	//{ decoded_imm_uj[31:20], decoded_imm_uj[10:1], decoded_imm_uj[11], decoded_imm_uj[19:12], decoded_imm_uj[0] } = signExtend({inst[31:12], 1'b0});
	

	// 12-bit signed I-immediate
	ISA_ADDRESS decoded_imm_i = signExtend(inst[31:20]);

	// immediate SB-type
	ISA_ADDRESS decoded_imm_sb = signExtend({inst[31], inst[7], inst[30:25], inst[11:8], 1'b0});

	// immediate S-type
	ISA_ADDRESS decoded_imm_s = signExtend({inst[31:25], inst[11:7]});

	// immediate U-type
	ISA_ADDRESS decoded_imm_U = zeroExtend(inst[31:12]);

	// Calculate the branch-taken destination for conditional branches.
        ISA_ADDRESS branch_taken_dest = addr + decoded_imm_sb;

	// Calculate a zero-extended offset used by many logical operations.
        ISA_VALUE zero_ext_imm = zeroExtend(inst[31:20]); 


	

        RV32I_OPCODE op = inst[6:0];

        if (enableISADebug())
        begin
            debugClient.makeRequest_noteInstr(
               contextIdToRRR(tokContextId(req.token)),
               inst,
               addr,
               srcs[0],
               srcs[1]);
        end

        case (op)


		// LUI
		// LUI (load upper immediate) is used to build 32-bit 			
		// constants and uses the U-type format. LUI
		// places the U-immediate value in the top 20 bits of the  			// destination register rd, filling in the lowest
		// 12 bits with zeros.Does not touch memory
		rv32iLUI:
		begin
			ISA_VALUE result = decoded_imm_U << 12;
			writeback = tagged Valid result;
			 // Log it.
			$fdisplay(debug_log, "0x%h:  LUI 0x%h = 0x%h << 12", addr, result, decoded_imm_U);
			// No information needs to be returned to the timing partition.
			timep_result = tagged RNop;
			
		end

		// AUIPC ?????
		// AUIPC (add upper immediate to pc) is used to build pc-relative addresses and uses the U-type
		// format. AUIPC forms a 32-bit offset from the 20-bit U-immediate, filling in the lowest 12 bits with
		// zeros, adds this offset to the pc, then places the result in register rd.
		rv32iAUIPC:
		begin
			ISA_VALUE result = {inst[31:12], 12'b000000000000};
			ISA_ADDRESS dest = addr + result;
			writeback = tagged Valid dest;
			 // Log it.
			$fdisplay(debug_log, "0x%h:  AUIPC 0x%h = 0x%h + 0x%h", addr, dest, addr, result);
			// No information needs to be returned to the timing partition.
			timep_result = tagged RNop;
			
		end

		// JAL
		// The offset is sign-extended and added to the pc to form the jump target 			
		// address. JAL stores the address of the instruction following the jump (pc+4) 		
		// into register rd
		rv32iJAL:
		begin
			// Do the address calculation.
			ISA_ADDRESS dest = addr + decoded_imm_uj;
			// Write the old PC into our destination.
			writeback = tagged Valid branch_not_taken_dest;
			// Log it.
			$fdisplay(debug_log, "0x%h:  JAL PC <= 0x%h = {%0h + %0h}, (Old PC: 0x%h)", addr, dest, addr, decoded_imm_uj, branch_not_taken_dest);
			// Return the branch result to the timing partition.
			timep_result = tagged RBranchTaken dest;
		end

		// JALR
		// The target address is obtained by adding the 12-bit signed I-immediate to the 			
		// register rs1, then setting the least-significant bit of the result to zero.
		rv32iJALR:
		begin
			// Do the address calculation.
			ISA_ADDRESS dest = decoded_imm_i + srcs[0];
			dest = {dest[31:1], 1'b0};
			// Write the old PC back to our destination.
			writeback = tagged Valid (branch_not_taken_dest);
			// Log it.
			$fdisplay(debug_log, "0x%h:  JALR PC <= 0x%h = 0x%h + 0x%h, (Old PC: 0x%h)", addr, dest, decoded_imm_i, srcs[0] , branch_not_taken_dest);
			// Tell the timing partition the branch was taken.
			timep_result = tagged RBranchTaken dest;

			//???????

			if ((srcs[0] == 0) && (decoded_imm_i == 0))
			begin
				//src1==1 means we passed
				Bool pf = True;
			//	timep_result = tagged RTerminate pf;
				timep_result =  tagged RTerminate pf ;

			end

		end
		
		rv32iBRANCH:
		begin
			RV32I_FUNCT3 branch_op = inst[14:12];
			case (branch_op)
				rv32iF3BEQ:
				begin
					// BEQ
					// take the branch if registers rs1 and rs2 are equal
					Bool taken = srcs[0] == srcs[1];
					// Log it.
					$fdisplay(debug_log, "0x%h:  BEQ PC <= 0x%h (offset 0x%h) if (0x%h == 0x%h)", addr, branch_taken_dest, decoded_imm_sb, srcs[0], srcs[1]);
					// Tell the timing partition if the branch was taken or not.
					timep_result = taken ? (tagged RBranchTaken branch_taken_dest) : (tagged RBranchNotTaken branch_not_taken_dest);	
				end

				rv32iF3BNE:
				begin
					// Branch is taken if src1 != src2.
					Bool taken = srcs[0] != srcs[1];
					// Log it.
					$fdisplay(debug_log, "0x%h:  BNE PC <= 0x%h (offset 0x%h) if (0x%h != 0x%h)", addr, branch_taken_dest, decoded_imm_sb, srcs[0], srcs[1]);
					// Tell the timing partition if the branch was taken or not.
					timep_result = taken ? (tagged RBranchTaken branch_taken_dest) : (tagged RBranchNotTaken branch_not_taken_dest);
				end

				rv32iF3BLT:
				begin
					// BLT and BLTU take the branch if rs1 is less than rs2, using signed and unsigned comparison respectively
					Bool taken = signedLT(srcs[0], srcs[1]);
                        		// Log it.
                        		$fdisplay(debug_log, "0x%h:  BLT PC <= 0x%h (offset 0x%h) if (0x%h < 0x%h)", addr, branch_taken_dest, decoded_imm_sb, srcs[0], srcs[1]);
                        		// Tell the timing partition if the branch was taken or not.
                        		timep_result = taken ? (tagged RBranchTaken branch_taken_dest) : (tagged RBranchNotTaken branch_not_taken_dest);

                    		end

				rv32iF3BLTU:
				begin
					// BLT and BLTU take the branch if rs1 is less than rs2, using signed and unsigned comparison respectively
                        		Bool taken = srcs[0] < srcs[1];
                        		// Log it.
                        		$fdisplay(debug_log, "0x%h:  BLTU PC <= 0x%h (offset 0x%h) if (0x%h < 0x%h)", addr, branch_taken_dest, decoded_imm_sb, srcs[0], srcs[1]);
                        		// Tell the timing partition if the branch was taken or not.
                        		timep_result = taken ? (tagged RBranchTaken branch_taken_dest) : (tagged RBranchNotTaken branch_not_taken_dest);

                    		end

				rv32iF3BGE:
				begin
					// BGE and BGEU take the branch if rs1 is greater than or equal to rs2, using signed and unsigned comparison respectively
                        		Bool taken = signedGE(srcs[0], srcs[1]);
                        		// Log it.
                        		$fdisplay(debug_log, "0x%h:  BGE PC <= 0x%h (offset 0x%h) if (0x%h >= 0x%h)", addr, branch_taken_dest, decoded_imm_sb, srcs[0], srcs[1]);
                        		// Tell the timing partition if the branch was taken or not.
                        		timep_result = taken ? (tagged RBranchTaken branch_taken_dest) : (tagged RBranchNotTaken branch_not_taken_dest);
                    		end

				rv32iF3BGEU:
				begin
					// BGE and BGEU take the branch if rs1 is greater than or equal to rs2, using signed and unsigned comparison respectively
					Bool taken = srcs[0] >= srcs[1];
                        		// Log it.
                        		$fdisplay(debug_log, "0x%h:  BGEU PC <= 0x%h (offset 0x%h) if (0x%h < 0x%h)", addr, branch_taken_dest, decoded_imm_sb, srcs[0], srcs[1]);
                        		// Tell the timing partition if the branch was taken or not.
                        		timep_result = taken ? (tagged RBranchTaken branch_taken_dest) : (tagged RBranchNotTaken branch_not_taken_dest);
                    		end
			endcase

		end

		rv32iLOAD:
		begin
			// Load (LB-LH-LW-LBU-LHU). (Effective Address calculation only.)
			// The effective byte address is obtained by adding register rs1 to the sign-extended 12-bit offset
			effective_addr = srcs[0] + decoded_imm_i;
                	// Log it.
                	$fdisplay(debug_log, "0x%h:  LW EADDR 0x%h = 0x%h + 0x%h", addr, effective_addr, srcs[0], decoded_imm_i);
                	// Return the effective address to the timing partition.
                	timep_result = tagged REffectiveAddr effective_addr;
		end

		rv32iSTORE:
		begin
                	// Store (SB-SH-SW) Calculate the Effective Address.
                	effective_addr = srcs[0] + decoded_imm_s;
                	// Log it.
			// The SW, SH, and SB instructions store 32-bit, 16-bit, and 8-bit values from the low bits of register rs2 to memory.
                	$fdisplay(debug_log, "0x%h:  SW EADDR 0x%h = 0x%h + 0x%h <= 0x%h", addr, effective_addr, srcs[0], decoded_imm_s, srcs[1]);
                	// Return the effective address to the timing partition.
                	timep_result = tagged REffectiveAddr effective_addr;
                	// Write src2 into our destination.
                	// No one can observe this but it allows the doStores operation to 
                	// avoid reading the maptable.
                	writeback = tagged Valid srcs[1];
           	 end
		
		// Integer Register-Immediate Operations
		rv32iOPIMM:
		begin
			RV32I_FUNCT3 imm_op = inst[14:12];
			case (imm_op)
				rv32iF3ADDI:
				begin
					// Do the addition and write it back to our dest.
					// ADDI adds the sign-extended 12-bit immediate to register rs1
					ISA_VALUE result = srcs[0] + decoded_imm_i;
					writeback = tagged Valid result;
					// Log it.
					$fdisplay(debug_log, "0x%h:  ADDI 0x%h = 0x%h + 0x%h]", addr, result, srcs[0], decoded_imm_i);
					// No information needs to be returned to the timing partition.
					timep_result = tagged RNop;
				end
				
				rv32iF3SLTI:
				begin
					// Do the calculation.
					ISA_VALUE result = zeroExtend(pack(signedLT(srcs[0], decoded_imm_i)));
					writeback = tagged Valid result;
					// Log it.
					$fdisplay(debug_log, "0x%h:  SLTI 0x%h = slt(0x%h, 0x%h)", addr, result, srcs[0], decoded_imm_i);
					// No information needs to be returned to the timing partition.
					timep_result = tagged RNop;
				end
				rv32iF3SLTIU:
				begin
					// Do the calculation.
					ISA_VALUE result = zeroExtend(pack(srcs[0] < decoded_imm_i));
					writeback = tagged Valid result;
					// Log it.
					$fdisplay(debug_log, "0x%h:  SLTIU 0x%h = 0x%h < 0x%h", addr, result, srcs[0], decoded_imm_i);
					// No information needs to be returned to the timing partition.
					timep_result = tagged RNop;
				end
				rv32iF3XORI:
				begin
					// Do the XOR and write it back to our destination.
					ISA_VALUE result = srcs[0] ^ decoded_imm_i;
					writeback = tagged Valid result;
					// Log it.
					$fdisplay(debug_log, "0x%h:  XORI 0x%h = 0x%h ^ 0x%h", addr, result, srcs[0], decoded_imm_i);
					timep_result = tagged RNop;
				end

				rv32iF3ORI:
				begin
					// Do the OR and write it back to our destination.
					ISA_VALUE result = srcs[0] | decoded_imm_i;
					writeback = tagged Valid result;
					// Log it.
					$fdisplay(debug_log, "0x%h:  ORI 0x%h = 0x%h | 0x%h", addr, result, srcs[0], decoded_imm_i);
					// No information needs to be returned to the timing partition.
					timep_result = tagged RNop;
				end

				rv32iF3ANDI:
				begin
					// Do the AND and write it back to our destination.
					ISA_VALUE result = srcs[0] & decoded_imm_i;
					writeback = tagged Valid result;
					// Log it.
					$fdisplay(debug_log, "0x%h:  ANDI 0x%h = 0x%h & 0x%h", addr, result, srcs[0], decoded_imm_i);
					// No information needs to be returned to the timing partition.
					timep_result = tagged RNop;
				end

				rv32iF3SLLI:
				begin
					// Get the shift amount from the instruction.
					Bit#(5) shift_amount = inst[24:20];
					// Do the calculation and write it back.
					ISA_VALUE result = srcs[0] << shift_amount;
					writeback = tagged Valid result;
					// Log it.
					$fdisplay(debug_log, "0x%h:  SLLI 0x%h = 0x%h << 0x%h", addr, result, srcs[0], shift_amount);
					// The timing partition does not need any information.
					timep_result = tagged RNop;
				end
				rv32iF3_SRLISRAI:
				begin
					RV32I_FUNCT7 shr_op = inst[31:25];
					case (shr_op)
						rv32iF7SRLI:
						begin
							// Get the shift amount from the instruction.
							Bit#(5) shift_amount = inst[24:20];
							// Do the calculation and write it back.
							ISA_VALUE result = srcs[0] >> shift_amount;
							writeback = tagged Valid result;
							// Log it.
							$fdisplay(debug_log, "0x%h:  SRL 0x%h = 0x%h >> 0x%h", addr, result, srcs[0], shift_amount);
							// The timing partition does not need any information.
							timep_result = tagged RNop;
						end

						rv32iF7SRAI:
						begin
							// Get the shift amount from the instruction.
							Bit#(5) shift_amount = inst[24:20];
							// Do the calculation and write it back.
							ISA_VALUE result = signedShiftRight(srcs[0], shift_amount);
							writeback = tagged Valid result;
							// Log it.
							$fdisplay(debug_log, "0x%h:  SRAI 0x%h = 0x%h >> a 0x%h", addr, result, srcs[0], shift_amount);
							// The timing partition does not need any information.
							timep_result = tagged RNop;
						end
					endcase
			
				end
			endcase
		end

		// Integer Register-Register Operations
		rv32iOP:
		begin
			RV32I_FUNCT3 rr_op = inst[14:12];
			case (rr_op)
			rv32iF3_ADDSUB:
			begin
				RV32I_FUNCT7 addSub_op = inst[31:25];
				case (addSub_op)
					rv32iF7ADD:
					begin
						// Do the addition.
						ISA_VALUE result = srcs[0] + srcs[1];
						writeback = tagged Valid result;
						// Log it.
						$fdisplay(debug_log, "0x%h:  ADDU %0h = %0h + %0h", addr, result, srcs[0], srcs[1]);
						// The timing partition does not need any information.
						timep_result = tagged RNop;
					end
					rv32iF7SUB:
					begin
						// Do the subtraction.
						ISA_VALUE result = srcs[0] - srcs[1];
						writeback = tagged Valid result;
						// Log it.
						$fdisplay(debug_log, "0x%h:  SUBU %0h = %0h - %0h", addr, result, srcs[0], srcs[1]);
						// The timing partition does not need any information.
						timep_result = tagged RNop;
					end

				endcase
			end

			rv32iF3SLL:
			begin
				// Get the shift amount from the instruction.
				// the shift amount held in the lower 5 bits of register rs2
				Bit#(5) shift_amount = srcs[1][4:0];
				// Do the calculation and write it back.
				ISA_VALUE result = srcs[0] << shift_amount;
				writeback = tagged Valid result;
				// Log it.
				$fdisplay(debug_log, "0x%h:  SLL 0x%h = 0x%h << 0x%h", addr, result, srcs[0], shift_amount);
				// The timing partition does not need any information.
				timep_result = tagged RNop;
			end
			rv32iF3SLT:
			begin
				// Set if src1 < src2.
				ISA_VALUE result = zeroExtend(pack(srcs[0] < srcs[1]));
				writeback = tagged Valid result;
				// Log it.
				$fdisplay(debug_log, "0x%h:  SLT %0h = sltu(%0h, %0h)", addr, result, srcs[0], srcs[1]);
				// The timing partition does not need any information.
				timep_result = tagged RNop;
			end

			rv32iF3SLTU:
			begin
				// Set if src1 < src2 (signed).
				ISA_VALUE result = zeroExtend(pack(signedLT(srcs[0], srcs[1])));
				writeback = tagged Valid result;
				// Log it.
				$fdisplay(debug_log, "0x%h:  SLTU %0h = slt(%0h, %0h)", addr, result, srcs[0], srcs[1]);
				// The timing partition does not need any information.
				timep_result = tagged RNop;
			end

			rv32iF3XOR:
			begin
				// Do the XOR.
				ISA_VALUE result = srcs[0] ^ srcs[1];
				writeback = tagged Valid result;
				// Log it.
				$fdisplay(debug_log, "0x%h:  XOR %0h = %0h ^ %0h", addr, result, srcs[0], srcs[1]);
				// The timing partition does not need any information.
				timep_result = tagged RNop;
			end

			rv32iF3_SRLSRA:
			begin
				RV32I_FUNCT7 shra_op = inst[31:25];
				case (shra_op)
					rv32iF7SRL:
					begin
						// Get the shift amount from the instruction.
						Bit#(5) shift_amount = srcs[1][4:0];
						// Do the calculation and write it back.
						ISA_VALUE result = srcs[0] >> shift_amount;
						writeback = tagged Valid result;
						// Log it.
						$fdisplay(debug_log, "0x%h:  SRL 0x%h = 0x%h >> 0x%h", addr, result, srcs[0], shift_amount);
						// The timing partition does not need any information.
						timep_result = tagged RNop;
					end
					rv32iF7SRA:
					begin
						// Get the shift amount from the instruction.
						Bit#(5) shift_amount = srcs[1][4:0];
						// Do the calculation and write it back.
						ISA_VALUE result = signedShiftRight(srcs[0], shift_amount);
						writeback = tagged Valid result;
						// Log it.
						$fdisplay(debug_log, "0x%h:  SRA 0x%h = 0x%h >>a 0x%h", addr, result, srcs[0], shift_amount);
						// The timing partition does not need any information.
						timep_result = tagged RNop;
					end

				endcase
			end
			rv32iF3OR:
			begin
				// Do the OR.
				ISA_VALUE result = srcs[0] | srcs[1];
				writeback = tagged Valid result;
				// Log it.
				$fdisplay(debug_log, "0x%h:  OR %0h = %0h | %0h", addr, result, srcs[0], srcs[1]);
				// The timing partition does not need any information.
				timep_result = tagged RNop;
			end
			
			rv32iF3AND:
			begin
				// Do the AND.
				ISA_VALUE result = srcs[0] & srcs[1];
				writeback = tagged Valid result;
				// Log it.
				$fdisplay(debug_log, "0x%h:  AND %0h = %0h & %0h", addr, result, srcs[0], srcs[1]);
				// The timing partition does not need any information.
				timep_result = tagged RNop;
			end
			

					
			endcase

		end
            
            default
            begin
            
                // An illegal instruction
                $fdisplay(debug_log, "0x%h:  WARNING: EXECUTING ILLEGAL INSTRUCTION: %0h", addr, inst);
            
            end

        endcase

        // Return the result to the functional partition.
        link_fp.makeResp(initISADatapathRsp(FUNCP_ISA_EXCEPT_NONE,
                                            timep_result));

        // Send the writeback.  For RV32I there is at most one register written,
        // so the operation is simple.
        FUNCP_ISA_WRITEBACK wb_msg;
        if (writeback matches tagged Valid .val)
        begin
            // Valid writeback value.
            wb_msg = initISAWriteback(req.token, req.instDstPhysRegs[0], val, True);
        end
        else
        begin
            // Nothing to say.  Still need a "done" message.
            wb_msg = initISAWriteback(req.token, tagged Invalid, ?, True);
        end

        link_fp_writeback.send(wb_msg);
    endrule

endmodule
