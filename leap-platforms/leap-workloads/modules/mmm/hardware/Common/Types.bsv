/*
Copyright (c) 2007 MIT

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

Author: Kermin Fleming
*/

// Global imports
import GetPut::*;
import FIFO::*;
import Vector::*;
import Complex16::*;

import Parameters::*;

// Typdefs for various bit types

typedef Complex16 ComplexWord;

typedef enum
{
  A,
  B, 
  C
} MatrixRegister
    deriving(Bits,Eq);

typedef enum
{
  Multiply,
  Zero,
  MultiplyAddAccumulate,
  MultiplySubAccumulate,
  AddAccumulate,
  SubAccumulate  
} 
  FunctionalUnitOp
    deriving(Bits,Eq);

typedef union tagged
{
  Bit#(LogRowSize) RowSize;
  BlockAddr LoadPage;
  BlockAddr StorePage;
} 
  PLBMasterCommand
    deriving(Bits,Eq);

typedef union tagged
{
  FunctionalUnitAddr StoreFromFU;
  FunctionalUnitMask LoadToFUs;
} 
  MemorySwitchCommand 
    deriving(Bits,Eq);

typedef struct
{
  FunctionalUnitAddr  fuSrc;   
  MatrixRegister      regSrc;
  FunctionalUnitMask  fuDests;
  MatrixRegister      regDest;
} 
  FUNetworkCommand 
    deriving(Bits,Eq);

typedef union tagged
{  
  MatrixRegister   ForwardSrc;
  MatrixRegister   ForwardDest;
  MatrixRegister   Load;
  MatrixRegister   Store;
  FunctionalUnitOp Op;  
  
} FunctionalUnitCommand
    deriving(Bits,Eq);

typedef union tagged 
{  
  struct{
    FunctionalUnitMask    fus;
    FunctionalUnitOp      op;
  } ArithmeticInstruction;

  struct
  {
    FunctionalUnitMask fus;
    MatrixRegister     regName;
    PLBAddr            addr;
  } LoadInstruction;

  struct
  {
    FunctionalUnitAddr fu;
    MatrixRegister     regName;
    PLBAddr            addr;
  } StoreInstruction;

  struct 
  { 
    FunctionalUnitAddr  fuSrc;   
    MatrixRegister      regSrc;
    FunctionalUnitMask  fuDests;
    MatrixRegister      regDest;
  } ForwardInstruction;     

  Bit#(LogRowSize) SetRowSizeInstruction;
  void             SyncInstruction;
  void             FinishInstruction;
} Instruction
    deriving(Bits,Eq);

/*

Template for cases on Instruction.

  case (ins) matches
    tagged ArithmeticInstruction .i:   //{.fus, .op}
    tagged LoadInstruction .i:         //{.fus, .regName, .addr}
    tagged StoreInstruction .i:        //{.fus, .regName, .addr}
    tagged ForwardInstruction .i:      //{.fuSrc, .regSrc, .fuDests, .regDest}
    tagged SetRowSizeInstruction .sz:
  endcase

*/

//A message to the PPC
typedef Bit#(32) PPCMessage;


typedef enum{
  FI_InIdle,	     
  FI_InStartCheckRead,
  FI_InStartRead,
  FI_InStartTake,
  FI_OutStartCheckWrite,
  FI_OutStartWrite,
  FI_OutStartPush,
  FI_CheckLoadStore,
  FI_Load,
  FI_LoadTake,
  FI_Store,
  FI_StorePush,
  FI_command
} FeederState deriving(Eq,Bits);     

function Maybe#(BlockAddr) translateLoad(Instruction inst);
  if(inst matches tagged LoadInstruction .l) 
    begin
      return tagged Valid (truncate(l.addr)); 
    end
  else 
    begin
      return tagged Invalid;
    end
endfunction

function Maybe#(BlockAddr) translateStore(Instruction inst);
  if(inst matches tagged StoreInstruction .l) 
    begin
      return tagged Valid (truncate(l.addr)); 
    end
  else 
    begin
      return tagged Invalid;
    end
endfunction

function Maybe#(BlockAddr) translateRowSize(Instruction inst);
  if(inst matches tagged SetRowSizeInstruction .sz) 
    begin
      return tagged Valid (1 << sz); 
    end
  else 
    begin
      return tagged Invalid;
    end
endfunction

// Local imports
interface Controller;

  interface Get#(PLBMasterCommand)     plbMasterCommandOutput;
  interface Get#(MemorySwitchCommand)  memorySwitchCommandOutput;
  interface Get#(FUNetworkCommand)     fuNetworkCommandOutput;
  interface Vector#(FunctionalUnitNumber, Get#(FunctionalUnitCommand)) functionalUnitCommandOutputs;

endinterface


interface FunctionalUnit;
  interface Put#(FunctionalUnitCommand) functionalUnitCommandInput;
  interface FUNetworkLink               link;
  interface Put#(ComplexWord)           switchInput;
  interface Get#(ComplexWord)           switchOutput;
endinterface


interface MemorySwitch;
  interface Put#(ComplexWord)              plbMasterComplexWordInput;
  interface Get#(ComplexWord)              plbMasterComplexWordOutput;
  interface Put#(MemorySwitchCommand)      memorySwitchCommandInput;
  interface Vector#(FunctionalUnitNumber, 
                    Get#(ComplexWord))     functionalUnitComplexWordOutputs;
  interface Vector#(FunctionalUnitNumber, 
                    Put#(ComplexWord))     functionalUnitComplexWordInputs;

endinterface


interface PLBMaster;
  interface Put#(ComplexWord) wordInput;
  interface Get#(ComplexWord) wordOutput;
  interface Put#(PLBMasterCommand) plbMasterCommandInput;
endinterface

interface FUNetwork;

  interface Put#(FUNetworkCommand) fuNetworkCommandInput;

endinterface


interface FUNetworkLink;

  interface Get#(ComplexWord) a_out;
  interface Get#(ComplexWord) b_out;
  interface Get#(ComplexWord) c_out;
  
  interface Put#(ComplexWord) a_in;
  interface Put#(ComplexWord) b_in;
  interface Put#(ComplexWord) c_in;

endinterface

