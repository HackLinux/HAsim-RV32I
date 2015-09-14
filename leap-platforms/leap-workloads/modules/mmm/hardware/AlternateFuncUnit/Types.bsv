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

Author: Muralidaran Vijayaraghavan
*/

typedef 16 FU_BlockRowSize;
typedef Bit#(32) Data;
typedef TLog#(FU_BlockRowSize) LogFU_BlockRowSize;
typedef TMul#(FU_BlockRowSize, FU_BlockRowSize) FU_BlockSize;
typedef TLog#(FU_BlockSize) LogFU_BlockSize;
typedef TAdd#(LogFU_BlockSize,1) Plus1LogFU_BlockSize;
typedef Bit#(TLog#(FU_BlockRowSize)) RegFileAddr;

typedef enum {Zero, Load, LoadMul, Mul, Store, LoadAddr, StoreAddr, SetRowSize} Op deriving (Bits, Eq);

//The following types are for integration purposes
typedef struct {
    Bit#(30) addr;
    Bit#(29) zero;
    Op op;
} Inst deriving(Eq,Bits);

typedef Inst Instruction;

typedef Bit#(32) PPCMessage;

typedef TMul#(BlockSize, BlockSize) BlockElements;
typedef TLog#(TMul#(BlockSize, BlockSize)) LogBlockElements;
typedef 1024 MaxBlockSize;
typedef TAdd#(TLog#(MaxBlockSize), 2) LogRowSize;

typedef 32 PLBAddrSize;
typedef Bit#(PLBAddrSize) PLBAddr;
typedef 16 BurstSize;

typedef Bit#(30) BlockAddr;

typedef Data ComplexWord;


typedef union tagged
{
  Bit#(LogRowSize) RowSize;
  BlockAddr LoadPage;
  BlockAddr StorePage;
} 
  PLBMasterCommand
    deriving(Bits,Eq);


typedef enum{
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
  if(inst.op == LoadAddr) 
    begin
      return tagged Valid (truncate(inst.addr)); 
    end
  else 
    begin
      return tagged Invalid;
    end
endfunction

function Maybe#(BlockAddr) translateStore(Instruction inst);
  if(inst.op == StoreAddr) 
    begin
      return tagged Valid (truncate(inst.addr)); 
    end
  else 
    begin
      return tagged Invalid;
    end
endfunction

function Maybe#(BlockAddr) translateRowSize(Instruction inst);
  if(inst.op == SetRowSize) 
    begin
      return tagged Valid (truncate(1 << inst.addr)); 
    end
  else 
    begin
      return tagged Invalid;
    end
endfunction

typedef FU_BlockRowSize BlockSize;
typedef LogFU_BlockRowSize LogBlockSize;
