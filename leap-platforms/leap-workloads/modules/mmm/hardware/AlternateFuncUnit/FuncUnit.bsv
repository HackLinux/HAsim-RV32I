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

import Bram::*;
import FIFO::*;
import FIFOF::*;
import Vector::*;
import GetPut::*;
import Types::*;
import Interfaces::*;
import Parameters::*;


interface FuncUnit;
    interface Put#(Op) putInst;
    interface Put#(Data) putData;
    interface Get#(Data) getData;
    interface Get#(Bit#(0)) getZero;
    interface Get#(Bit#(0)) getLoad;
    interface Get#(Bit#(0)) getLoadMul;
    interface Get#(Bit#(0)) getMul;
    interface Get#(Bit#(0)) getStore;
endinterface

//Semantics:
//Zero     : C = 0
//Load     : B = vals from load buffer
//LoadMul  : A = vals from load buffer. C = A*B
//Mul      : C = A*B
//Store    : vals in store buffer = C

interface MultiplyAdder;
    method Action multiplyAddRequest(Data a, Data b, Data c, Bool isLoad);
    method Data multiplyAddResponse();
endinterface

import "BVI" multiplyAdder =
module mkMultiplyAdder(MultiplyAdder);
    no_reset;
    method multiplyAddRequest(x, y, accum, is_load) enable(enable);
    method mulAdd multiplyAddResponse();
    schedule multiplyAddRequest C multiplyAddRequest;
    schedule multiplyAddResponse CF (multiplyAddRequest, multiplyAddResponse);
endmodule

interface MultiplyAdderUnit;
    method Action multiplyAddRequest(Data a, Vector#(FU_BlockRowSize, Data) b, Vector#(FU_BlockRowSize, Data) c, Bool isLoad);
    method Vector#(FU_BlockRowSize, Data) multiplyAddResponse();
endinterface

(* synthesize *)
module mkMultiplyAdderUnit(MultiplyAdderUnit);
    Vector#(FU_BlockRowSize, MultiplyAdder) multiplyAdderVec <- replicateM(mkMultiplyAdder);
    method Action multiplyAddRequest(Data a, Vector#(FU_BlockRowSize, Data) b, Vector#(FU_BlockRowSize, Data) c, Bool isLoad);
        for(Integer i = 0; i < valueOf(FU_BlockRowSize); i=i+1)
            multiplyAdderVec[i].multiplyAddRequest(a, b[i], c[i], isLoad);
    endmethod

    method Vector#(FU_BlockRowSize, Data) multiplyAddResponse();
        Vector#(FU_BlockRowSize, Data) resVec = newVector();
        for(Integer i = 0; i < valueOf(FU_BlockRowSize); i=i+1)
            resVec[i] = multiplyAdderVec[i].multiplyAddResponse();
        return resVec;
    endmethod
endmodule

typedef Bram#(RegFileAddr, Vector#(FU_BlockRowSize, Data)) BramRegFile; 

(* synthesize *)
module mkBramRegFile(BramRegFile);
    Bram#(RegFileAddr, Vector#(FU_BlockRowSize, Data)) bram <- mkBram();
    return bram; 
endmodule

(* synthesize *)
module mkfunctionalunit(FuncUnit);
    FIFOF#(Op)      instFIFO <- mkFIFOF();
    FIFOF#(Data)  dataInFIFO <- mkFIFOF();
    FIFOF#(Data) dataOutFIFO <- mkFIFOF();

    FIFOF#(Bit#(0))    zeroFIFO <- mkFIFOF();
    FIFOF#(Bit#(0))    loadFIFO <- mkFIFOF();
    FIFOF#(Bit#(0)) loadMulFIFO <- mkFIFOF();
    FIFOF#(Bit#(0))     mulFIFO <- mkFIFOF();
    FIFOF#(Bit#(0))   storeFIFO <- mkFIFOF();

    Reg#(Vector#(FU_BlockRowSize, Data)) vectorRegA <- mkRegU();
    Reg#(Vector#(FU_BlockRowSize, Data)) vectorRegB <- mkRegU();
    Reg#(Vector#(FU_BlockRowSize, Data)) vectorRegC <- mkRegU();

    BramRegFile regFileA <- mkBramRegFile();
    BramRegFile regFileB <- mkBramRegFile();
    BramRegFile regFileC <- mkBramRegFile();

    MultiplyAdderUnit multiplyAdderUnit <- mkMultiplyAdderUnit();

    Reg#(Bit#(Plus1LogFU_BlockSize)) counter <- mkRegU();
    Bit#(LogFU_BlockRowSize)       rowCounter = truncate(counter);
    Bit#(LogFU_BlockSize)         tempCounter = truncate(counter);
    Bit#(LogFU_BlockRowSize)    columnCounter = tpl_1(split(tempCounter));

    Bit#(Plus1LogFU_BlockSize) maxCounterVal = 1<<fromInteger(valueOf(LogFU_BlockSize));

    Reg#(Bool)     stateZero <- mkReg(False);
    Reg#(Bool)     stateLoad <- mkReg(False);
    Reg#(Bool)  stateLoadMul <- mkReg(False);
    Reg#(Bool)      stateMul <- mkReg(False);
    Reg#(Bool)    stateStore <- mkReg(False);

    FIFOF#(Data) loadMulPipeline  <- mkLFIFOF();
    Reg#(Bit#(Plus1LogFU_BlockSize)) loadMulCounter <- mkRegU();

    let inst    = instFIFO.first();
    let dataIn  = dataInFIFO.first();

    Vector#(FU_BlockRowSize, Data) vecAdd = multiplyAdderUnit.multiplyAddResponse();

    Bit#(LogFU_BlockRowSize)    currRowCounter = truncate(counter-1);
    Bit#(LogFU_BlockSize)      currTempCounter = truncate(counter-1);
    Bit#(LogFU_BlockRowSize) currColumnCounter = tpl_1(split(currTempCounter));

    Bit#(LogFU_BlockRowSize)    prevRowCounter = truncate(counter-2);
    Bit#(LogFU_BlockSize)      prevTempCounter = truncate(counter-2);
    Bit#(LogFU_BlockRowSize) prevColumnCounter = tpl_1(split(prevTempCounter));

    let isLoad = rowCounter == 1;

    //Zero     : C = 0
    rule zeroStart(inst == Zero && !stateZero);
        stateZero <= True;
        regFileC.zero();
        $display("  real FU: got Zero");
    endrule

    rule zeroEnd(stateZero);
        instFIFO.deq();
        stateZero <= False;
        zeroFIFO.enq(?);
    endrule

    //Load     : B = vals from load buffer
    rule loadStart(inst == Load && !stateLoad);
        stateLoad <= True;
        counter   <= 0;
        $display("  real FU: got Load");
    endrule

    rule loadMiddle(stateLoad && counter != maxCounterVal);
        counter <= counter + 1;
        dataInFIFO.deq();
        vectorRegB[rowCounter] <= dataIn;
        if(rowCounter == 0)
            regFileB.write(columnCounter-1, vectorRegB);
    endrule

    rule loadFinish(stateLoad && counter == maxCounterVal);
        stateLoad <= False;
        regFileB.write(columnCounter-1, vectorRegB);
        instFIFO.deq();
        $display("  real FU: Load Finish");
        loadFIFO.enq(?);
    endrule

    //LoadMul  : A = vals from load buffer. C = A*B
    rule loadMulStart(inst == LoadMul && !stateLoadMul);
        stateLoadMul <= True;
        loadMulCounter <= 0;
        counter <= 0;
        $display("  real FU: got Load Mul");
    endrule

    rule loadMulVectorRequest(stateLoadMul && loadMulCounter != maxCounterVal);
        dataInFIFO.deq();
        loadMulPipeline.enq(dataIn);
        loadMulCounter <= loadMulCounter+1;

        Bit#(LogFU_BlockRowSize)    rowLMCounter = truncate(loadMulCounter);
        Bit#(LogFU_BlockSize)      tempLMCounter = truncate(loadMulCounter);
        Bit#(LogFU_BlockRowSize) columnLMCounter = tpl_1(split(tempLMCounter));

        regFileB.readReq(rowLMCounter);

        if(rowLMCounter == 0)
        begin
            regFileC.readReq(columnLMCounter);
        end
    endrule

    rule loadMulVectorResponse(stateLoadMul);
        loadMulPipeline.deq();
        let loadData = loadMulPipeline.first();
        vectorRegA[rowCounter] <= loadData;
        vectorRegB <= regFileB.readResp();
        counter <= counter + 1;

        multiplyAdderUnit.multiplyAddRequest(vectorRegA[currRowCounter], vectorRegB, vectorRegC, isLoad);

        if(counter > 1)
            regFileC.write(prevColumnCounter, vecAdd);

        if(rowCounter == 0)
            vectorRegC <= regFileC.readResp();
        /*
        else
            vectorRegC <= vecAdd;
        */

        if(rowCounter == 0)
            regFileA.write(columnCounter-1, vectorRegA);
    endrule

    rule loadMulFinish(stateLoadMul && counter == maxCounterVal);
        multiplyAdderUnit.multiplyAddRequest(vectorRegA[currRowCounter], vectorRegB, vectorRegC, isLoad);
        regFileC.write(prevColumnCounter, vecAdd);
        regFileA.write(currColumnCounter, vectorRegA);
        counter <= counter + 1;
    endrule

    rule loadMulRealFinish(stateLoadMul && counter == maxCounterVal+1);
        stateLoadMul <= False;
        regFileC.write(prevColumnCounter, vecAdd);
        instFIFO.deq();
        $display("  real FU: Load Mul Finish");
        loadMulFIFO.enq(?);
    endrule

    //Mul      : C = A*B
    rule mulStart(inst == Mul && !stateMul);
        stateMul <= True;
        counter <= 0;
        regFileA.readReq(0);
        regFileB.readReq(0);
        regFileC.readReq(0);
        $display("  real FU: got Mul");
    endrule

    rule mulMiddle(stateMul && counter != maxCounterVal);
        if(rowCounter == 0)
            vectorRegA <= regFileA.readResp();
        vectorRegB <= regFileB.readResp();
        counter <= counter + 1;

        multiplyAdderUnit.multiplyAddRequest(vectorRegA[currRowCounter], vectorRegB, vectorRegC, isLoad);

        if(counter > 1)
            regFileC.write(prevColumnCounter, vecAdd);

        regFileA.readReq(columnCounter+1);
        regFileB.readReq(rowCounter+1);

        if(rowCounter == 0)
            vectorRegC <= regFileC.readResp();
        /*
        else
            vectorRegC <= vecAdd;
        */

        regFileC.readReq(columnCounter+1);
    endrule

    rule mulFinish(stateMul && counter == maxCounterVal);
        multiplyAdderUnit.multiplyAddRequest(vectorRegA[currRowCounter], vectorRegB, vectorRegC, isLoad);
        regFileC.write(prevColumnCounter, vecAdd);
        counter <= counter + 1;
    endrule

    rule mulRealFinish(stateMul && counter == maxCounterVal+1);
        stateMul <= False;
        regFileC.write(prevColumnCounter, vecAdd);
        instFIFO.deq();
        mulFIFO.enq(?);
    endrule

    //Store    : vals in store buffer = C
    rule storeStart(inst == Store && !stateStore);
        stateStore <= True;
        counter <= 0;
        regFileC.readReq(0);
    endrule

    rule storeMiddle(stateStore && counter != maxCounterVal);
        let vecVal = regFileC.readResp();
        if(rowCounter == 0)
            vectorRegC <= vecVal;
        counter <= counter + 1;
        regFileC.readReq(columnCounter+1);
        if(counter != 0)
            dataOutFIFO.enq(vectorRegC[rowCounter-1]);
    endrule

    rule storeFinish(stateStore && counter == maxCounterVal);
        stateStore <= False;
        dataOutFIFO.enq(vectorRegC[rowCounter-1]);
        instFIFO.deq();
        storeFIFO.enq(?);
        $display("  real FU: FU finished store");
    endrule

    interface putInst    = fifoToPut(fifofToFifo(instFIFO));
    interface putData    = fifoToPut(fifofToFifo(dataInFIFO));
    interface getData    = fifoToGet(fifofToFifo(dataOutFIFO));
    interface getZero    = fifoToGet(fifofToFifo(zeroFIFO));
    interface getLoad    = fifoToGet(fifofToFifo(loadFIFO));
    interface getLoadMul = fifoToGet(fifofToFifo(loadMulFIFO));
    interface getMul     = fifoToGet(fifofToFifo(mulFIFO));
    interface getStore   = fifoToGet(fifofToFifo(storeFIFO));
endmodule
