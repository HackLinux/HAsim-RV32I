/*
Copyright (c) 2009 MIT

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

Author: Asif Khan
*/

import FIFOF::*;


typedef TAdd#(IntWidthData, FracWidthData) DataWidth;
typedef TAdd#(IntWidthTheta, FracWidthTheta) TDataWidth;


typedef struct {
    Bit#(1)  rnw;
    Bit#(32) addr;
    Bit#(64) data;
    Bit#(8)  be;
} TypeMainMemReq deriving (Bits, Eq);

typedef enum{
    Idle,
    Start,
    Working
} CartPolState deriving (Bits,Eq);

typedef enum{
    GetPos,
    LdReq1,
    LdReq2,
    LdReq3,
    LdReq4,
    StReq
} ConState deriving (Bits,Eq);

interface ICartPol;
    method Action nValue(Bit#(32) _n);
    method Action r0Value(Bit#(32) r0);
    method Action r1Value(Bit#(32) r1);
    method Action theta0Value(Bit#(32) theta0);
    method Action theta1Value(Bit#(32) theta1);
    method Bit#(32) result;
    method ActionValue#(TypeMainMemReq) memReq();
    method Action memResp(Bit#(64) resp);
endinterface

(* always_ready = "nValue, r0Value, r1Value, theta0Value, theta1Value" *)
(* synthesize *)
module mkCartPol#(Clock slowClock, Reset slowReset) (ICartPol);
    Reg#(Index)                      n <- mkRegU();
    Reg#(Bit#(DataWidth))            r <- mkRegU();
    Reg#(Bit#(TDataWidth))       theta <- mkRegU();   
    Reg#(CartPolState)           state <- mkReg(Idle);
    Reg#(ConState)                 con <- mkReg(GetPos);
    Reg#(Bit#(32))          cycleCount <- mkRegU;
    Reg#(Bit#(32))                addr <- mkRegU;
    Reg#(Bit#(16))               cartR <- mkRegU;
    Reg#(Bit#(16))               cartI <- mkRegU;
    Reg#(Index)                   rows <- mkRegU;    
    Reg#(Index)                   cols <- mkRegU;    

    FIFOF#(TypeMainMemReq)        reqQ <- mkFIFOF();
    FIFOF#(Bit#(64))             respQ <- mkFIFOF();

    `ifdef COMPUTE_1
    ComputeTop                computer <- mkComputeTop(slowClock,slowReset);
    `else
    Compute                   computer <- mkCompute(slowClock,slowReset);
    `endif

    rule countCycles(state == Working);
       cycleCount <= cycleCount + 1;
    endrule

    rule commence(state==Start);
        computer.setParam(unpack(r),unpack(theta),n);
        n <= n-1;
        rows <= 0;
        cols <= 0;
        cycleCount <= 0;
        state <= Working;
    endrule

    rule calcAddr(con==GetPos && state==Working);
        let pos <- computer.getPos();
        //Bit#(20) yx = pack(unsignedMul(unpack(pos.y), unpack(n))) + zeroExtend(pos.x);
        //Bit#(32) newAddr = {4'h9, 6'b000000, yx, 2'b0};
        Bit#(32) newAddr = {4'h9, 2'b0, 3'b0, pos.y, 1'b0, pos.x, 2'b0};
        reqQ.enq(TypeMainMemReq{rnw:1'b1, addr:newAddr, data:?, be:?});
        addr <= newAddr;
        con <= LdReq1;
    endrule

    rule get1(con==LdReq1 && state==Working);
        respQ.deq;
        Bit#(32) newAddr = addr + 32'h00000004;
        cartR <= addr[2]==1 ? respQ.first[63:48] : respQ.first[31:16];
        cartI <= addr[2]==1 ? respQ.first[47:32] : respQ.first[15:0];
        addr <= newAddr;
        reqQ.enq(TypeMainMemReq{rnw:1'b1, addr:newAddr, data:?, be:?});
        con <= LdReq2;
    endrule

    rule get2(con==LdReq2 && state==Working);
        respQ.deq;
        //Bit#(32) newAddr = addr + zeroExtend(n<<2);
        Bit#(32) newAddr = addr + 32'h00002000;
        cartR <= cartR + (addr[2]==1 ? respQ.first[63:48] : respQ.first[31:16]);
        cartI <= cartI + (addr[2]==1 ? respQ.first[47:32] : respQ.first[15:0]);
        addr <= newAddr;
        reqQ.enq(TypeMainMemReq{rnw:1'b1, addr:newAddr, data:?, be:?});
        con <= LdReq3;
    endrule

    rule get3(con==LdReq3 && state==Working);
        respQ.deq;
        Bit#(32) newAddr = addr - 32'h00000004;
        cartR <= cartR + (addr[2]==1 ? respQ.first[63:48] : respQ.first[31:16]);
        cartI <= cartI + (addr[2]==1 ? respQ.first[47:32] : respQ.first[15:0]);
        addr <= newAddr;
        reqQ.enq(TypeMainMemReq{rnw:1'b1, addr:newAddr, data:?, be:?});
        con <= LdReq4;
    endrule

    rule get4(con==LdReq4 && state==Working);
        respQ.deq;
        cartR <= cartR + (addr[2]==1 ? respQ.first[63:48] : respQ.first[31:16]);
        cartI <= cartI + (addr[2]==1 ? respQ.first[47:32] : respQ.first[15:0]);
        Bit#(20) yx = pack(unsignedMul(unpack(rows), unpack(n))) + zeroExtend(cols);
        //addr <= {4'h9, 6'b010000, yx, 2'b0};
        addr <= {4'h9, 2'b01, 3'b0, rows, 1'b0, cols, 2'b0};
        con <= StReq;
    endrule

    rule put(con==StReq && state==Working);
        Bit#(64) data = {cartR>>2, cartI>>2, cartR>>2, cartI>>2};
        Bit#(8) be = addr[2]==1'b1 ? 8'hf0 : 8'h0f;
        reqQ.enq(TypeMainMemReq{rnw:1'b0, addr:addr, data:data, be:be});
        rows <= (cols==n) ? rows+1 : rows;
        cols <= (cols==n) ? 0 : cols+1;
        state <= (cols==n && rows==n) ? Idle : state;
        con <= GetPos;
    endrule

    method Action nValue(Bit#(32) _n);
        n <= truncate(_n);
        state <= Start;
    endmethod

    method Action r0Value(Bit#(32) r0);
        r <= zeroExtend(r0);
    endmethod

    method Action r1Value(Bit#(32) r1);
        r <= {truncate(r1), r[31:0]};
    endmethod

    method Action theta0Value(Bit#(32) theta0);
        theta <= zeroExtend(theta0);
    endmethod

    method Action theta1Value(Bit#(32) theta1);
        theta <= {truncate(theta1), theta[31:0]};
    endmethod

    method Bit#(32) result if(state==Idle);
        return cycleCount;
    endmethod

    method ActionValue#(TypeMainMemReq) memReq();
        reqQ.deq;
        return reqQ.first;
    endmethod

    method Action memResp(Bit#(64) resp);
        respQ.enq(resp);
    endmethod
endmodule
