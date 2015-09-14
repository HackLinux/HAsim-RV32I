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

Author: Muralidaran Vijayraghavan, Kermin Fleming
*/

import GetPut::*;
import BRAMInitiatorWires::*;

typedef Bit#(32) Data;

interface MCDBRAMGet#(numeric type burst_size, numeric type data_size);
    method ActionValue#(Data) get();
    method Action ack();
    method Action nack();
endinterface

interface MCDBRAMPut#(numeric type burst_size);
    method Action put(Data data);
    method Action nack();
endinterface

interface MCDBRAMFlat#(numeric type burst_size);
    method ActionValue#(Data) get();
    method Action get_ack();
    method Action get_nack();
    method Data   get_peek();
    method Bool   get_valid();

    method Action put(Data data);
    method Action put_nack();
    method Bool   put_valid();
endinterface

interface MCDBRAM#(numeric type burst_size);
    method Action get_ack();
    method Action get_nack(); 
    method Data   get_peek();
    method Bool   get_valid();
    method Action put_nack();
    method Bool   put_valid();
    interface Get#(Bit#(32)) get;
    interface Put#(Bit#(32)) put;
endinterface

import "BVI" mcd_bram_interface =
module mkMCDBRAMFlat#(Clock clk1, Clock clk2, Reset rst1, Reset rst2)(MCDBRAMFlat#(burst_sz));
   
    no_reset;

    parameter burst_size = valueOf(burst_sz);
    parameter data_size  = 64;
    parameter log_burst_size = valueOf(TLog#(burst_sz));
    parameter log_data_size  = 6;
    parameter bram_size = 7;
    parameter log_bram_size = 128;
    parameter num_bursts = valueOf(TDiv#(64, burst_sz));
    parameter log_num_bursts = valueOf(TSub#(TLog#(64), TLog#(burst_sz)));

    input_clock clk1(clk1) = clk1 ; // put clock
    input_clock clk2(clk2) = clk2 ; // get clock

    input_reset rst1(rst_n1) = rst1;
    input_reset rst2(rst_n2) = rst2;

    method get2_data get() enable(get2_en) ready(get2_rdy) clocked_by(clk2);
    method get_ack() enable(get2_ack_en)  clocked_by(clk2);
    method get_nack() enable(get2_nack_en) clocked_by(clk2);
    method get2_data get_peek() clocked_by(clk2); // ready(get2_rdy); 
    method get2_rdy get_valid() clocked_by(clk2);
 
    method put(put1_data) enable(put1_en) ready(put1_rdy) clocked_by(clk1);
    method put_nack() enable(put1_nack_en)  clocked_by(clk1);
    method put1_rdy put_valid() clocked_by(clk1);


    schedule get       CF (get_ack, get_valid, get_nack, put, put_nack, put_valid);
    schedule get_ack   CF (get, get_valid, get_peek, put, put_nack, put_valid);
    schedule get_nack  CF (get, get_valid, get_peek, put, put_nack, put_valid);
    schedule get_peek  CF (get, get_valid, get_peek, get_ack, get_nack, put, put_nack, put_valid);
    schedule get_valid CF (get, get_valid, get_peek, get_ack, get_nack, put, put_nack, put_valid);
    schedule put       CF (get, get_valid, get_peek, get_ack, get_nack, put_nack, put_valid);
    schedule put_nack  CF (get, get_valid, get_peek, get_ack, get_nack, put, put_valid);
    schedule put_valid CF (get, get_valid, get_peek, get_ack, get_nack, put, put_nack, put_valid);    

endmodule

module mkMCDBRAM#(Clock clk1, Reset rst1, Clock clk2, Reset rst2)(MCDBRAM#(burst_sz));
    MCDBRAMFlat#(burst_sz) basic <- mkMCDBRAMFlat(clk1, clk2, rst1, rst2);
    
    method get_ack = basic.get_ack;
    method get_nack = basic.get_nack;
    method get_peek = basic.get_peek; 
    method get_valid = basic.get_valid;
    method put_nack = basic.put_nack;
    method put_valid = basic.put_valid;  

    interface Get get;
        method get = basic.get;
    endinterface
 
    interface Put put;
        method put = basic.put;
    endinterface

endmodule
