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

*/

`include "awb/provides/librl_bsv.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/soft_services.bsh"
`include "awb/provides/soft_services_lib.bsh"
`include "awb/provides/soft_services_deps.bsh"
`include "awb/provides/cartpol_common.bsh"
`include "awb/provides/cartpol_cordic.bsh"


import FIFOF::*;
import RWire::*;
import GetPut::*;

// Convert a FIFOF into a put interface

function Put#(item_t) fifofToPut( FIFOF#(item_t) f ) provisos ( );
  return
  (
    interface Put
      method Action put( item_t item );
        f.enq(item);
      endmethod
    endinterface
  );
endfunction

// Convert a FIFOF into a get interface

function Get#(item_t) fifofToGet( FIFOF#(item_t) f ) provisos ( );
  return
  (
    interface Get
      method ActionValue#(item_t) get();
        f.deq();
        return f.first();
      endmethod
    endinterface
   );
endfunction

// Convert a register into an (always ready) put interface

function Put#(item_t) regToPut( Reg#(item_t) r ) provisos ( );
  return
  (
    interface Put
      method Action put( item_t item );
        r <= item;
      endmethod
    endinterface
  );
endfunction

// Convert a register into an (always ready) get interface

function Get#(item_t) regToGet( Reg#(item_t) r ) provisos ( );
  return
  (
    interface Get
      method ActionValue#(item_t) get();
        return r;
      endmethod
    endinterface
   );
endfunction

// Convert a Wire into a put interface

function Put#(item_t) wireToPut( Wire#(item_t) w ) provisos ( );
  return
  (
    interface Put
      method Action put( item_t item );
        w._write(item);
      endmethod
    endinterface
  );
endfunction

// Convert a WIREF into a get interface

function Get#(item_t) wireToGet( Wire#(item_t) w ) provisos ( );
  return
  (
    interface Get
      method ActionValue#(item_t) get();
        return w._read();
      endmethod
    endinterface
   );
endfunction

// Convert a RWire into a put interface

function Put#(item_t) rwireToPut( RWire#(item_t) w ) provisos ( );
  return
  (
    interface Put
      method Action put( item_t item );
        w.wset(item);
      endmethod
    endinterface
  );
endfunction

// Convert a RWire into a get interface

function Get#(item_t) rwireToGet( RWire#(item_t) w ) provisos ( );
  return
  (
    interface Get
      method ActionValue#(item_t) get() if ( isValid(w.wget()) );
        return unJust(w.wget());
      endmethod
    endinterface
   );
endfunction

