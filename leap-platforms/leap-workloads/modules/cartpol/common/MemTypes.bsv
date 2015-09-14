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


import ClientServer::*;

//----------------------------------------------------------------------
// Some test related values
//---------------------------------------------------------------------
typedef 4 NumTests;

//----------------------------------------------------------------------
// Basic memory requests and responses
//----------------------------------------------------------------------

typedef union tagged
{
  struct { Bit#(addrSz) addr; Bit#(tagSz) tag; } LoadReq;
}
MemReq#( type addrSz, type tagSz, type dataSz ) 
deriving(Eq,Bits);

typedef union tagged
{
  struct { Bit#(tagSz) tag; data_t data; } LoadResp;
}
MemResp#( type tagSz, type data_t )
deriving(Eq,Bits);

//----------------------------------------------------------------------
// Specialized req/resp for inst/data/host
//----------------------------------------------------------------------

typedef 32 CacheWordSize;
typedef Bit#(CacheWordSize) CacheWord;
typedef TDiv#(CacheWordSize,8) BytesPerCacheWord;
typedef 22 AddrSz;
typedef 0 TagSz;
typedef CacheWordSize DataSz;

typedef MemReq#(AddrSz,TagSz,DataSz)     DataReq;
typedef MemResp#(TagSz,Bit#(DataSz))           DataResp;


//----------------------------------------------------------------------
// Function for extracting the data from the double
//----------------------------------------------------------------------
typedef struct {
  Bit#(DataSz) lo;
  Bit#(DataSz) hi;
} DoubleWord deriving (Bits,Eq);

typedef MemResp#(TagSz,DoubleWord)           DoublePumpDataResp;


//----------------------------------------------------------------------
// Specialized req/resp for main memory
//----------------------------------------------------------------------

typedef 22 MainMemAddrSz;
typedef 01 MainMemTagSz;
typedef BusWordSize MainMemDataSz;

typedef MemReq#(MainMemAddrSz,MainMemTagSz,MainMemDataSz) MainMemReq;
typedef MemResp#(MainMemTagSz,Bit#(MainMemDataSz))       MainMemResp;

// Types for swap buffer
typedef TMul#(1024,TMul#(1024,4)) BufferSz;

// Cache types

typedef BitsPerBurst CacheLineSz;
typedef TLog#(BytesPerBurst)  CacheLineBlockSz; // 7
typedef TSub#(CacheLineBlockSz, TLog#(BytesPerCacheWord)) CacheLineBlockIndexSz; //5
typedef 9   CacheLineIndexSz; // Size of cache
typedef TSub#(TSub#(AddrSz,CacheLineBlockSz),CacheLineIndexSz)   CacheLineTagSz;

typedef Bit#(CacheLineIndexSz)      CacheLineIndex;
typedef Bit#(TAdd#(CacheLineIndexSz,TLog#(TDiv#(CacheLineSz,SizeOf#(BusWord)))))  SkinnyCacheIndex;
typedef Bit#(CacheLineTagSz)        CacheLineTag;
typedef Bit#(CacheLineBlockSz)      CacheLineBlock;
typedef Bit#(CacheLineBlockIndexSz) CacheLineBlockIndex;
typedef Bit#(CacheLineSz)           CacheLine;

interface CacheWrapper;
   interface Server#(Coord,Bit#(32)) mmem_server;
   interface Client#(MainMemReq,MainMemResp) mmem_client;
   method Action set_n(Bit#(AddrSz) n);
   method Action reset();
endinterface

typedef struct {
   Bit#(10) x;
   Bit#(10) y;
   } Coord deriving (Eq,Bits,Bounded);


