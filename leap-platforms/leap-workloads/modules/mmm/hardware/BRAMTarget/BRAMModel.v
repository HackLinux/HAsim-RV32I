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

module BRAMModel (
    CLK,
    RST_N,
    clk1,
    clk2,
    rst_n1,
    rst_n2,
    BRAM_RSTA, // not used
    BRAM_CLKA,
    BRAM_ENA,
    BRAM_WENA,
    BRAM_AddrA,
    BRAM_DinA, // I/O relavetive to BRAM 
    BRAM_DoutA,  
    BRAM_RSTB, // not used
    BRAM_CLKB,
    BRAM_ENB,
    BRAM_WENB,
    BRAM_AddrB,
    BRAM_DinB, // I/O relavetive to BRAM 
    BRAM_DoutB,  
    DUMMYA0,
    DUMMYA1,
    DUMMYA2,
    DUMMYA3,
    DUMMYA4,
    DUMMYA5,
    DUMMYB0,
    DUMMYB1,
    DUMMYB2,
    DUMMYB3,
    DUMMYB4,
    DUMMYB5   
  );

  parameter addr_width = 1;
  parameter addr_exp = 2;

  input CLK;
  input RST_N;
  input clk1;
  input clk2;
  input rst_n1;
  input rst_n2;
  input BRAM_RSTA; // not used
  input BRAM_CLKA;
  input BRAM_ENA;
  input [3:0]BRAM_WENA;
  input [addr_width - 1:0] BRAM_AddrA;
  output reg [31:0] BRAM_DinA; // I/O relavetive to BRAM 
  input [31:0] BRAM_DoutA;
 
  input BRAM_RSTB; // not used
  input BRAM_CLKB;
  input BRAM_ENB;
  input [3:0]BRAM_WENB;
  input [addr_width - 1:0] BRAM_AddrB;
  output reg[31:0] BRAM_DinB; // I/O relavetive to BRAM 
  input [31:0] BRAM_DoutB;

  input DUMMYA0;
  input DUMMYA1;
  input DUMMYA2;
  input DUMMYA3;
  input DUMMYA4;
  input DUMMYA5;

  input DUMMYB0;
  input DUMMYB1;
  input DUMMYB2;
  input DUMMYB3;
  input DUMMYB4;
  input DUMMYB5;  

  integer x;


  reg [31:0] mem[0:addr_exp-1];

  always @(posedge BRAM_CLKA) begin
    if (BRAM_ENA && BRAM_WENA[0]) begin
        mem[BRAM_AddrA] <= BRAM_DoutA;
    end
    BRAM_DinA <= mem[BRAM_AddrA];
  end

  always @(posedge BRAM_CLKB) begin
    if (BRAM_ENB && BRAM_WENB[0]) begin
        mem[BRAM_AddrB] <= BRAM_DoutB;
    end
    BRAM_DinB <= mem[BRAM_AddrB];
  end

  initial
    begin
      $display("Verilog: BRAM init begin %d %d", addr_width, addr_exp);
      for (x = 0; x < addr_exp; x = x + 1)
        begin
          mem[x] <= 0;
        end
      $display("Verilog: BRAM init done");
    end

endmodule

module BRAMModel128 (
    CLK,
    RST_N,
    clk1,
    clk2,
    rst_n1,
    rst_n2,
    BRAM_RSTA, // not used
    BRAM_CLKA,
    BRAM_ENA,
    BRAM_WENA,
    BRAM_AddrA,
    BRAM_DinA, // I/O relavetive to BRAM 
    BRAM_DoutA,  
    BRAM_RSTB, // not used
    BRAM_CLKB,
    BRAM_ENB,
    BRAM_WENB,
    BRAM_AddrB,
    BRAM_DinB, // I/O relavetive to BRAM 
    BRAM_DoutB,  
    DUMMYA0,
    DUMMYA1,
    DUMMYA2,
    DUMMYA3,
    DUMMYA4,
    DUMMYA5,
    DUMMYB0,
    DUMMYB1,
    DUMMYB2,
    DUMMYB3,
    DUMMYB4,
    DUMMYB5   
  );

  input CLK;
  input RST_N;
  input clk1;
  input clk2;
  input rst_n1;
  input rst_n2;
  input BRAM_RSTA; // not used
  input BRAM_CLKA;
  input BRAM_ENA;
  input [3:0]BRAM_WENA;
  input [6:0] BRAM_AddrA;
  output reg [31:0] BRAM_DinA; // I/O relavetive to BRAM 
  input [31:0] BRAM_DoutA;
 
  input BRAM_RSTB; // not used
  input BRAM_CLKB;
  input BRAM_ENB;
  input [3:0]BRAM_WENB;
  input [6:0] BRAM_AddrB;
  output reg[31:0] BRAM_DinB; // I/O relavetive to BRAM 
  input [31:0] BRAM_DoutB;

  input DUMMYA0;
  input DUMMYA1;
  input DUMMYA2;
  input DUMMYA3;
  input DUMMYA4;
  input DUMMYA5;

  input DUMMYB0;
  input DUMMYB1;
  input DUMMYB2;
  input DUMMYB3;
  input DUMMYB4;
  input DUMMYB5;  

  integer x;


  reg [31:0] mem[0:127];

  always @(posedge BRAM_CLKA) begin
    if (BRAM_ENA && BRAM_WENA[0]) begin
        mem[BRAM_AddrA] <= BRAM_DoutA;
    end
    BRAM_DinA <= mem[BRAM_AddrA];
  end

  always @(posedge BRAM_CLKB) begin
    if (BRAM_ENB && BRAM_WENB[0]) begin
        mem[BRAM_AddrB] <= BRAM_DoutB;
    end
    BRAM_DinB <= mem[BRAM_AddrB];
  end

  initial
    begin
      for (x = 0; x < 128; x = x + 1)
        begin
          mem[x] <= 0;
        end
      $display("Verilog: BRAM init done");
    end
endmodule

module BRAMModel8 (
    CLK,
    RST_N,
    clk1,
    clk2,
    rst_n1,
    rst_n2,
    BRAM_RSTA, // not used
    BRAM_CLKA,
    BRAM_ENA,
    BRAM_WENA,
    BRAM_AddrA,
    BRAM_DinA, // I/O relavetive to BRAM 
    BRAM_DoutA,  
    BRAM_RSTB, // not used
    BRAM_CLKB,
    BRAM_ENB,
    BRAM_WENB,
    BRAM_AddrB,
    BRAM_DinB, // I/O relavetive to BRAM 
    BRAM_DoutB,  
    DUMMYA0,
    DUMMYA1,
    DUMMYA2,
    DUMMYA3,
    DUMMYA4,
    DUMMYA5,
    DUMMYB0,
    DUMMYB1,
    DUMMYB2,
    DUMMYB3,
    DUMMYB4,
    DUMMYB5   
  );

  input CLK;
  input RST_N;
  input clk1;
  input clk2;
  input rst_n1;
  input rst_n2;
  input BRAM_RSTA; // not used
  input BRAM_CLKA;
  input BRAM_ENA;
  input [3:0]BRAM_WENA;
  input [2:0] BRAM_AddrA;
  output reg [31:0] BRAM_DinA; // I/O relavetive to BRAM 
  input [31:0] BRAM_DoutA;
 
  input BRAM_RSTB; // not used
  input BRAM_CLKB;
  input BRAM_ENB;
  input [3:0]BRAM_WENB;
  input [2:0] BRAM_AddrB;
  output reg[31:0] BRAM_DinB; // I/O relavetive to BRAM 
  input [31:0] BRAM_DoutB;

  input DUMMYA0;
  input DUMMYA1;
  input DUMMYA2;
  input DUMMYA3;
  input DUMMYA4;
  input DUMMYA5;

  input DUMMYB0;
  input DUMMYB1;
  input DUMMYB2;
  input DUMMYB3;
  input DUMMYB4;
  input DUMMYB5;  

  integer x;


  reg [31:0] mem[0:7];

  always @(posedge BRAM_CLKA) begin
    if (BRAM_ENA && BRAM_WENA[0]) begin
        mem[BRAM_AddrA] <= BRAM_DoutA;
    end
    BRAM_DinA <= mem[BRAM_AddrA];
  end

  always @(posedge BRAM_CLKB) begin
    if (BRAM_ENB && BRAM_WENB[0]) begin
        mem[BRAM_AddrB] <= BRAM_DoutB;
    end
    BRAM_DinB <= mem[BRAM_AddrB];
  end

  initial
    begin
      for (x = 0; x < 8; x = x + 1)
        begin
          mem[x] <= 0;
        end
      $display("Verilog: BRAM init done");
    end

endmodule