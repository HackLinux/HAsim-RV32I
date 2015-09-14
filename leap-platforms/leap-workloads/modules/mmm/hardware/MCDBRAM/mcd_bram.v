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

//2 is blusepec
module mcd_bram
(
    clk1,
    clk2,
    rst_n1,
    rst_n2,

    en1,
    addr1,
    write_en1,
    write_data1,
    read_data1,

    en2,
    addr2,
    write_en2,
    write_data2,
    read_data2
);
    parameter addr_width = 1;
    parameter data_width = 1;
    parameter lo = 0;
    parameter hi = 1;

    input clk1;
    input clk2;
    input rst_n1;
    input rst_n2;

    input en1;
    input write_en1;
    input  [addr_width-1:0] addr1;
    input  [data_width-1:0] write_data1;
    output reg [data_width-1:0] read_data1;

    input en2;
    input write_en2;
    input  [addr_width-1:0] addr2;
    input  [data_width-1:0] write_data2;
    output reg [data_width-1:0] read_data2;

    //synthesis attribute ram_extract of RAM is yes;
    //synthesis attribute ram_style of RAM is block;
    reg [data_width-1:0] ram[hi:lo];
 
    integer x,y; 

    always@(posedge clk1)
    begin
        if(en1)
        begin
            if(write_en1)
                ram[addr1] <= write_data1;
            if(!rst_n1)
              begin
                // synopsys translate_off
                $display("Resetting BRAM");
                for (x = lo; x < hi; x = x + 1)
                  begin
                    ram[x] <= 0;
                  end
                // synopsys translate_on
                read_data1 <= 0;
              end
            else
              begin
                read_data1 <= ram[addr1];
              end
        end
    end

    always@(posedge clk2)
    begin
        if(en2)
        begin
            if(write_en2)
                ram[addr2] <= write_data2;
            if(!rst_n2)
              begin 
                // synopsys translate_off
                for (y = lo; y < hi; y = y + 1)
                  begin
                    ram[y] <= 0;
                  end
                // synopsys translate_on
                read_data2 <= 0;
              end
            else
                read_data2 <= ram[addr2];

        end
    end
endmodule
