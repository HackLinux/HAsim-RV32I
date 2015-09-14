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

module bram_fu
(
    CLK,
    CLK_GATE,
    RST,
    write_en,
    write_addr,
    write_data,
    read_en,
    read_addr,
    read_data
);
    parameter addr_width = 9;
    parameter data_width = 1024;
    parameter lo = 0;
    parameter hi = 511;

    input CLK;
    input CLK_GATE;
    input RST;
    input write_en;
    input  [addr_width-1:0] write_addr;
    input  [data_width-1:0] write_data;
    input read_en;
    input  [addr_width-1:0] read_addr;
    output reg [data_width-1:0] read_data;

    wire en = write_en | read_en;
    
    //synthesis attribute ram_extract of RAM is yes;
    //synthesis attribute ram_style of RAM is block;
    reg [data_width-1:0] ram[hi:lo];

    integer x;

    always@(posedge CLK)
    begin
        if(RST)
        begin
            // synopsys translate_off
            for (x = lo; x <= hi; x = x + 1)
            begin
                ram[x] <= 0;
            end
            // synopsys translate_on
            read_data <= 0;
        end
        else
        if(en)
        begin
            read_data <= ram[read_addr];
            if(write_en)
                ram[write_addr] <= write_data;
        end
    end
endmodule
