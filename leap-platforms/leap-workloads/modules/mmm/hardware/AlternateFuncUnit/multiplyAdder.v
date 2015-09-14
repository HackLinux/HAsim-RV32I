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

module multiplyAdder
(
    CLK,
    CLK_GATE,
    x,
    y,
    accum,
    is_load,
    mulAdd,
    enable
);

    input CLK;
    input CLK_GATE;
    input [31:0] x;
    input [31:0] y;
    input  is_load;
    input [31:0] accum;
    output [31:0] mulAdd;

    input enable;

    wire [15:0] a = x[31:16];
    wire [15:0] b = x[15:0];
    wire [15:0] c = y[31:16];
    wire [15:0] d = y[15:0];

    reg [15:0] ac;
    reg [15:0] bd;
    reg [15:0] ad;
    reg [15:0] bc;

    wire [15:0] ac_wire;
    wire [15:0] bd_wire;
    wire [15:0] ad_wire;
    wire [15:0] bc_wire;

    /*
    mult mul1(.mult_fp_1(a), .mult_fp_2(c), .mult_fp(ac_wire));
    mult mul2(.mult_fp_1(b), .mult_fp_2(d), .mult_fp(bd_wire));
    mult mul3(.mult_fp_1(a), .mult_fp_2(d), .mult_fp(ad_wire));
    mult mul4(.mult_fp_1(b), .mult_fp_2(c), .mult_fp(bc_wire));
    */

    always@(posedge CLK)
    begin
        if(enable)
        begin
            ac <= ac_wire;
            bd <= bd_wire;
            ad <= ad_wire;
            bc <= bc_wire;
        end
    end

    reg [31:0] accum_reg1;

    always@(posedge CLK)
    begin
        if(enable)
        begin
            if(is_load)
                accum_reg1 <= accum;
            else
                accum_reg1 <= mulAdd;
        end
    end

    wire [15:0] r_part = ac-bd;
    wire [15:0] i_part = ad+bc;

    wire [15:0] real_part = r_part+accum_reg1[31:16];
    wire [15:0] imag_part = i_part+accum_reg1[15:0];

    assign mulAdd = {real_part, imag_part};

endmodule
