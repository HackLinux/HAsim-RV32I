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

// Simple state for the put side.  We are either polling, or we are capable of loading data, or we are setting the flag
// One hot the states
`define SETUP_POLLING   2'b00
`define POLLING         2'b01
`define DATA            2'b10
`define SET_FLAG        2'b11

 //2'b00 - prefetch, 2'b01 - reading, 2'b10 - write mask done, read mask wait one cycle, 2'b11 - read mask spin
`define PREFETCH 3'b000
`define READING_POST_PREFETCH 3'b001
`define READING  3'b010
`define WRITE_MASK_DONE 3'b011
`define WRITE_MASK_SPIN 3'b100

//1 is for producer. 2 is for consumer
//WARNING - cant nack and request data in the same cycle for the consumer.

module mcd_bram_interface
(
    CLK,
    CLK_GATE,
    clk1,
    clk2,
    rst_n1,
    rst_n2,

    put1_en,
    put1_rdy,
    put1_data,
    put1_nack_en,

    get2_en,
    get2_rdy,
    get2_data,

    get2_ack_en,
    get2_nack_en
);


    parameter burst_size = 16;
    parameter log_burst_size = 4;
    parameter data_size = 64;
    parameter log_data_size = 6;

    parameter bram_size = 128;
    parameter log_bram_size = 7;
    parameter num_bursts = 64/burst_size;
    parameter log_num_bursts = 6 - log_burst_size;

    input         clk1;  //get-side
    input         clk2;  //put-side

    input         CLK;
    input         CLK_GATE;

    input         rst_n1;
    input         rst_n2;

    input         put1_en;
    output        put1_rdy;
    input [31:0]  put1_data;
    input         put1_nack_en;

    input         get2_en;
    output        get2_rdy;
    output [31:0] get2_data;
    input         get2_ack_en;
    input         get2_nack_en;


    // State
    reg[log_num_bursts-1:0] burst_number;
    reg[log_burst_size:0]   word_in_burst;
    reg[1:0]                state;
   
    //Wires named reg
    reg[log_burst_size:0]    word_in_burst_next;
    reg[1:0]                 state_next;
    wire[6:0]  addr1; // The bram address

    // Wires
    wire[log_num_bursts-1:0] burst_number_next;
    wire                     write_en1;   
    wire[31:0]               write_data1;
    wire[31:0]               read_data1;
    wire                     en1;
 
    //Wire assignments
    assign en1 = 1;

    assign put1_rdy = (state == `DATA);

    assign write_data1[0] = (state == `SET_FLAG)? 1'b1:put1_data[0];

    assign write_data1[31:1] = put1_data[31:1];
    
    assign write_en1 = (put1_en && put1_rdy) || (state == `SET_FLAG); 

    assign addr1 = {word_in_burst,burst_number};

    always@(*)
      begin
        if(state == `SET_FLAG)
          begin
            state_next = `SETUP_POLLING; 
          end
        else if(state == `SETUP_POLLING)
          begin
            state_next = `POLLING;
          end
        else if((read_data1[0] == 0) && (state == `POLLING))
          begin
            state_next = `DATA;
          end
        else if((state == `DATA) && word_in_burst_next[log_burst_size])  
          begin
            state_next = `SET_FLAG;
          end      
        else
          begin
            state_next = state;
          end
      end

    always@(*)
      begin
        if((put1_nack_en && (state == `DATA)) || ((state == `POLLING) && (read_data1[0] == 0)))
          begin
            word_in_burst_next = 0;
          end
        else if(put1_en)
          begin
            word_in_burst_next = word_in_burst + 1;
          end
        else
          begin
            word_in_burst_next = word_in_burst;
          end 
      end
  
    `ifdef DEBUG
    always@(word_in_burst_next)
      begin
        $display("Word in burst next: %d critical bit: %d, bit: %d ", word_in_burst_next, word_in_burst_next[log_burst_size], log_burst_size); 
      end
    `endif
 
   
    assign burst_number_next = (state == `SET_FLAG)? burst_number + 1: burst_number;

    `ifdef DEBUG
    always@(*)
      if(state == `SET_FLAG)
        $display($time, "   MCDBRAM: Put Side State: SET_FLAG addr:%x data:%x", addr1, write_data1);  
      else if(state == `POLLING)
        $display($time, "   MCDBRAM: Put Side State: POLLING addr:%x data:%d",addr1,read_data1); 
      else if(state == `DATA)
        $display($time, "   MCDBRAM: Put Side State: DATA, word_in_burst: %d", word_in_burst);     
    //Register assignment
    `endif

    always@(posedge clk1)
      begin
        if(~rst_n1)
          begin
            burst_number <= 0;
            word_in_burst <=  1 << log_burst_size;
            state <= `SETUP_POLLING;        
          end
        else
          begin
            state <= state_next;
            burst_number <= burst_number_next;
            word_in_burst <= word_in_burst_next;
          end
      end

    wire en2;
    wire [6:0] addr2; //wire
    wire write_en2;
    wire [31:0] write_data2;
    wire [31:0] read_data2;

    reg [31:0] prefetch_data;


    reg [log_burst_size:0] data2_count;
    reg [log_burst_size:0] data2_count_observed; // the signal;
    reg [log_burst_size:0] data2_read;    
    wire[log_burst_size:0] data2_count_less_one;
    wire[log_burst_size:0] data2_read_plus_one;   
    reg [log_num_bursts-1:0] data2_bursts;
    wire is_last_addr = data2_count_observed[log_burst_size];
    wire read_last_addr_next = data2_read_plus_one[log_burst_size];
    wire read_last_addr = data2_read[log_burst_size];
    reg [2:0] state2; //2'b00 - prefetch, 2'b01 - reading, 2'b10 - write mask done, read mask wait one cycle, 2'b11 - read mask spin
    reg [2:0] state2_last;
    reg       rst_n2_last;
    reg       get2_en_last;
    reg       is_last_addr_last; 
    reg       get2_nack_en_last;        
    reg       read_data2_0_last; 


    assign data2_count_less_one = data2_count_observed - 1;
    assign data2_read_plus_one = data2_read + 1;   

    always@(posedge clk2)
    begin 
        if(state2 == `READING_POST_PREFETCH)
          begin
            prefetch_data <= read_data2;
          end
        else if((state2 == `READING) && get2_en)
          begin
            prefetch_data <= read_data2;
          end
        else 
          begin 
            prefetch_data <= prefetch_data;
          end 
    end

    assign get2_data = prefetch_data;


    always@(posedge clk2)
    begin
        if(!rst_n2)
        begin
            state2 <= `WRITE_MASK_SPIN;
        end
        else if(state2 == `PREFETCH)
        begin
            state2 <= `READING_POST_PREFETCH;
        end
        else if(state2 == `READING_POST_PREFETCH) // This state just latches the first read value in the prefetch reg
        begin
            state2 <= `READING;
        end

        else if(state2 == `READING && get2_nack_en) // on a nack, we need to swithc to the prefetch state to reload data.
        begin
            state2 <= `PREFETCH;
        end        
        else if(state2 == `READING && get2_ack_en && (read_last_addr || (read_last_addr_next && get2_en)))
        begin
            state2 <= `WRITE_MASK_DONE;
        end
        else if(state2 == `WRITE_MASK_DONE)
        begin
            state2 <= `WRITE_MASK_SPIN;
        end
        else if(state2 == `WRITE_MASK_SPIN && read_data2[0])
        begin
            state2 <= `PREFETCH;
        end
    end

    
    always@(posedge clk2)
      begin
        if(!rst_n2)
          begin
            data2_read <= 0;
          end
        else if(state2 == `READING && get2_en)
          begin
            data2_read <= data2_read_plus_one;
          end 
        else if(state2 == `READING)
          begin
            data2_read <= data2_read;
          end
        else
          begin
            data2_read <= 0;
          end
      end

    always@(posedge clk2)
    begin
        if(!rst_n2)
          begin
            data2_bursts <= 0;
          end
        else if(state2 == `READING && get2_ack_en && is_last_addr && !get2_nack_en)
          begin
            data2_bursts <= data2_bursts+1;
          end   
    end     

    always@(posedge clk2)
      begin
        if(!rst_n2)
          begin
            data2_count <= {1'b1, {(log_burst_size){1'b0}}}; 
          end
        else
          begin
            data2_count <= data2_count_observed;
          end       
      end
    
    always@(posedge clk2)
      begin
        state2_last <= state2;
      end    

    always@(posedge clk2)
      begin    
        rst_n2_last <= rst_n2;
      end  
   
    always@(posedge clk2)
       begin    
        get2_en_last <= get2_en;
      end        

    always@(posedge clk2)
       begin    
        is_last_addr_last <= is_last_addr;
      end 

    always@(posedge clk2)
       begin    
        get2_nack_en_last <= get2_nack_en;
      end 

    always@(posedge clk2)
       begin    
         read_data2_0_last <= read_data2[0];
       end

    always@(*)
      begin
        if(!rst_n2_last)
          begin
            data2_count_observed  = {1'b1, {(log_burst_size){1'b0}}};
          end
        else if(state2_last == `READING && get2_nack_en_last)
          begin
            data2_count_observed = 0;
          end
        else if(get2_en_last || (state2_last == `READING_POST_PREFETCH) || (state2_last == `PREFETCH))
          begin 
            data2_count_observed = data2_count + ((is_last_addr_last)?0:1);  // If the last addr is high then we add nothing
          end
        else if(state2_last == `WRITE_MASK_SPIN  && read_data2_0_last)
          begin
            data2_count_observed = 0;
          end
        else
          begin
            data2_count_observed = data2_count;
          end
    end

    `ifdef DEBUG
     always@(*)
      if(state2 == `PREFETCH)
        $display($time, "  MCDBRAM: Get Side State: PREFETCH %b %b %b %b %b %b %b %b %b", data2_count, data2_bursts, write_en2, read_data2, get2_ack_en, get2_en, is_last_addr, get2_nack_en, get2_rdy);  
      else if(state2 == `READING)
        $display($time, "  MCDBRAM: Get Side State: READING %b %b %b %b %b %b %b %b %b", data2_count, data2_bursts, write_en2, read_data2, get2_ack_en, get2_en, is_last_addr, get2_nack_en, get2_rdy); 
      else if(state2 == `WRITE_MASK_DONE)
        $display($time, "  MCDBRAM: Get Side State: WRITE_MASK_DONE %b %b %b %b %b %b %b %b %b", data2_count, data2_bursts, write_en2, read_data2, get2_ack_en, get2_en, is_last_addr, get2_nack_en, get2_rdy);
      else if(state2 == `WRITE_MASK_SPIN)
        $display($time, "  MCDBRAM: Get Side State: WRITE_MASK_SPIN %b %b %b %b %b %b %b %b %b", data2_count, data2_bursts, write_en2, bram.read_data2, get2_ack_en, get2_en, is_last_addr, get2_nack_en, get2_rdy);  
    `endif

    assign en2 = 1'b1;

    // Basically, if we are reading and we didn't get an enable, we'll drop data   
    // so, we should just use the last address.
    assign addr2 = (state2 == `READING && !get2_en && !is_last_addr)?{data2_count_less_one, data2_bursts}:{data2_count_observed, data2_bursts};

    assign write_en2 = state2 == `READING && get2_ack_en && is_last_addr; 
    assign write_data2 = 32'h00000000;

    assign get2_rdy = state2 == `READING && !read_last_addr;


    BRAMMCD128 bmc(    
      .BRAM_RSTA(~rst_n1), // not used
      .BRAM_CLKA(clk1),
      .BRAM_ENA(en1),
      .BRAM_WENA({write_en1,write_en1,write_en1,write_en1}),
      .BRAM_AddrA(addr1),
      .BRAM_DinA(read_data1), // I/O relavetive to BRAM 
      .BRAM_DoutA(write_data1),  
      .BRAM_RSTB(~rst_n2), // not used
      .BRAM_CLKB(clk2),
      .BRAM_ENB(en2),
      .BRAM_WENB({write_en2,write_en2,write_en2,write_en2}),
      .BRAM_AddrB(addr2),
      .BRAM_DinB(read_data2), // I/O relavetive to BRAM 
      .BRAM_DoutB( write_data2)  
    );

endmodule
 

module BRAMMCD128 (
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
  );
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
