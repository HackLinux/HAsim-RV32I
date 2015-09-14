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

Author: Myron King, Nirav Dave
*/


// Global imports
import Clocks::*;

 
// Local imports
import Parameters::*;
import Types::*;
import Interfaces::*;
import Multiplier::*;
import PLBMasterWires::*;
import BRAMTargetWires::*;
import PPCModel::*;
import PLBModel::*; 
import BRAMModel::*; 

module mkSystemSimulator();
  Clock fpga_clock <- exposeCurrentClock();
  Reset fpga_reset <- exposeCurrentReset();

  Multiplier  multiplier <- mkMultiplierTop;                                      

  PPC ppc <- mkPPCModel();
  
  Empty bram_between_ppc_and_feeder <- 
      mkBRAMModel(fpga_clock, fpga_reset,fpga_clock, fpga_reset, ppc.bramInitiatorWires, multiplier.bramInitiatorWires);



  Empty  plb  <- mkPLBModel(multiplier.plbMasterWires);                              


endmodule
