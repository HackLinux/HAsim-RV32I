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
import Types::*;
import Interfaces::*;
import MultiplierBackup::*;
import PLBMasterWires::*;
import BRAMTargetWires::*;
import PPCModel::*;
import PLBMaster_backupPPC::*; 
import BRAMModel::*; 
import BRAMInitiatorWires::*;
import PLBMasterWires::*;

module mkSystemSimulatorBackup();
  Clock fpgaClock <- exposeCurrentClock;
  Reset fpgaReset <- exposeCurrentReset;
  
  Clock plbClock <- mkAbsoluteClock(3, 3);
  Reset plbReset <- mkInitialReset(1, clocked_by plbClock);   

  MultiplierBackup  multiplier <- mkMultiplierBackupTop;

  PPC ppc <- mkPPCModel();
  BRAMInitiatorWires#(Bit#(14)) plb  <- mkPLB_backupPPC();  

  Empty bram_between_ppc_and_feeder <- 
      mkBRAMModel(fpgaClock, fpgaReset, fpgaClock, fpgaReset, ppc.bramInitiatorWires, multiplier.bramInitiatorWires);

  Empty bram_between_ppc_and_plb <- 
      mkBRAMModel(fpgaClock, fpgaReset, fpgaClock, fpgaReset, plb, multiplier.plbBRAMWires);



endmodule
