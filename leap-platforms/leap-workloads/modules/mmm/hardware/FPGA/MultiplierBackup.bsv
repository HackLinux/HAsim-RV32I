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

Author: Nirav Dave
*/

// Global imports
import Connectable::*;
import Vector::*;

// Local imports
import Types::*;
import Interfaces::*;
import Parameters::*;
import Utils::*;

import PLBMasterWires::*;
import BRAMTargetWires::*;
import BRAMInitiatorWires::*;
import SimpleMemorySwitch::*;
import PLBMasterBackup::*;
import SimpleFUNetwork::*;
import SimpleController::*;
import FunctionalUnit::*;
import BRAMFeeder::*;
import BRAMModel::*;


/* This is the top level interface. We only have wires going to the 
   plb bus and the DSOCM which implements the BRAM interface. 
*/
(*synthesize*)
module mkMultiplierBackupTop (MultiplierBackup);
  Clock fpgaClock <- exposeCurrentClock();
  Reset fpgaReset <- exposeCurrentReset();
 
  //Feeder feeder <- mkPPCFeeder();
  Feeder feeder <- mkBRAMFeeder();

  PLBMasterBackup     plbMaster <- mkPLBMasterBackup();
  MemorySwitch  memorySwitch <- mkSimpleMemorySwitch();
  Vector#(FunctionalUnitNumber, FunctionalUnit#(8)) fus <- replicateM(mkFunctionalUnit_STRIPPED8);
  


  let fu_links = map(getLink, fus);
  
  FUNetwork     funet <- mkSimpleNetwork(fu_links);
  Controller    controller <- mkSimpleController();

  mkConnection(feeder.ppcInstructionOutput, controller.instructionInput);
  mkConnection(plbMaster.wordInput, memorySwitch.plbMasterComplexWordOutput);
  mkConnection(plbMaster.wordOutput, memorySwitch.plbMasterComplexWordInput);
  mkConnection(controller.plbMasterCommandOutput, plbMaster.plbMasterCommandInput);
  mkConnection(controller.fuNetworkCommandOutput, funet.fuNetworkCommandInput);
  mkConnection(controller.memorySwitchCommandOutput, memorySwitch.memorySwitchCommandInput);

  mkConnection(memorySwitch.getInstructionOutput, feeder.ppcMessageInput);
  
  for (Integer x = 0; x < valueof(FunctionalUnitNumber); x = x + 1)
  begin
    mkConnection(controller.functionalUnitCommandOutputs[x], fus[x].functionalUnitCommandInput);
    mkConnection(memorySwitch.functionalUnitComplexWordOutputs[x], fus[x].switchInput);
    mkConnection(fus[x].switchOutput, memorySwitch.functionalUnitComplexWordInputs[x]);
  end
  
  interface plbBRAMWires = plbMaster.plbBRAMWires;
  //interface bramTargetWires = feeder.bramTargetWires;
  interface bramInitiatorWires = feeder.bramInitiatorWires;  

endmodule
