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
import Connectable::*;
import Vector::*;

// Local imports
`include "Common.bsv"

import PLBMasterWires::*;
import BRAMTargetWires::*;
import BRAMInitiatorWires::*;
import SimpleMemorySwitch::*;
import PLBMasterMagic::*;
import SimpleFUNetwork::*;
import SimpleController::*;
import SimpleFunctionalUnit::*;
import SimpleFeeder::*;
import FunctionalUnit::*;

/* This is the top level interface. We only have wires going to the 
   plb bus and the DSOCM which implements the BRAM interface. 
*/

module [Module] mkMultiplierFunctionalSimple (Multiplier);

  Feeder feeder <- mkSimpleFeeder();
  PLBMaster     plbMaster <- mkPLBMasterMagic();
  MemorySwitch  memorySwitch <- mkSimpleMemorySwitch();
  Vector#(FunctionalUnitNumber, FunctionalUnit#(1)) fus <- replicateM(mkSimpleFunctionalUnit);
  
  let fu_links = map(getLink, fus);
  
  FUNetwork     funet <- mkSimpleNetwork(fu_links);
  Controller    controller <- mkSimpleController();
  
  //Feeder pushs insts to Controller
  mkConnection(feeder.ppcInstructionOutput, controller.instructionInput);
  
  //
  mkConnection(plbMaster.wordInput, memorySwitch.plbMasterComplexWordOutput);
  mkConnection(plbMaster.wordOutput, memorySwitch.plbMasterComplexWordInput);

  mkConnection(controller.plbMasterCommandOutput, plbMaster.plbMasterCommandInput);
  mkConnection(controller.fuNetworkCommandOutput, funet.fuNetworkCommandInput);
  mkConnection(controller.memorySwitchCommandOutput, memorySwitch.memorySwitchCommandInput);
  
  for (Integer x = 0; x < valueof(FunctionalUnitNumber); x = x + 1)
  begin
    mkConnection(controller.functionalUnitCommandOutputs[x], fus[x].functionalUnitCommandInput);
    mkConnection(memorySwitch.functionalUnitComplexWordOutputs[x], fus[x].switchInput);
    mkConnection(fus[x].switchOutput, memorySwitch.functionalUnitComplexWordInputs[x]);
  end
  
  interface plbMasterWires = plbMaster.plbMasterWires;
  interface bramInitiatorWires = feeder.bramInitiatorWires;
endmodule


module [Module] mkMultiplierFunctional (Multiplier);

  Feeder feeder <- mkSimpleFeeder();
  PLBMaster     plbMaster <- mkPLBMasterMagic();
  MemorySwitch  memorySwitch <- mkSimpleMemorySwitch();
  Vector#(FunctionalUnitNumber, FunctionalUnit#(8)) fus <- replicateM(mkFunctionalUnit_STRIPPED8);
  
  let fu_links = map(getLink, fus);
  
  FUNetwork     funet <- mkSimpleNetwork(fu_links);
  Controller    controller <- mkSimpleController();
  
  //Feeder pushs insts to Controller
  mkConnection(feeder.ppcInstructionOutput, controller.instructionInput);
  
  //
  mkConnection(plbMaster.wordInput, memorySwitch.plbMasterComplexWordOutput);
  mkConnection(plbMaster.wordOutput, memorySwitch.plbMasterComplexWordInput);

  mkConnection(controller.plbMasterCommandOutput, plbMaster.plbMasterCommandInput);
  mkConnection(controller.fuNetworkCommandOutput, funet.fuNetworkCommandInput);
  mkConnection(controller.memorySwitchCommandOutput, memorySwitch.memorySwitchCommandInput);
  
  for (Integer x = 0; x < valueof(FunctionalUnitNumber); x = x + 1)
  begin
    mkConnection(controller.functionalUnitCommandOutputs[x], fus[x].functionalUnitCommandInput);
    mkConnection(memorySwitch.functionalUnitComplexWordOutputs[x], fus[x].switchInput);
    mkConnection(fus[x].switchOutput, memorySwitch.functionalUnitComplexWordInputs[x]);
  end
  
  interface plbMasterWires = plbMaster.plbMasterWires;
  interface bramInitiatorWires = feeder.bramInitiatorWires;
endmodule



//This wraps the multiplier wires to get an empty top-level interface for simulation.
//Timing models for the bus could be added here.

module [Module] mkMultiplierFunctionalSimulator();
   Multiplier mkMultiplierSimulator <- mkMultiplierFunctional();  
endmodule

module [Module] mkMultiplierFunctionalSimulatorSimple();
  Multiplier mkMultiplierSimulator <- mkMultiplierFunctionalSimple();
endmodule
