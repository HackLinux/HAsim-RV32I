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

`include "awb/provides/librl_bsv.bsh"
`include "awb/provides/soft_connections.bsh"
`include "awb/provides/soft_services.bsh"
`include "awb/provides/soft_services_lib.bsh"
`include "awb/provides/soft_services_deps.bsh"
`include "awb/provides/mmm_controller.bsh"
`include "awb/provides/mmm_common.bsh"
`include "awb/provides/mmm_functional_unit.bsh"
`include "awb/provides/mmm_memory_switch.bsh"
`include "awb/provides/mmm_functional_unit_network.bsh"
`include "awb/provides/mmm_memory_unit.bsh"

module [CONNECTED_MODULE] mkConnectedApplication ();
   

  PLBMaster     plbMaster <- mkPLBMasterMagic();

  MemorySwitch  memorySwitch <- mkSimpleMemorySwitch();

  Vector#(FunctionalUnitNumber, FunctionalUnit) fus <- replicateM(mkFunctionalUnit);
  


  let fu_links = map(getLink, fus);
  
  FUNetwork     funet <- mkSimpleNetwork(fu_links);
  Controller    controller <- mkSimpleController();


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
  
endmodule
