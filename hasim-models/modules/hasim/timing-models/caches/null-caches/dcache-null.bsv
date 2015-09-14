import FIFO::*;
import Vector::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/module_local_controller.bsh"

`include "asim/provides/hasim_isa.bsh"
`include "asim/provides/fpga_components.bsh"
`include "asim/provides/funcp_interface.bsh"

`include "asim/provides/chip_base_types.bsh"
`include "asim/provides/memory_base_types.bsh"

module [HASIM_MODULE] mkDCache();


    // ****** Ports ******

    // Incoming port from CPU with speculative stores
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_LOAD_INPUT) loadReqFromCPU <- mkPortRecv_Multiplexed("CPU_to_DCache_load", 0);

    // Incoming port from CPU with committed stores
    PORT_RECV_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_STORE_INPUT) storeReqFromCPU <- mkPortRecv_Multiplexed("CPU_to_DCache_store", 0);

    // Outgoing port to CPU with speculative immediate response
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_LOAD_OUTPUT_IMMEDIATE) loadRspImmToCPU <- mkPortSend_Multiplexed("DCache_to_CPU_load_immediate");

    // Outgoing port to CPU with speculative delayed response
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_LOAD_OUTPUT_DELAYED) loadRspDelToCPU <- mkPortSend_Multiplexed("DCache_to_CPU_load_delayed");

    // Outgpong port to CPU with commit immediate response
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_STORE_OUTPUT_IMMEDIATE) storeRspImmToCPU <- mkPortSend_Multiplexed("DCache_to_CPU_store_immediate");

    // Outgoing port to CPU with commit delayed response
    PORT_SEND_MULTIPLEXED#(MAX_NUM_CPUS, DCACHE_STORE_OUTPUT_DELAYED) storeRspDelToCPU <- mkPortSend_Multiplexed("DCache_to_CPU_store_delayed");

    // communication with local controller
    Vector#(2, INSTANCE_CONTROL_IN#(MAX_NUM_CPUS)) inctrls = newVector();
    Vector#(4, INSTANCE_CONTROL_OUT#(MAX_NUM_CPUS)) outctrls = newVector();
    inctrls[0] = loadReqFromCPU.ctrl;
    inctrls[1] = storeReqFromCPU.ctrl;
    outctrls[0] = loadRspImmToCPU.ctrl;
    outctrls[1] = loadRspDelToCPU.ctrl;
    outctrls[2] = storeRspImmToCPU.ctrl;
    outctrls[3] = storeRspDelToCPU.ctrl;

    LOCAL_CONTROLLER#(MAX_NUM_CPUS) localCtrl <- mkLocalController(inctrls, outctrls);

    // ****** Rules ******

    // stage1_loadReq
    
    // Stores always hit in the cache. Loads also always hit but we also 

    (* conservative_implicit_conditions *)
    rule stage1_loadReq (True);

        // Begin a new model cycle.
        let cpu_iid <- localCtrl.startModelCycle();

        // read store input port
        let store_from_cpu <- storeReqFromCPU.receive(cpu_iid);

        // check request type
        case (store_from_cpu) matches

	    tagged Invalid:
	    begin

                // Propogate the bubble.
	        storeRspImmToCPU.send(cpu_iid, tagged Invalid);
	        storeRspDelToCPU.send(cpu_iid, tagged Invalid);

	    end

	    tagged Valid .req:
	    begin


                // An actual cache would do something with the physical 
                // address to determine hit or miss. We always hit.

	        storeRspImmToCPU.send(cpu_iid, tagged Valid initDCacheStoreOk(req.storeToken));
	        storeRspDelToCPU.send(cpu_iid, tagged Invalid);

	    end

        endcase

        // read load input port
        let msg_from_cpu <- loadReqFromCPU.receive(cpu_iid);

        // check request type
        case (msg_from_cpu) matches

	    tagged Invalid:
	    begin

                // Propogate the bubble.
	        loadRspImmToCPU.send(cpu_iid, tagged Invalid);
	        loadRspDelToCPU.send(cpu_iid, tagged Invalid);
                
            // End of model cycle. (Path 1)
            localCtrl.endModelCycle(cpu_iid, 1);

	    end

	    tagged Valid .req:
	    begin


                // An actual cache would do something with the physical 
                // address to determine hit or miss. We always hit.

	        loadRspImmToCPU.send(cpu_iid, tagged Valid initDCacheLoadHit(req));
	        loadRspDelToCPU.send(cpu_iid, tagged Invalid);

                // End of model cycle. (Path 2)
            localCtrl.endModelCycle(cpu_iid, 2);

	    end

        endcase
         
     

    endrule

endmodule
