import ClientServer::*;
import FIFO::*;
import GetPut::*;
import Vector::*;

function Server#(req_type,resp_type) putGetToServer(Put#(req_type) put, Get#(resp_type) get);
   Server#(req_type,resp_type) rv = ?;
   rv = interface Server
	   interface request = put;
	   interface response = get;
	endinterface;
   return rv;
endfunction


function Client#(req_type,resp_type) putGetToClient(Put#(resp_type) put, Get#(req_type) get);
   Client#(req_type,resp_type) rv = ?;
   rv = interface Client
	   interface response = put;
	   interface request  = get;
	endinterface;
   return rv;
endfunction
