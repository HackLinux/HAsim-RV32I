//
// Copyright (c) 2014, Intel Corporation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the Intel Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

//
// Generate clocks of requested frequency.
//

import Clocks::*;
import Real::*;

// Conservative Virtex 6 parameters...
// If you have a better speed grade, you might try something different.
Real fINmin  = 10;
Real fINmax  = 700;
Real fVCOmin = 600;
Real fVCOmax = 1200;
Real fPFDmin = 10;
Real fPFDmax = 450;
Real fOUTmin = 4.69;
Real fOUTmax = 700;
Integer dMax = 80;
Integer mMax = 64;
Integer outDivMax = 128;

// We also have to provide a user clock ratio so that 
module mkUserClock_Ratio#(Integer inFreq,
                          Integer clockMultiplier,
                          Integer clockDivider)  
    (UserClock);

    let clk <- mkUserClock_PLL(inFreq,
                               inFreq*clockMultiplier/clockDivider);

    return clk;
endmodule
