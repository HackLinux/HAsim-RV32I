# Copyright (c) 2009 MIT
#
# Permission is hereby granted, free of charge, to any person
# obtaining a copy of this software and associated documentation
# files (the "Software"), to deal in the Software without
# restriction, including without limitation the rights to use,
# copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following
# conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
# OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
# WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
# OTHER DEALINGS IN THE SOFTWARE.



# Build files
sim_top= CacheTesterSim.bsv
dut_top= CacheTesterFPGA.bsv
syn_top= mkCacheTester
par_project_dir=../../par/rfbase

# use this for imported ip
synth_tcl = ../synplify_xupV2.tcl
synth_sdc = CartPol.sdc 
extra_syn_src = foo.v bar.v
extra_par_files=../../par/topLevel/mkTransceiverFPGA_hw.tcl  ../../par/topLevel/mkTransceiverFPGA.v


par: syn

syn: lib

build: bsim

include ../bsv.mk
