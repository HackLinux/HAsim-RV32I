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

Author: Kermin Fleming
*/

_STACK_SIZE = DEFINED(_STACK_SIZE) ? _STACK_SIZE : 0x500;
_HEAP_SIZE = DEFINED(_HEAP_SIZE) ? _HEAP_SIZE : 0x400;

/* Define Memories in the system */

MEMORY
{
   DDR_256MB_32MX64_rank1_row13_col10_cl2_5_C_MEM0_BASEADDR : ORIGIN = 0x00000000, LENGTH = 0x10000000
   iocm_cntlr : ORIGIN = 0xFFFF8000, LENGTH = 0x00008000
   docm_cntlr : ORIGIN = 0xE8800080, LENGTH = 0x00003F80
}

/* Specify the default entry point to the program */

ENTRY(_boot)
STARTUP(boot.o)

/* Define the sections, and where they are mapped in memory */

SECTIONS
{
.vectors : {
   __vectors_start = .;
   *(.vectors)
   __vectors_end = .;
} > iocm_cntlr

.text : {
   *(.text)
   *(.text.*)
   *(.gnu.linkonce.t.*)
} > iocm_cntlr

.init : {
   KEEP (*(.init))
} > iocm_cntlr

.fini : {
   KEEP (*(.fini))
} > iocm_cntlr

.rodata : {
   __rodata_start = .;
   *(.rodata)
   *(.rodata.*)
   *(.gnu.linkonce.r.*)
   __rodata_end = .;
} > docm_cntlr

.sdata2 : {
   __sdata2_start = .;
   *(.sdata2)
   *(.gnu.linkonce.s2.*)
   __sdata2_end = .;
} > docm_cntlr

.sbss2 : {
   __sbss2_start = .;
   *(.sbss2)
   *(.gnu.linkonce.sb2.*)
   __sbss2_end = .;
} > docm_cntlr

.data : {
   __data_start = .;
   *(.data)
   *(.data.*)
   *(.gnu.linkonce.d.*)
   __data_end = .;
} > docm_cntlr

.got : {
   *(.got)
} > docm_cntlr

.got1 : {
   *(.got1)
} > docm_cntlr

.got2 : {
   *(.got2)
} > docm_cntlr

.ctors : {
   __CTOR_LIST__ = .;
   ___CTORS_LIST___ = .;
   KEEP (*crtbegin.o(.ctors))
   KEEP (*(EXCLUDE_FILE(*crtend.o) .ctors))
   KEEP (*(SORT(.ctors.*)))
   KEEP (*(.ctors))
   __CTOR_END__ = .;
   ___CTORS_END___ = .;
} > docm_cntlr

.dtors : {
   __DTOR_LIST__ = .;
   ___DTORS_LIST___ = .;
   KEEP (*crtbegin.o(.dtors))
   KEEP (*(EXCLUDE_FILE(*crtend.o) .dtors))
   KEEP (*(SORT(.dtors.*)))
   KEEP (*(.dtors))
   __DTOR_END__ = .;
   ___DTORS_END___ = .;
} > docm_cntlr

.fixup : {
   __fixup_start = .;
   *(.fixup)
   __fixup_end = .;
} > docm_cntlr

.eh_frame : {
   *(.eh_frame)
} > docm_cntlr

.jcr : {
   *(.jcr)
} > docm_cntlr

.gcc_except_table : {
   *(.gcc_except_table)
} > docm_cntlr

.sdata : {
   __sdata_start = .;
   *(.sdata)
   *(.gnu.linkonce.s.*)
   __sdata_end = .;
} > docm_cntlr

.sbss : {
   __sbss_start = .;
   *(.sbss)
   *(.gnu.linkonce.sb.*)
   *(.scommon)
   __sbss_end = .;
} > docm_cntlr

.tdata : {
   __tdata_start = .;
   *(.tdata)
   *(.gnu.linkonce.td.*)
   __tdata_end = .;
} > docm_cntlr

.tbss : {
   __tbss_start = .;
   *(.tbss)
   *(.gnu.linkonce.tb.*)
   __tbss_end = .;
} > docm_cntlr

.bss : {
   __bss_start = .;
   *(.bss)
   *(.gnu.linkonce.b.*)
   *(COMMON)
   . = ALIGN(4);
   __bss_end = .;
} > docm_cntlr

.boot0 : {
   __boot0_start = .;
   *(.boot0)
   __boot0_end = .;
} > iocm_cntlr

.boot 0xFFFFFFFC : {
   __boot_start = .;
   *(.boot)
   __boot_end = .;
} 

/* Generate Stack and Heap Sections */

.stack : {
   _stack_end = .;
   . += _STACK_SIZE;
   . = ALIGN(16);
   __stack = .;
} > docm_cntlr

.heap : {
   . = ALIGN(16);
   _heap_start = .;
   . += _HEAP_SIZE;
   . = ALIGN(16);
   _heap_end = .;
} > docm_cntlr

}

