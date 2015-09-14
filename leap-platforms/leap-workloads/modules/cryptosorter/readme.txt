There are many files included in this distribution.  However, the following 
files implement the core functionality of the design and should be examined
to get a flavor of the design operation.  All path names are given relative
to the base directory.

memocodeDesignContest07/hardware/Feeder/BRAMFeeder.bsv
memocodeDesignContest2008/aesCores/bsv/aesCipherTop.bsv - controller for aesCores
memocodeDesignContest2008/sort/Sort.bsv - Builds a full sort treee from VLevel FIFOs
memocodeDesignContest2008/sort/BRAMLevelFIFOAdders/BRAMVLevelFIFO.bsv - time multiplexes a BRAM between multiple
logical fifos.  This is the key module in the sort tree.
memocodeDesignContest2008/ctrl/mkCtrl.bsv - control module orchestrating memory, the sorter, and the aes.
memocodeDesignContest2008/xup/PLBMaster/PLBMaster.bsv - Parametric PLB bus master  
memocodeDesignContest2008/xup/Top/Sorter.bsv - top level module





