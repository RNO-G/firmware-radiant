`ifndef RADIANT_DEBUG_VH
 `define RADIANT_DEBUG_VH

    // There's a lot of debugging options in the firmware, enough that it makes a big resource hog to turn them all on.
    
    // ILA to view the LAB4D control shift register inputs/outputs
 `define LAB4D_SHIFT_REGISTER_DEBUG "FALSE"
    // ILA to view the LAB4D data shift register input/outputs
 `define LAB4D_DATA_REGISTER_DEBUG "FALSE"
    // ILA to view the data coming into/out of the LAB4D RAM (plus a VIO to select which one)
 `define LAB4D_RAM_DEBUG "FALSE"
    // ILA to view the SPI Flash interface
 `define RAD_ID_CTRL_SPI_DEBUG "FALSE"
    // ILA to view the CPLD JTAG interface
 `define RAD_ID_CTRL_JTAG_DEBUG "FALSE"
    // ILA to view the entire WB bus
 `define WBC_INTERCON_DEBUG "TRUE"
    // ILA to view the CalRam process (for channel 0)
 `define WB_CALRAM_DEBUG "TRUE"
    // ILA to view the board manager interface (UART)
 `define BOARDMAN_INTERFACE_DEBUG "FALSE"
    // ILA to view the phase scanner
 `define PHASE_SCANNER_DEBUG "TRUE"
    // ILA for the RADIANT's trigger top
 `define RADIANT_TRIG_TOP_DEBUG "FALSE"
    // ILA for the SPI DMA interface (DON'T USE THIS IT'S WAY TOO FAST FOR AN ILA)
 `define FAST_SPI_FIFO_DEBUG "FALSE"
    // ILA for the SPI DMA engine
 `define SPIDMA_ENGINE_DEBUG "TRUE"    

`endif
