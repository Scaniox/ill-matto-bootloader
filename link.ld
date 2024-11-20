MEMORY {
    /* flash (rx) : ORIGIN = 0x1FC00, LENGTH = 64K */
	flash (rx) : ORIGIN = 0x0, LENGTH = 128K
}

SECTIONS {
    .text : {
        *(.text)
    } > flash
}
