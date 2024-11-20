MEMORY {
	/* 1284p */
    flash (rx) : ORIGIN = 0x1E000, LENGTH = 8K 
	/* flash (rx) : ORIGIN = 0x0, LENGTH = 128K  */
}

SECTIONS {
    .bootloader : 
    {
        *(.bootloader)
    } > flash
}
