MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 0x1C0000 - 0x100
    CONFIG : ORIGIN = 0x101C0000, LENGTH = 0x40000
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}

EXTERN(BOOT2_FIRMWARE)

SECTIONS {
    /* ### Boot loader */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2

    .config ORIGIN(CONFIG) :
    {
        KEEP(*(.config));
    } > CONFIG
} INSERT BEFORE .text;