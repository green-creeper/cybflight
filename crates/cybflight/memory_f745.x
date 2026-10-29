MEMORY {
    /* STM32F745VG has 1024 KB single-bank Flash:
       - Sector 0 (0x0800_0000, 32K): code entry / vector table
       - Sector 1 (0x0800_8000, 32K) & Sector 2 (0x0801_0000, 32K):
         OR Sectors 6 & 7 (256K each at 0x0808_0000 and 0x080C_0000).
       We reserve the last two 256K sectors (Sectors 6+7, 512K total)
       for the dual-sector KV parameter store, leaving 512K for firmware
       code (Cybflight release image is ~200K). */
    FLASH   : ORIGIN = 0x08000000, LENGTH = 512K
    /* 320 KB contiguous RAM on STM32F745VG (64K DTCM + 240K SRAM1 + 16K SRAM2). */
    RAM     : ORIGIN = 0x20000000, LENGTH = 320K
    /* STM32F745 4 KiB backup SRAM. */
    BKPSRAM : ORIGIN = 0x40024000, LENGTH = 4K
}

SECTIONS {
    .bkpsram (NOLOAD) : ALIGN(8) {
        *(.bkpsram .bkpsram.*)
    } > BKPSRAM
}
