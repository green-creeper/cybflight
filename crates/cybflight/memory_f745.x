MEMORY {
    /* STM32F745VG has 1024 KB single-bank Flash:
       - Sectors 0..5 (0x0800_0000..0x0808_0000, 512K total): firmware code
       - Sectors 6+7  (0x0808_0000..0x0810_0000, 2x256K): dual-sector KV parameter store */
    FLASH   : ORIGIN = 0x08000000, LENGTH = 512K
    /* 64 KB 0-wait-state 64-bit DTCM-RAM dedicated to MSP + interrupt executor stacks.
       Stack overflow below 0x2000_0000 triggers a clean BusFault instead of corrupting .bss. */
    DTCM    : ORIGIN = 0x20000000, LENGTH = 64K
    /* 240 KB SRAM1 only — stay strictly inside contiguous SRAM1 without straddling 0x2004_C000. */
    RAM     : ORIGIN = 0x20010000, LENGTH = 240K
    /* STM32F745 4 KiB backup SRAM. */
    BKPSRAM : ORIGIN = 0x40024000, LENGTH = 4K
}

_stack_start = ORIGIN(DTCM) + LENGTH(DTCM);
_stack_end   = ORIGIN(DTCM);

SECTIONS {
    .bkpsram (NOLOAD) : ALIGN(8) {
        *(.bkpsram .bkpsram.*)
    } > BKPSRAM
}
