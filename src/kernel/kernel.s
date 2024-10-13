/**********************************************************************************************************************
kernel.s

AArch64OS Kernel
================

An AArch64 kernel for Raspberry Pi 4 Model B (RPi4B).

Build
-----
First assemble:

   aarch64-none-elf-as -g -c kernel.s -o kernel.o

Then extract just the binary without the ELF header:

   aarch64-none-elf-objcopy -O binary kernel.o kernel.bin

No linking is required.

Then prepare a FAT32 formatted SD card. The following are the minimum required files on the SD card. All except
kernel.bin and config.txt are from the Raspberry Pi firmware repository:

- kernel.bin
- config.txt
- bcm2711-rpi-4-b.dtb
- fixup4.dat
- start4.elf
- overlays/disable-bt.dtbo

The contents of config.txt must be:

   # Defaults to loading kernel8.img if not specified. Better to be explicit.
   kernel=kernel.bin

   # load the kernel to the memory address 0x0. This disables the armstub and allows booting into Exception level 3
   # with all cores running.
   kernel_old=1

   # Set the ARM architecture to 64-bit. This causes the kernel to be relocated to 0x80000.
   # If kernel= is not set and we use the default kernel name of kernel8.img, we don't have to set this as it
   # relocates the kernel to 0x80000 anyway.
   arm_64bit=1

   # Set the disable_commandline_tags command to 1 to stop start4.elf from filling in ATAGS (memory from 0x100)
   # before launching the kernel.
   # Boot fails with kernel_old=1 if this is not set because it overwrites the kernel.
   disable_commandline_tags=1

   # Specifies how much memory, in megabytes, to reserve for the exclusive use of the GPU
   # 76MB is the default if more than 1GB of RAM is installed. Setting explicitly to 76MB to avoid confusion.
   # The top of the video core memory is at 0x0_4000_0000 (1024 MiB) so setting this to 76 means the video core
   # memory will start at 0x0_3B40_0000 (948 MiB).
   gpu_mem=76

   # Set the device tree address. By default it loads at 0x100 which was being overwritten by the kernel when
   # kernel_old=1.
   # Setting to much higher address like 0x3AC00000 (just below video core memory) results in the following error:
   # "dterror: not a valid FDT - err -9"
   device_tree_address=0x20000000 # 512 MiB

   # disable bluetooth so that UART0 (PL011) can be the primary UART. Otherwise, the mini UART is the primary UART
   # and it is not as full-featured as the PL011.
   dtoverlay=disable-bt

   # log start4.elf debug output to UART
   uart_2ndstage=1

   # Disable pull downs (required for JTAG)
   gpio=22-27=np

   # Enable JTAG pins (i.e. GPIO22-GPIO27)
   enable_jtag_gpio=1

Debugging
---------
For remote debugging with a JTAG debug probe and GDB, it is useful to have an ELF file with the start address
changed to 0x80000 which is where the 64-bit Arm core expects to find the kernel. This file can be used with
the GDB load command:

    aarch64-none-elf-objcopy -O elf64-littleaarch64 --change-address 0x80000 kernel.o kernel.elf

References
----------
1. ARM Cortex-A72 MPCore Processor Technical Reference Manual: https://developer.arm.com/documentation/100095/0003/
2. Arm Architecture Reference Manual for A-profile architecture: https://developer.arm.com/documentation/ddi0487/latest/
3. AArch64 memory management examples: https://developer.arm.com/documentation/102416/0100

Notes
-----
- use 4 byte alignment for all functions
- not using bss section since we are not using C so no need to zero out memory
- use `bl` instead of `b` even for 'no return' functions so the link register gets set for debugging

Use simple code to allow bootstrapping with a minimal assembler:
- no linking
- no macros
- no sections
- minimal number of mnemonics and directives
- no expressions (using them for string lengths and similar for now, but can replace with constants)
- no local labels or local symbol names
- no pseudo-ops like ldr x1,=label, manually create literal pools when needed

Register Usage
--------------
- x0-x7: argument registers
- x8: indirect result location register and used for syscall number
- x9-x15: temporary registers
- x16-x17: intra-procedure-call temporary registers (can be used by linker generated call veneers and Procedure
           Linkage Table code). Not planning to use these, but avoid using them in case they are needed.
- x18: platform register (not currently using this for anything, but avoid this register in case it is needed)
- x19-x28: callee-saved registers
- x29 (fp): frame pointer
- x30 (lr): link register
- x31 (xzr): zero register

Raspberry Pi 4 Model B Memory Map
---------------------------------
This is a memory map for the Raspberry Pi 4 Model B. The memory map is based on
[Raspberry Pi BC2711 ARM Peripherals](https://datasheets.raspberrypi.org/bcm2711/bcm2711-peripherals.pdf)
and represents the "Low Peripheral" mode, which is the default for the RPi4B. The only difference with this and the
"Full 35-bit Address Map" is the main peripherals are moved to the top of the lower Reserved region, and the Arm Local
Peripherals are moved to the bottom of the upper Reserved region.

| Description             | Start         | End           | Size (MiB) |
| ----------------------- | ------------- | ------------- | ---------- |
| PCIe                    | 0x6_0000_0000 | 0x7_FFFF_FFFF |  8191      |
| Reserved                | 0x4_C000_0000 | 0x6_0000_0000 |  5120      |
| L2 Cache Non-Allocating | 0x4_8000_0000 | 0x4_C000_0000 |  1024      |
| Reserved                | 0x4_4000_0000 | 0x4_8000_0000 |  1024      |
| L2 Cache Allocating     | 0x4_0000_0000 | 0x4_4000_0000 |  1024      |
| SDRAM (Arm)             | 0x1_0000_0000 | 0x4_0000_0000 | 12288      |
| Arm Local Peripherals   | 0x0_FF80_0000 | 0x1_0000_0000 |     8      |
| Main Peripherals        | 0x0_FC00_0000 | 0x0_FF80_0000 |    56      |
| SDRAM (Arm)             | 0x0_4000_0000 | 0x0_FC00_0000 |  3008      |
| SDRAM (Video Core)      | 0x0_3B40_0000 | 0x0_4000_0000 |    76      |
| SDRAM (Arm)             | 0x0_0000_0000 | 0x0_3B40_0000 |   948      |

There is also the "Legacy Master View" of the address map:

| Description             | Start         | End           | Size (MiB) |
| ----------------------- | ------------- | ------------- | ---------- |
| SDRAM (Arm)             | 0x0_C000_0000 | 0x0_FFFF_FFFF |  1023      |
| L2 Cache Non-Allocating | 0x0_8000_0000 | 0x0_C000_0000 |  1024      |
| Main Peripherals        | 0x0_7C00_0000 | 0x0_8000_0000 |    64      |
| Reserved                | 0x0_4000_0000 | 0x0_7c00_0000 |   960      |
| L2 Cache Allocating     | 0x0_0000_0000 | 0x0_4000_0000 |  1024      |

AArch64OS Memory Layout
---------------------------------

| Description           | Start                  | End                    | Notes                                      |
| --------------------- | ---------------------- | ---------------------- | ------------------------------------------ |
| arm local peripherals | 0x0_FF80_0000 (4088MB) | 0x1_0000_0000 (4096MB) | Device-nGnRnE, EL1=RW, not executable      |
| main peripherals      | 0x0_FC00_0000 (4032MB) | 0x0_FF80_0000 (4088MB) | Device-nGnRnE, EL1=RW, not executable      |
| unused                | 0x0_4000_0000 (1024MB) | 0x0_FC00_0000 (4032MB) |                                            |
| video core memory     | 0x0_3B40_0000 ( 948MB) | 0x0_4000_0000 (1024MB) | Normal-noncacheable, EL1=RW, not executable|
| userspace memory      | 0x0_2000_0000 ( 512MB) | 0x0_3B40_0000 ( 948MB) | Normal-cacheable, EL0=RW                   |
| kernel memory         | 0x0_0080_0000 (   8MB) | 0x0_2000_0000 ( 512MB) | Normal-cacheable, EL1=RW, not executable   |
| kernel image          | 0x0_0008_0000 ( 0.5MB) | 0x0_0080_0000 (   8MB) | Normal-cacheable, EL1=RO, executable       |
| unused                | 0x0_0000_0000 (   0MB) | 0x0_0008_0000 ( 0.5MB) | bootloader load area; nothing useful here  |

- The device tree is loaded by bootloader at 0x2000_0000 (about ~55KB), but not using it. If we need it we can move it.
- Bootloader loads the kernel at 0x0 but start4.elf relocates it to 0x80000.

Translation Tables
------------------

| Entry             | Description                      | Range                                     | Notes             |
| ----------------- | -------------------------------- | ----------------------------------------- | ----------------- |
| tt_l1_0[0]        | table descriptor (tt_l2_0_0)     | 0x0000_0000 - 0x3FFF_FFFF (0 to 1GB)      |                   |
| tt_l1_0[1]        | invalid (0x0)                    | 0x4000_0000 - 0x7FFF_FFFF (1 to 2GB)      |                   |
| tt_l1_0[2]        | invalid (0x0)                    | 0x8000_0000 - 0xBFFF_FFFF (2 to 3GB)      |                   |
| tt_l1_0[3]        | table descriptor (tt_l2_0_3)     | 0xC000_0000 - 0xFFFF_FFFF (3 to 4GB)      |                   |
| ----------------- | -------------------------------- | ----------------------------------------- | ------------------|
| tt_l2_0_0[0-3]    | block descriptors (EL1 RO, exec) | 0x0000_0000 - 0x007F_FFFF (0 to 8MB)      |kernel text/data.ro|
| tt_l2_0_0[4]      | block descriptor (EL1 RW)        | 0x0080_0000 - 0x009F_FFFF (8 to 10MB)     | kernel data       |
| tt_l2_0_0[5-254]  | invalid (0x0)                    | 0x00A0_0000 - 0x1FDF_FFFF (10 to 510MB)   |                   |
| tt_l2_0_0[255]    | block descriptor (EL1 RW)        | 0x1FE0_0000 - 0x1FFF_FFFF (510 to 512MB)  | kernel stack 0    |
| tt_l2_0_0[256]    | block descriptor (EL0 RO, exec)  | 0x2000_0000 - 0x201F_FFFF (512 to 514MB)  | user text/data.ro |
| tt_l2_0_0[257-472]| invalid (0x0)                    | 0x2020_0000 - 0x3B1F_FFFF (514 to 946MB)  |                   |
| tt_l2_0_0[473]    | block descriptor (EL0 RW)        | 0x3B20_0000 - 0x3B3F_FFFF (946 to 948MB)  | user stack 0      |
| tt_l2_0_0[474-511]| block descriptors (EL1 RWnocache)| 0x3B40_0000 - 0x3FFF_FFFF (948 to 1024MB) | video core        |
| ----------------- | -------------------------------- | ----------------------------------------- | ----------------- |
| tt_l2_0_3[0-479]  | invalid (0x0)                    | 0xC000_0000 - 0xFBFF_FFFF (3072 to 4032MB)|                   |
| tt_l2_0_3[480-511]| block descriptor (device)        | 0xFC00_0000 - 0xFFFF_FFFF (4032 to 4096MB)| peripherals       |

- Using MMU because many Arm features depend on it, but using  a 1:1 mappping to keep simple.
- tt_l1_0: 512 entries; either 1GB block descriptors or table descriptors for a level 2 table.
- tt_l2_0_0: 512 entries; either 2MB block descriptors or table descriptors for a level 3 table.
- tt_l2_0_3: 512 entries; either 2MB block descriptors or table descriptors for a level 3 table.
- Kernel stack starts at 0x2000_0000 and grow down.
- Userspace stack starts at 0x3B40_0000 and grows down.
- Video core memory is RW to allow writing to framebuffer.
- Kernel image cannot be EL0=RW or it automatically becomes PXN (privileged execute never) and cannot be executed.

***********************************************************************************************************************/

// Defines /////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Standard error codes:
	.set	ERROR_NONE,0
	.set	ERROR,1

// Startup Code ////////////////////////////////////////////////////////////////////////////////////////////////////////

// NAME
//	_start - kernel entrypoint
//
// SYNOPIS
//	void _start(void);
//
// DESCRIPTION
//	_start() is the entrypoint for the kernel. It starts a chain of non-returning functions that
//	initialize the exception levels and puts unused cores to sleep and ultimately starts userspace.
//
//	The name does not matter since the kernel is not linked. The important thing is that this is the
//	first byte of the binary.
//
// RETURN VALUE
//	Does not return.
_start:
	// check the processor ID to determine which core we are running on
	mrs	x9,mpidr_el1	// multi-processor affinity register for EL1
	and	x9,x9,0xFF	// mask bits 0-7 (Aff0) which contain processor ID for RPi4B

	// branch to core-specific code
	cmp	x9,0	// check if core 0
	b.eq	init_el3	// init EL3 on core 0 (no return)
	cmp	x9,1	// check if core 1
	b.eq	stop_core	// stop core 1 (no return)
	cmp	x9,2	// check if core 2
	b.eq	stop_core	// stop core 2 (no return)
	cmp	x9,3	// check if core 3
	b.eq	stop_core	// stop core 3 (no return)
	b	.	// should never reach here

// NAME
//	stop_core - stop the core
//
// SYNOPIS
//	void stop_core(void);
//
// DESCRIPTION
//	stop_core() puts the core to sleep.

// RETURN VALUE
//	Does not return.
// TODO: allow for core to be woken up. See armstub8.S from Raspberry Pi for example.
stop_core:
	b	sleep_forever
	b	.	// should never reach here

// NAME
//	init_el3 - initialize exception level 3 (EL3)
//
// SYNOPIS
//	void init_el3(void);
//
// DESCRIPTION
//	init_el3() initializes exception level 3 (EL3) then branches to init_el2().
// RETURN VALUE
//	Does not return.
init_el3:
	// Make sure we are in EL3 (not splitting into a function b/c stack is not initialized yet).
	mrs	x19,currentel	// read current exception level (use callee-saved register, since we need it after a bl)
	lsr	x19,x19,2	// shift right to get EL
	and	x19,x19,0x3	// mask bits 0-1
	cmp	x19,3	// check if we are at EL3
	b.eq	.init_el3_good_el	// if we are at EL3, continue

	// Not EL3; log an error message and put the core to sleep
	adr	x0,not_el3_msg
	mov	x1,not_el3_msg_size
	bl	write_bytes_pri_uart

	// put the core to sleep
	bl	sleep_forever

.init_el3_good_el:	// Log that we are initializing EL3.
	adr	x0,init_el3_msg
	mov	x1,init_el3_msg_size
	bl	write_bytes_pri_uart

	// Setup EL3.
	ldr	x9,stack_top_el3_0
	mov	sp,x9	// set EL3 stack pointer (early as possible so can safely do nested bl)

	adr	x9,el3_vector_table
	msr	vbar_el3,x9	// set EL3 vector base address

	msr	cptr_el3, xzr	// Disable trapping of accesses to CPACR, CPACR_EL1, HCPTR,
			// CPTR_EL2, trace, Activity Monitor, SME, Streaming SVE,
			// SVE, and Advanced SIMD and floating-point functionality to EL3.

	// Set up the EL3 secure configuration register.
	// Raspberry Pi armstub8.S uses (SCR_RW | SCR_HCE | SCR_SMD | SCR_RES1_5 | SCR_RES1_4 | SCR_NS)
	mov	x9,1	// NS[0]=1: EL2 and EL1 are non-secure
	orr	x9,x9,(1<<1)	// IRQ[1]=1: IRQs routed to EL3 (TODO: do we want this?)
	orr	x9,x9,(1<<2)	// FIQ[2]=1: FIQs routed to EL3 (TODO: do we want this?)
	orr	x9,x9,(1<<3)	// EA[3]=1: SError routed to EL3 (TODO: do we want this?)
	orr	x9,x9,(1<<4)	// RES1[4]=1: Reserved, RPi ampstub8.S sets this.
	orr	x9,x9,(1<<5)	// RES1[5]=1: Reserved, RPi ampstub8.S sets this.
			// RES0[6]=0: Reserved
	orr	x9,x9,(1<<7)	// SMD[7]=1: Secure Monitor Call (SMC) instructions are disabled
	orr	x9,x9,(1<<8)	// HCE[8]=1: HVC instructions are enabled
			// SIF[9]=0: Secure state instruction fetches from Non-secure memory are permitted
	orr	x9,x9,(1<<10)	// RW[10]=1: Next EL down uses AArch64
	orr	x9,x9,(1<<11)	// ST[11]=1: Secure EL1 can access timers
			// TWI[12]=0: EL2, EL1 and EL0 execution of WFI instructions is not trapped to EL3
			// TWE[13]=0: EL2, EL1 and EL0 execution of WFE instructions is not trapped to EL3
	msr	scr_el3, x9	// set secure configuration register

	// Prepare to jump to EL2.
	msr	sctlr_el2,xzr	// set EL2 system control register to safe defaults
	msr	hcr_el2,xzr	// set EL2 hypervisor configuration register to safe defaults

	mov	x9,0b1001	// M[3:0]=0b1001: EL2 with SP_EL2 (EL2h)
			// M[4]=0: AArch64
			// F[6]=0: FIQs are not masked
			// I[7]=0: IRQs are not masked
			// A[8]=0: SError exceptions are not masked
			// D[9]=0: Debug exceptions are not masked
	msr	spsr_el3, x9	// set EL3 saved program status register

	adr	x9,init_el2
	msr	elr_el3,x9	// set EL3 exception link register to start EL2 at init_el2

.init_el3_done:	// log EL3 initialization complete
	adr	x0,init_el3_done_msg
	mov	x1,init_el3_done_msg_size
	bl	write_bytes_pri_uart

	eret		// return to EL2

// NAME
//	init_el2 - initialize exception level 2 (EL2)
//
// SYNOPIS
//	void init_el2(void);
//
// DESCRIPTION
//	init_el2() initializes exception level 2 (EL2) then branches to init_el1().
// RETURN VALUE
//	Does not return.
init_el2:
	// Make sure we are in EL2 (not splitting into a function b/c stack is not initialized yet).
	mrs	x19,currentel	// read current exception level (use callee-saved register, since we need it after a bl)
	lsr	x19,x19,2	// shift right to get EL
	and	x19,x19,0x3	// mask bits 0-1
	cmp	x19,2	// check if we are at EL2
	b.eq	.init_el2_good_el	// if we are at EL2, continue

	// Not EL2; log an error message and put the core to sleep
	adr	x0,not_el2_msg
	mov	x1,not_el2_msg_size
	bl	write_bytes_pri_uart

	// put the core to sleep
	bl	sleep_forever

.init_el2_good_el:	// Log that we are initializing EL2.
	adr	x0,init_el2_msg
	mov	x1,init_el2_msg_size
	bl	write_bytes_pri_uart

	// Setup EL2.
	ldr	x9,stack_top_el2_0
	mov	sp,x9	// set EL2 stack pointer

	adr	x9,el2_vector_table
	msr	vbar_el2,x9	// set EL2 vector base address

	msr	cptr_el2, xzr	// Disable trapping of accesses to CPACR, CPACR_EL1, trace,
			// Activity Monitor, SME, Streaming SVE, SVE, and Advanced
			// SIMD and floating-point functionality to EL2.

	msr	vttbr_el2,xzr	// VMID[63:48]=0: Although we are not using stage 2
			// translation, NS.EL1 still cares about the VMID.

	// Set up the EL2 hypervisor configuration register.
	mov	x9,0	// VM[0]=0: EL1&0 stage 2 address translation disabled
	orr	x9,x9,(1<<3)	// FMO[3]=1: Physical FIQ routing (TODO: do we want this?)
	orr	x9,x9,(1<<4)	// IMO[4]=1: Physical IRQ routing (TODO: do we want this?)
	orr	x9,x9,(1<<31)	// RW[31]=1: NS.EL1 is AArch64
			// TGE[27]=0: Entry to NS.EL1 is possible
	msr	hcr_el2,x9	// set hypervisor configuration register

	// From ref. 3: Reads of the MPIDR_EL1 and MIDR_EL1 registers at NS.EL1 return virtual values. The
	// registers which hold these virtual values, VMPIDR_EL2 and VPIDR_EL2, do not have defined reset
	// values. Software should initialize these registers before entering EL1 for the first time. For
	// this example, we are not using virtualization. This means that we can copy the physical values.
	mrs	x9, midr_el1
	msr	vpidr_el2, x9
	mrs	x9, mpidr_el1
	msr	vmpidr_el2, x9

	// Prepare to jump to EL1.
	msr	sctlr_el1,xzr	// set EL1 system control register to safe defaults

	mov	x9,0b0101	// M[3:0]=0b0101: EL1 with SP_EL1 (EL1h).EL1 with SP_EL1 (EL1h)
			// M[4]=0: AArch64
			// F[6]=0: FIQs are not masked
			// I[7]=0: IRQs are not masked
			// A[8]=0: SError exceptions are not masked
			// D[9]=0: Debug exceptions are not masked
	msr	spsr_el2, x9	// set EL2 saved program status register

	adr	x9,init_el1
	msr	elr_el2,x9	// set EL2 exception link register to start EL1 at init_el1

.init_el2_done:	// log EL2 initialization complete
	adr	x0,init_el2_done_msg
	mov	x1,init_el2_done_msg_size
	bl	write_bytes_pri_uart

	eret		// return to EL1

// NAME
//	init_el1 - initialize exception level 1 (EL1)
//
// SYNOPIS
//	void init_el1(void);
//
// DESCRIPTION
//	init_el1() initializes exception level 1 (EL1) then branches to init_el0().
// RETURN VALUE
//	Does not return.
init_el1:
	// Make sure we are in EL1 (not splitting into a function b/c stack is not initialized yet).
	mrs	x19,currentel	// read current exception level (use callee-saved register, since we need it after a bl)
	lsr	x19,x19,2	// shift right to get EL
	and	x19,x19,0x3	// mask bits 0-1
	cmp	x19,1	// check if we are at EL1
	b.eq	.init_el1_good_el	// if we are at EL1, continue

	// Not EL1; log an error message and put the core to sleep
	adr	x0,not_el1_msg
	mov	x1,not_el1_msg_size
	bl	write_bytes_pri_uart

	// put the core to sleep
	bl	sleep_forever

.init_el1_good_el:	// Log that we are initializing EL1.
	adr	x0,init_el1_msg
	mov	x1,init_el1_msg_size
	bl	write_bytes_pri_uart

	// Setup EL1.
	ldr	x9,stack_top_el1_0
	mov	sp,x9	// set EL1 stack pointer

	adr	x9,el1_vector_table
	msr	vbar_el1,x9	// set EL1 vector base address

	// Check that we have a device tree where we expect it. We are not using the device tree, but want
	// to make sure it was put where we expected it.
	bl	verify_device_tree	// verify device tree magic bytes
	cmp	x0,ERROR_NONE	// check if device tree is valid
	b.eq	.init_el1_good_dt	// if valid, continue
	bl	sleep_forever	// else, sleep forever (verify_device_tree logs error)

.init_el1_good_dt:

	// Copy userspace from userspace_start to userspace_addr (userspace_size bytes)
	adr	x9,userspace_addr
	ldr	x9,[x9]	// x9 = userspace entrypoint
	adr	x0,userspace_start
	mov	x1,x9
	mov	x2,userspace_size
	bl	copy_balign_16
	cmp	x0,ERROR_NONE	// check if copy was successful
	b.eq	.init_el1_good_copy	// if successful, continue
	adr	x0,not_balign_16_msg
	mov	x1,not_balign_16_msg_size
	bl	write_bytes_pri_uart
	bl	sleep_forever	// else, sleep forever

.init_el1_good_copy:

	// Enable MMU
	bl	enable_el1_el0_mmu


	// Prepare to jump to EL0.
	ldr	x9,stack_top_el0_0
	msr	sp_EL0,x9	// set EL0 stack pointer (cannot do it in EL0 so have to do here)

	mov	x9,0	// M[3:0]=0000: EL0
			// M[4]=0: AArch64
			// F[6]=0: FIQs are not masked
			// I[7]=0: IRQs are not masked
			// A[8]=0: SError exceptions are not masked
			// D[9]=0: Debug exceptions are not masked
	msr	spsr_el1,x9	// set EL1 saved program status register

	adr	x9,userspace_addr
	ldr	x9,[x9]	// x9 = address of userspace entrypoint
	msr	elr_el1,x9	// set EL1 exception link register to start EL0 at userspace entrypoint

.init_el1_done:	// log EL1 initialization complete
	adr	x0,init_el1_done_msg
	mov	x1,init_el1_done_msg_size
	bl	write_bytes_pri_uart

	eret		// return to EL0

// NAME
//	copy_balign_16 - copy 16-byte aligned bytes
//
// SYNOPIS
//	error copy_balign_16(void *src, void *dst, size size);
//
// DESCRIPTION
//	copy_balign_16() copies 16-byte aligned bytes from src to dst for size bytes.
//
// RETURN VALUE
//	Returns ERROR_NONE if the data is aligned, otherwise it returns ERROR.
copy_balign_16:
	stp	fp,lr,[sp,-16]!	// save frame pointer and link register

	// Check for alignment.
	and	x9,x0,0xF
	cbnz	x9,copy_b_16_unaligned
	and	x9,x1,0xF
	cbnz	x9,copy_b_16_unaligned
	and	x9,x2,0xF
	cbnz	x9,.copy_b_16_unaligned
	mov	x9,16
	udiv	x2,x2,x9	// divide by 16 to get number of 16-byte blocks
	b	.copy_b_16_loop

.copy_b_16_unaligned:
	mov	x0,ERROR
	b .copy_b_16_out

.copy_b_16_loop:	ldp	x3,x4,[x0],16	// read bytes from source and increment source address
	stp	x3,x4,[x1],16	// write byte to destination and increment destination address
	subs	x2,x2,1		// decrement number of bytes to copy
	b.ne	.copy_b_16_loop	// if not done, repeat
	mov	x0,ERROR_NONE	// return ERROR_NONE

.copy_b_16_out:	ldp	fp,lr,[sp],16	// restore frame pointer and link register
	ret

// NAME
//	verify_device_tree - verify device tree magic bytes
//
// SYNOPIS
//	error verify_device_tree(void);
//
// DESCRIPTION
//	verify_device_tree() verifies the device tree magic bytes in the device tree at address 0x20000000 or 0x0.
//
// RETURN VALUE
//	Returns E_OK if the device tree magic bytes are correct, otherwise it logs an error message and returns E_ERROR.
verify_device_tree:
	stp	fp,lr,[sp,-16]!	// save frame pointer and link register

	// verify device tree magic bytes
	// device tree address defined in config.txt: device_tree_address=0x20000000
	// must contain 0xD00DFEED (big-endian), 0xEDFE0DD0 (little-endian) in first 4 bytes
	mov	x1,0x20000000	// address of device tree

.verify_d_t_verify:	ldrb	w0,[x1],1	// read first byte of device tree and increment address
	cmp	x0,0xD0	// check if first byte is 0xD0
	b.ne	.verify_d_t_bad	// if not, log
	ldrb	w0,[x1],1	// read second byte of device tree and increment address
	cmp	x0,0x0D	// check if second byte is 0x0D
	b.ne	.verify_d_t_bad	// if not, log
	ldrb	w0,[x1],1	// read third byte of device tree and increment address
	cmp	x0,0xFE	// check if third byte is 0xFE
	b.ne	.verify_d_t_bad	// if not, log
	ldrb	w0,[x1],1	// read fourth byte of device tree and increment address
	cmp	x0,0xED	// check if fourth byte is 0xED
	b.ne	.verify_d_t_bad	// if not, log
	b	.verify_d_t_good	// if all bytes are correct, continue

.verify_d_t_qemu:	// try QEMU address for dtb
	mov	x1,0	// address of device tree for qemu
	b	.verify_d_t_verify	// repeat device tree check


.verify_d_t_bad:	// log that we found an invalid device tree and sleep
	cmp	x1,0	// check which address we were reading from
	b.ne	.verify_d_t_qemu	// try QEMU address for dtb
	adr	x0,bad_dtb_msg
	mov	x1,bad_dtb_msg_size
	bl	write_bytes_pri_uart
	mov	x0,ERROR	// return ERROR
	b	.verify_d_t_out

.verify_d_t_good:	// log that we found a valid device tree
	adr	x0,good_dtb_msg
	mov	x1,good_dtb_msg_size
	bl	write_bytes_pri_uart
	mov	x0,ERROR_NONE	// return ERROR_NONE
	b	.verify_d_t_out

.verify_d_t_out:	ldp	fp,lr,[sp],16	// restore frame pointer and link register
	ret

// NAME
//	enable_el1_el0_mmu - enable EL1 and EL0 MMU
//
// SYNOPIS
//	void enable_el1_el0_mmu(void);
//
// DESCRIPTION
//	enable_el1_el0_mmu() enables the EL1 and EL0 MMU.
//
enable_el1_el0_mmu:
	stp	fp,lr,[sp,-16]!	// save frame pointer and link register

	// Set translation table address (start at level 1, and only use ttbr0_el1, not ttbr1_el1).
	adr	x9,tt_l1_0	// x9=address of level 1 translation table
	msr	ttbr0_el1,x9	// set translation table base register 0

	// Setup the three memory attribute types we will use.
	mov	x9,0x44	// Attr0[7:0]=0b01000100: Normal, Inner/Outer Non-Cacheable
	orr	x9,x9,(0xFF<<8)	// Attr1[15:8]=0b11111111: Normal, Inner/Outer WB/WA/RA
			// Attr2[23:16]=0b00000000: Device-nGnRnE
	msr	mair_el1,x9	// set memory attribute indirection register

	// Set translation control register.
	mov	x9,25	// T0SZ[5:0]=0b011001: size offset of TTBR0 memory region
			// memory size is limited to 2^(64-T0SZ) = 2^39 = 512GB
			// TTBRO properties:
			// EPD0[7]=0: enable table walks from TTBR0
	orr	x9,x9,(0x1<<8)	// IGRN0[9:8]=0b01: normal inner write-back read-allocate write-allocate cacheable
	orr	x9,x9,(0x1<<10)	// OGRN0[11:10]=0b01: normal outer write-back read-allocate write-allocate cacheable
	orr	x9,x9,(0x3<<12)	// SH0[13:12]=0b11: inner shareable
			// TGO0[15:14]=0b00: 4KB granule
			// TBI0[37]=0b0: disable top byte ignore
			// TTBR1 properties:
	orr	x9,x9,(0x1<<23)	// EPD1[23]=0b1: disable table walks from TTBR1 so don't need to set other properties
			// misc properties:
			// A1[22]=0b0: TTBR0 contains the ASID
			// AS[26]=0b0: 8-bit ASID
			// IPS[34:32]=0b000: 32-bit ipa space
	msr	tcr_el1,x9	// set translation control register

	// Ensure changes to system register are visible before MMU enabled.
	isb

	// Invalidate TLB.
	tlbi	vmalle1
	dsb	sy
	isb

	// Generate Translation Tables.

	// Translation Table Block Descriptor Format:
	// [0]: valid bit
	// [1]: table bit (0 for block, 1 for table)
	// AttrIdx[4:2]: memory attribute index from MAIR_EL1
	// NS[5]: non-secure bit (ignored at EL1)
	// AP[7:6]: access permissions
	// SH[9:8]: shareability
	// AF[10]: access flag
	// nG[11]: not global bit
	// DBM[51]: dirty bit modifier
	// Contig[52]: contiguous bit
	// PXN[53]: privileged execute never bit
	// UXN[54]: unprivileged execute never bit

	// First fill tables with faults.
	adr	x9,tt_l1_0	// x9=address of level 1 translation table
	mov	w10,512	// number of entries
.enable_e_e_m_1:	stp	xzr,xzr,[x9],16	// 0x0 (fault) into table entries
	sub	w10,w10,2	// decrement count by 2 (writing two entries at once)
	cbnz	w10,.enable_e_e_m_1

	adr	x9,tt_l2_0_0	// x9=address of level 2 translation table 0
	mov	w10,512	// number of entries
.enable_e_e_m_2:	stp	xzr,xzr,[x9],16	// 0x0 (fault) into table entries
	sub	w10,w10,2	// decrement count by 2 (writing two entries at once)
	cbnz	w10,.enable_e_e_m_2

	adr	x9,tt_l2_0_3	// x9=address of level 2 translation table 3
	mov	w10,512	// number of entries
.enable_e_e_m_3:	stp	xzr,xzr,[x9],16	// 0x0 (fault) into table entries
	sub	w10,w10,2	// decrement count by 2 (writing two entries at once)
	cbnz	w10,.enable_e_e_m_3

	// Populate level 1 translation table 0.
	adr	x9,tt_l1_0	// x9=address of level 1 translation table
	// tt_l1_0[0]: table descriptor tt_l2_0_0, 0x0000_0000 - 0x3FFF_FFFF (0GB to 1GB)
	mov	x10,0b11	// valid bit, table type
	adr	x11,tt_l2_0_0
	orr	x10,x10,x11	// address = tt_l2_0_0
	str	x10,[x9],8	// write block descriptor to table

	// skip 1-2
	add	x9,x9,8*(2-1+1)

	// tt_l1_0[3]: table descriptor tt_l2_0_3, 0xC000_0000 - 0xFFFF_FFFF (3GB to 4GB)
	mov	x10,0b11	// valid bit, table type
	adr	x11,tt_l2_0_3
	orr	x10,x10,x11	// address = tt_l2_0_3
	str	x10,[x9],8	// write block descriptor to table

	// Populate level 2 translation table 0.
	adr	x9,tt_l2_0_0	// x9=address of level 2 translation table 0
	// tt_l2_0_0[0]: block descriptor (EL1 RO, exec), 0x0000_0000 - 0x001F_FFFF (0 to 2MB)kern text/ro
	mov	x10,0b01	// valid bit, block type
	orr	x10,x10,(0b001<<2)	// AttrIdx[4:2]=0b001: Attr1: Normal, Inner/Outer WB/WA/RA
	orr	x10,x10,(0b10<<6)	// AP[7:6]=0b10: EL1/2/3 RO, EL0 No Access
	orr	x10,x10,(0b11<<8)	// SH[9:8]=0b11: inner shareable
	orr	x10,x10,(0b1<<10)	// AF[10]=1: access flag
			// PXN[53]=0: EL1/2/3 can execute
			// UXN[54]=0: EL0 can execute (EL0 can't access though)
			// address = 0x0
	str	x10,[x9],8	// write block descriptor to table

	// tt_l2_0_0[1]: block descriptor (EL1 RO, exec), 0x0020_0000 - 0x003F_FFFF (2 to 4MB)kernel text/ro
	mov	x10,0b01	// valid bit, block type
	orr	x10,x10,(0b001<<2)	// AttrIdx[4:2]=0b001: Attr1: Normal, Inner/Outer WB/WA/RA
	orr	x10,x10,(0b10<<6)	// AP[7:6]=0b10: EL1/2/3 RO, EL0 No Access
	orr	x10,x10,(0b11<<8)	// SH[9:8]=0b11: inner shareable
	orr	x10,x10,(0b1<<10)	// AF[10]=1: access flag
			// PXN[53]=0: EL1/2/3 can execute
			// UXN[54]=0: EL0 can execute (EL0 can't access though)
	orr	x10,x10,0x00200000	// address = 0x0020_0000
	str	x10,[x9],8	// write block descriptor to table

	// tt_l2_0_0[2]: block descriptor (EL1 RO, exec), 0x0040_0000 - 0x005F_FFFF (4 to 6MB)kernel text/ro
	mov	x10,0b01	// valid bit, block type
	orr	x10,x10,(0b001<<2)	// AttrIdx[4:2]=0b001: Attr1: Normal, Inner/Outer WB/WA/RA
	orr	x10,x10,(0b10<<6)	// AP[7:6]=0b10: EL1/2/3 RO, EL0 No Access
	orr	x10,x10,(0b11<<8)	// SH[9:8]=0b11: inner shareable
	orr	x10,x10,(0b1<<10)	// AF[10]=1: access flag
			// PXN[53]=0: EL1/2/3 can execute
			// UXN[54]=0: EL0 can execute (EL0 can't access though)
	orr	x10,x10,0x00400000	// address = 0x0040_0000
	str	x10,[x9],8	// write block descriptor to table

	// tt_l2_0_0[3]: block descriptor (EL1 RO, exec), 0x0060_0000 - 0x007F_FFFF (6 to 8MB)kernel text/ro
	mov	x10,0b01	// valid bit, block type
	orr	x10,x10,(0b001<<2)	// AttrIdx[4:2]=0b001: Attr1: Normal, Inner/Outer WB/WA/RA
	orr	x10,x10,(0b10<<6)	// AP[7:6]=0b10: EL1/2/3 RO, EL0 No Access
	orr	x10,x10,(0b11<<8)	// SH[9:8]=0b11: inner shareable
	orr	x10,x10,(0b1<<10)	// AF[10]=1: access flag
			// PXN[53]=0: EL1/2/3 can execute
			// UXN[54]=0: EL0 can execute (EL0 can't access though)
	orr	x10,x10,0x00600000	// address = 0x0060_0000
	str	x10,[x9],8	// write block descriptor to table

	// tt_l2_0_0[4]: block descriptor (EL1 RW), 0x0080_0000 - 0x009F_FFFF (8 to 10MB), kernel data
	mov	x10,0b01	// valid bit, block type
	orr	x10,x10,(0b001<<2)	// AttrIdx[4:2]=0b001: Attr1: Normal, Inner/Outer WB/WA/RA
			// AP[7:6]=0b00: EL1/2/3 RW, EL0 No Access
	orr	x10,x10,(0b11<<8)	// SH[9:8]=0b11: inner shareable
	orr	x10,x10,(0b1<<10)	// AF[10]=1: access flag
	orr	x10,x10,(0b1<<53)	// PXN[53]=1: EL1/2/3 cannot execute
	orr	x10,x10,(0b1<<54) 	// UXN[54]=1: EL0 cannot execute
	orr	x10,x10,0x00800000	// address = 0x0080_0000
	str	x10,[x9],8	// write block descriptor to table

	// skip 5-254
	add	x9,x9,8*(254-5+1)

	// tt_l2_0_0[255]: block descriptor (EL1 RW), 0x1FE0_0000 - 0x1FFF_FFFF (510 to 512), kernel stack 0
	mov	x10,0b01	// valid bit, block type
	orr	x10,x10,(0b001<<2)	// AttrIdx[4:2]=0b001: Attr1: Normal, Inner/Outer WB/WA/RA
			// AP[7:6]=0b00: EL1/2/3 RW, EL0 No Access
	orr	x10,x10,(0b11<<8)	// SH[9:8]=0b11: inner shareable
	orr	x10,x10,(0b1<<10)	// AF[10]=1: access flag
	orr	x10,x10,(0b1<<53)	// PXN[53]=1: EL1/2/3 cannot execute
	orr	x10,x10,(0b1<<54) 	// UXN[54]=1: EL0 cannot execute
	orr	x10,x10,0x1FE00000	// address = 0x1FE0_0000
	str	x10,[x9],8	// write block descriptor to table

	// tt_l2_0_0[256]: block descriptor (EL0 RO, exec), 0x2000_0000 - 0x201F_FFFF (512 to 514MB), user text/data.ro
	mov	x10,0b01	// valid bit, block type
	orr	x10,x10,(0b001<<2)	// AttrIdx[4:2]=0b001: Attr1: Normal, Inner/Outer WB/WA/RA
	orr	x10,x10,(0b11<<6)	// AP[7:6]=0b11: EL1/2/3 RO, EL0 RO
	orr	x10,x10,(0b11<<8)	// SH[9:8]=0b11: inner shareable
	orr	x10,x10,(0b1<<10)	// AF[10]=1: access flag
			// PXN[53]=0: EL1/2/3 can execute
			// UXN[54]=0: EL0 can execute (EL0 can't access though)
	orr	x10,x10,0x20000000	// address = 0x2000_0000
	str	x10,[x9],8	// write block descriptor to table

	// skip 257-472
	add	x9,x9,8*(472-257+1)

	// tt_l2_0_0[473]: block descriptor (EL0 RW), 0x3B20_0000 - 0x3B3F_FFFF (946 to 948MB), user stack 0
	mov	x10,0b01	// valid bit, block type
	orr	x10,x10,(0b001<<2)	// AttrIdx[4:2]=0b001: Attr1: Normal, Inner/Outer WB/WA/RA
	orr	x10,x10,(0b01<<6)	// AP[7:6]=0b01: EL1/2/3 RW, EL0 RW
	orr	x10,x10,(0b11<<8)	// SH[9:8]=0b11: inner shareable
	orr	x10,x10,(0b1<<10)	// AF[10]=1: access flag
	orr	x10,x10,(0b1<<53)	// PXN[53]=1: EL1/2/3 cannot execute
	orr	x10,x10,(0b1<<54) 	// UXN[54]=1: EL0 cannot execute
	adrp	x11,0x3B200000	// out of range for adr and orr, so use adrp
	orr	x10,x10,x11	// address = 0x3B20_0000
	str	x10,[x9],8	// write block descriptor to table

	// TODO: video core tt_l2_0_0[474-511]
	// tt_l2_0_0[474-511]: block descriptors (EL1 RWnocache), 0x3B40_0000 - 0x3FFF_FFFF (948 to 1024MB), video core

	// TODO: tt_l2_0_3[480-511] (just making full block device-nGnRnE for now)
	// tt_l2_0_3[480-511]: block descriptor (device), 0xFC00_0000 - 0xFFFF_FFFF (4032 to 4096MB), peripherals

	// TODO: this is temporary
	// tt_l1_0[3]: block descriptor, 0xC000_0000 - 0xFFFF_FFFF (3GB to 4GB)
	adr	x9,tt_l1_0	// x9=address of level 1 translation table
	mov	x10,0b01	// valid bit, block type
	orr	x10,x10,(0b010<<2)	// AttrIdx[4:2]=0b010: Attr2: Device-nGnRnE
			// AP[7:6]=0b00: EL1/2/3 RW, EL0 No Access
	orr	x10,x10,(0b11<<8)	// SH[9:8]=0b11: inner shareable
	orr	x10,x10,(0b11<<10)	// AF[10]=1: access flag
	orr	x10,x10,(0b1<<53)	// PXN[53]=1: EL1/2/3 no execute
	orr	x10,x10,(0b1<<54)	// UXN[53]=1: EL0 no execute
	orr	x10,x10,0xC0000000	// address = 0xC000_0000
	str	x10,[x9,24]	// write block descriptor to table

	dsb	sy

	// Set system control register.
	mov	x0,(1<<0)	// M[0]=1: enable the stage 1 MMU
			// A[1]=0: strict alignment checking disabled
	orr	x0,x0,(1<<2)	// C[2]=1: enable data and unified caches
			// SA[3]=0: Stack alignment checking disabled
	orr	x0,x0,(1<<12)	// I[12]=1: enable instruction fetches to allocate into unified caches
	orr	x0,x0,(1<<16)	// nTWI[16]=0b1: allow EL0 to use wfi instruction
	orr	x0,x0,(1<<18)	// nTWE[18]=0b1: allow EL0 to use wfe instruction
			// WXN[19]=0: Write permission does not imply XN
			// EE[25]=0: EL3 data accesses are little endian
	msr	sctlr_el1,x0	// set system control register
	isb

	// MMU is now enabled. Not sure if these nop's are necessary.
	nop
	nop
	nop
	nop

	ldp	fp,lr,[sp],16	// restore frame pointer and link register
	ret

// EL3 Vector Table ////////////////////////////////////////////////////////////////////////////////////////////////////

	.balign	0x800	// vector tables must be 2KB aligned
el3_vector_table:
// Handle synchronous exceptions from the current EL using SP0
el3_curr_el_sp0_sync:
	eret
	.balign	0x80	// each vector must be 128B aligned
// Handle IRQs from the current EL using SP0
el3_curr_el_sp0_irq:
	eret
	.balign	0x80
// Handle FIQs from the current EL using SP0
el3_curr_el_sp0_fiq:
	eret
	.balign	0x80
// Handle system error exceptions from the current EL using SP0
el3_curr_el_sp0_serror:
	eret
	.balign	0x80
// Handle synchronous exceptions from the current EL using SPx
el3_curr_el_spx_sync:
	eret
	.balign	0x80
// Handle IRQs from the current EL using SPx
el3_curr_el_spx_irq:
	eret
	.balign	0x80
// Handle FIQs from the current EL using SPx
el3_curr_el_spx_fiq:
	eret
	.balign	0x80
// Handle system error exceptions from the current EL using SPx
el3_curr_el_spx_serror:
	eret
	.balign	0x80
// Handle synchronous exceptions from a lower EL (AArch64)
el3_lower_el_aarch64_sync:
	eret
	.balign	0x80
// Handle IRQs from a lower EL (AArch64)
el3_lower_el_aarch64_irq:
	eret
	.balign	0x80
// Handle FIQs from a lower EL (AArch64)
el3_lower_el_aarch64_fiq:
	eret
	.balign	0x80
// Handle system error exceptions from a lower EL (AArch64)
el3_lower_el_aarch64_serror:
	eret
	.balign	0x80
// Handle synchronous exceptions from a lower EL (AArch32)
el3_lower_el_aarch32_sync:
	eret
	.balign	0x80
// Handle IRQs from a lower EL (AArch32)
el3_lower_el_aarch32_irq:
	eret
	.balign	0x80
// Handle FIQs from a lower EL (AArch32)
el3_lower_el_aarch32_fiq:
	eret
	.balign	0x80
// Handle system error exceptions from a lower EL (AArch32)
el3_lower_el_aarch32_serror:
	eret

// EL2 Vector Table ////////////////////////////////////////////////////////////////////////////////////////////////////

	.balign	0x800
el2_vector_table:
// Handle synchronous exceptions from the current EL using SP0
el2_curr_el_sp0_sync:
	eret
	.balign	0x80
// Handle IRQs from the current EL using SP0
el2_curr_el_sp0_irq:
	eret
	.balign	0x80
// Handle FIQs from the current EL using SP0
el2_curr_el_sp0_fiq:
	eret
	.balign	0x80
// Handle system error exceptions from the current EL using SP0
el2_curr_el_sp0_serror:
	eret
	.balign	0x80
// Handle synchronous exceptions from the current EL using SPx
el2_curr_el_spx_sync:
	eret
	.balign	0x80
// Handle IRQs from the current EL using SPx
el2_curr_el_spx_irq:
	eret
	.balign	0x80
// Handle FIQs from the current EL using SPx
el2_curr_el_spx_fiq:
	eret
	.balign	0x80
// Handle system error exceptions from the current EL using SPx
el2_curr_el_spx_serror:
	eret
	.balign	0x80
// Handle synchronous exceptions from a lower EL (AArch64)
el2_lower_el_aarch64_sync:
	eret
	.balign	0x80
// Handle IRQs from a lower EL (AArch64)
el2_lower_el_aarch64_irq:
	eret
	.balign	0x80
// Handle FIQs from a lower EL (AArch64)
el2_lower_el_aarch64_fiq:
	eret
	.balign	0x80
// Handle system error exceptions from a lower EL (AArch64)
el2_lower_el_aarch64_serror:
	eret
	.balign	0x80
// Handle synchronous exceptions from a lower EL (AArch32)
el2_lower_el_aarch32_sync:
	eret
	.balign	0x80
// Handle IRQs from a lower EL (AArch32)
el2_lower_el_aarch32_irq:
	eret
	.balign	0x80
// Handle FIQs from a lower EL (AArch32)
el2_lower_el_aarch32_fiq:
	eret
	.balign	0x80
// Handle system error exceptions from a lower EL (AArch32)
el2_lower_el_aarch32_serror:
	eret

// EL1 Vector Table ////////////////////////////////////////////////////////////////////////////////////////////////////

	.balign	0x800
el1_vector_table:
// Handle synchronous exceptions from the current EL using SP0
el1_curr_el_sp0_sync:
	eret
	.balign	0x80
// Handle IRQs from the current EL using SP0
el1_curr_el_sp0_irq:
	eret
	.balign	0x80
// Handle FIQs from the current EL using SP0
el1_curr_el_sp0_fiq:
	eret
	.balign	0x80
// Handle system error exceptions from the current EL using SP0
el1_curr_el_sp0_serror:
	eret
	.balign	0x80
// Handle synchronous exceptions from the current EL using SPx
el1_curr_el_spx_sync:
	eret
	.balign	0x80
// Handle IRQs from the current EL using SPx
el1_curr_el_spx_irq:
	eret
	.balign	0x80
// Handle FIQs from the current EL using SPx
el1_curr_el_spx_fiq:
	eret
	.balign	0x80
// Handle system error exceptions from the current EL using SPx
el1_curr_el_spx_serror:
	eret
	.balign	0x80
// Handle synchronous exceptions from a lower EL (AArch64)
el1_lower_el_aarch64_sync:
	mrs	x9,esr_el1	// read exception syndrome register for EL1
	lsr	x10,x9, 26	// x10=EC[31:26]: exception class
	cmp	x10, 0b010101	// SVC instruction in AArch64 state
	b.eq	route_syscall	// if equal, branch to syscall handler
	eret
	.balign	0x80
// Handle IRQs from a lower EL (AArch64)
el1_lower_el_aarch64_irq:
	eret
	.balign	0x80
// Handle FIQs from a lower EL (AArch64)
el1_lower_el_aarch64_fiq:
	eret
	.balign	0x80
// Handle system error exceptions from a lower EL (AArch64)
el1_lower_el_aarch64_serror:
	eret
	.balign	0x80
// Handle synchronous exceptions from a lower EL (AArch32)
el1_lower_el_aarch32_sync:
	eret
	.balign	0x80
// Handle IRQs from a lower EL (AArch32)
el1_lower_el_aarch32_irq:
	eret
	.balign	0x80
// Handle FIQs from a lower EL (AArch32)
el1_lower_el_aarch32_fiq:
	eret
	.balign	0x80
// Handle system error exceptions from a lower EL (AArch32)
el1_lower_el_aarch32_serror:
	eret

// syscalls ////////////////////////////////////////////////////////////////////////////////////////////////////////////

	.set	SYSCALL_READ,63
	.set	SYSCALL_WRITE,64
	.set	SYSCALL_EXIT,93
	.set	SYSCALL_BRK,214

// NAME
//	route_syscall - route system calls to the appropriate handler
// SYNOPIS
//	void route_syscall(void);
// DESCRIPTION
//	handle_syscall() routes system calls to the appropriate handler.
route_syscall:
	cmp	x8,SYSCALL_WRITE
	b.eq	syscall_write

	adr	x0,unk_syscall_msg
	mov	x1,unk_syscall_msg_size
	bl	write_bytes_pri_uart

	eret

// NAME
//	syscall_write - write to a file descriptor
// SYNOPIS
//	size_or_error syscall_write(int64 const fd, void const * const buf, size const count);
// DESCRIPTION
//	syscall_write() writes count bytes from buf to the file descriptor fd.
// RETURN VALUE
//	On success, the number of bytes written is returned. On error, a negated error code is returned.
syscall_write:
	// TODO: ignore fd for now, always assume stdout
	mov	x0,x1	// data=buf
	mov	x1,x2	// count=count
	bl	write_bytes_pri_uart
	eret

// Kernel Library Functions ////////////////////////////////////////////////////////////////////////////////////////////

// NAME
//	sleep_forever - sleep forever
//
// SYNOPIS
//	void sleep_forever(void);
//
// DESCRIPTION
//	sleep_forever() puts the core to sleep forever.
// RETURN VALUE
//	This procedure does not return.
sleep_forever:
	wfe		// wait for an event
	b	sleep_forever

// NAME
//	write_bytes_pri_uart - write bytes to the primary UART
//
// SYNOPIS
//	size write_bytes_pri_uart(byte const * const data, size const count);
//
// DESCRIPTION
//	write_bytes_pri_uart() writes count bytes to the primary UART. It blocks until count bytes are
//	written to the UART data register.
// RETURN VALUE
//	The number of bytes written is returned. It always returns count.
// NOTES
//	This function must not use the stack since we call it before the stack is initialized.
write_bytes_pri_uart:
	add	x9,x0,x1	// x9 = data + count (byte after last)
.write_b_p_u_loop:	ldrb	w10,[x0],1	// w10 = *data++ (load byte then increment address)
	ldr	x12,pri_uart_dr	// x12 = uart data register adddress
.write_b_p_u_inner:	ldr	w11,[x12,0x18]	// w11 = uart flag register
	tst	w11,0x20	// test uart flag register bit 5 (TXFF) is set
	bne	.write_b_p_u_inner	// if set, wait for it to clear
	strb	w10,[x12]	// store byte in uart data register
	cmp	x0,x9	// check if we are done
	b.ne	.write_b_p_u_loop	// if not done, repeat
	mov	x0,x1	// return count
	ret

// NAME
//	uint64_to_ascii_hex - convert a 64-bit unsigned integer to ASCII hexadecimal
//
// SYNOPIS
//	size uint64_to_ascii_hex(uint64 const value, ascii * const buffer, size const size);
//
// DESCRIPTION
//	uint64_to_ascii_hex() converts a 64-bit unsigned integer to ASCII hexadecimal and stores the result in
//	buffer. It returns the number of characters written to the buffer.
// NOTES
//	This function must not use the stack since we call it before the stack is initialized.

uint64_to_ascii_hex:
	// x0: 64-bit value to convert
	// x1: address of the buffer where the result will be stored
	// x2: size of the buffer (in bytes)
	mov	x9,0	// x9 = 0 (init loop counter)
	mov	x10,x0	// x10 = x0 (copy value to x10)
	mov	x11,x1	// x11 = x1 (copy buffer address to x11)
	add	x12,x1,x2	// x12 = x1 + x2 (byte after last)
.uint64_t_a_h_loop:
	cmp	x11,x12	// check if we are at the end of the buffer
	b.eq	.uint64_t_a_h_done
	cmp	x9,16	// check if we are at the last nibble
	b.eq	.uint64_t_a_h_last

	lsr	x13,x10,60	// x13 = x10 >> 60 (get 4-bit most significant nibble)
	cmp	x11,x1	// check if we are at the beginning of the buffer
	b.ne	.uint64_t_a_h_s	// if not, skip leading zero check
	cbz	x13,.uint64_t_a_h_0	// if x13 == 0, skip leading zeros
.uint64_t_a_h_s:
	add	x13,x13,'0'	// x13 = x13 + '0' (convert to ASCII)
	cmp	x13,'9'	// check if x13 is greater than '9'
	b.le	.uint64_t_a_h_09	// if not, skip adding 7
	add	x13,x13,7	// x13 = x13 + 7 (convert to A-F)
.uint64_t_a_h_09:
	strb	w13,[x11],1	// *x11++ = w13 (store character and increment buffer address)

.uint64_t_a_h_0:
	// end of loop bookkeeping
	lsl	x10,x10,4	// x10 = x10 << 4 (shift left 4 bits)
	add	x9,x9,1	// x9++ (increment loop counter)
	b	.uint64_t_a_h_loop	// repeat loop

.uint64_t_a_h_last:	// if we are at the last nibble and have not written anything yet, write a zero
	cmp	x11,x1	// check if we have written anything yet
	b.ne	.uint64_t_a_h_done
	mov	w13,'0'	// w13 = '0'
	strb	w13,[x11],1	// *x11++ = w13 (store character and increment buffer address)
.uint64_t_a_h_done:
	sub	x0,x11,x1	// x0 = x11 - x1 (return number of characters written)
	ret

// RW Data /////////////////////////////////////////////////////////////////////////////////////////////////////////////

	.balign	0x1000	// 4KB alignment
tt_l1_0:
	.fill 4096 , 1 , 0
	.balign	0x1000	// 4KB alignment
tt_l2_0_0:
	.fill 4096 , 1 , 0
	.balign	0x1000	// 4KB alignment
tt_l2_0_3:
	.fill 4096 , 1 , 0

// RO Data /////////////////////////////////////////////////////////////////////////////////////////////////////////////

// These stack address must match the addresses in the translation tables.
	.balign	8
stack_top_el3_0:	.quad	0x20000000	// 512MB
stack_top_el2_0:	.quad	0x1FFF8000
stack_top_el1_0:	.quad	0x1FFF0000
	.balign	8
stack_top_el0_0:	.quad	0x3B400000	// 948MB (right below video core)
	.balign	8
pri_uart_dr:	.quad	0xFE201000	// primary UART data register (RPi4B)
	.balign	8
init_el3_msg:	.ascii	"INFO: initlizing EL3...\r\n"
	.set	init_el3_msg_size,(. - init_el3_msg)
	.balign	8
init_el3_done_msg:	.ascii	"INFO: EL3 initialization complete\r\n"
	.set	init_el3_done_msg_size,(. - init_el3_done_msg)
	.balign	8
not_el3_msg:	.ascii	"ERROR: not EL3, putting core to sleep"
	.set	not_el3_msg_size,(. - not_el3_msg)
	.balign	8
init_el2_msg:	.ascii	"INFO: initlizing EL2...\r\n"
	.set	init_el2_msg_size,(. - init_el2_msg)
	.balign	8
init_el2_done_msg:	.ascii	"INFO: EL2 initialization complete\r\n"
	.set	init_el2_done_msg_size,(. - init_el2_done_msg)
	.balign	8
not_el2_msg:	.ascii	"ERROR: not EL2, putting core to sleep"
	.set	not_el2_msg_size,(. - not_el2_msg)
	.balign	8
init_el1_msg:	.ascii	"INFO: initlizing EL1...\r\n"
	.set	init_el1_msg_size,(. - init_el1_msg)
	.balign	8
init_el1_done_msg:	.ascii	"INFO: EL1 initialization complete\r\n"
	.set	init_el1_done_msg_size,(. - init_el1_done_msg)
	.balign	8
not_el1_msg:	.ascii	"ERROR: not EL1, putting core to sleep"
	.set	not_el1_msg_size,(. - not_el1_msg)
	.balign	8
init_el0_msg:	.ascii	"INFO: initlizing EL0...\r\n"
	.set	init_el0_msg_size,(. - init_el0_msg)
	.balign	8
init_el0_done_msg:	.ascii	"INFO: EL0 initialization complete\r\n"
	.set	init_el0_done_msg_size,(. - init_el0_done_msg)
	.balign	8
not_el0_msg:	.ascii	"ERROR: not EL0, putting core to sleep"
	.set	not_el0_msg_size,(. - not_el0_msg)
	.balign	8
bad_dtb_msg:	.ascii	"ERROR: no valid device tree found\r\n"
	.set	bad_dtb_msg_size,(. - bad_dtb_msg)
	.balign	8
good_dtb_msg:	.ascii	"INFO: found valid device tree\r\n"
	.set	good_dtb_msg_size,(. - good_dtb_msg)
	.balign	8
not_balign_16_msg:	.ascii	"ERROR: address not 16-byte aligned\r\n"
	.set	not_balign_16_msg_size,(. - not_balign_16_msg)
	.balign	8
unk_syscall_msg:	.ascii	"ERROR: unknown syscall\r\n"
	.set	unk_syscall_msg_size,(. - unk_syscall_msg)
	.balign	8
userspace_addr:	.quad	0x20000000	// dest to copy userspace - must match translation table and must be 16 byte aligned
	.balign	8

// Userspace ///////////////////////////////////////////////////////////////////////////////////////////////////////////
	.set	userspace_size,0x40000	// TODO: 256KB for now

// For ease of initial development, userspace init is concatenated here. Eventually, userspace will be loaded from disk.
// compiled separately and loaded at the appropriate address.
	.balign	16	// userspace_start must be 16-byte aligned
userspace_start:	

// vim: set ts=20:
