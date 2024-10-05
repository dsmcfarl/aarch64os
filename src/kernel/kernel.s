// kernel.s
//
// An AArch64 kernel for Raspberry Pi 4 Model B (RPi4B).
//
// To build, first assemble:
//
//    aarch64-none-elf-as -g -c kernel.s -o kernel.o
//
// Then extract just the binary without the ELF header:
//
//    aarch64-none-elf-objcopy -O binary kernel.o kernel.bin
//
// No linking is required.
//
// Then prepare a FAT32 formatted SD card. The following are the minimum required files on the SD card. All except
// kernel.bin and config.txt are from the Raspberry Pi firmware repository:
//
// - kernel.bin
// - config.txt
// - bcm2711-rpi-4-b.dtb
// - fixup4.dat
// - start4.elf
// - overlays/disable-bt.dtbo
//
// The contents of config.txt must be:
//
//    # Defaults to loading kernel8.img if not specified. Better to be explicit.
//    kernel=kernel.bin
//
//    # load the kernel to the memory address 0x0. This disables the armstub and allows booting into Exception level 3
//    # with all cores running.
//    kernel_old=1
//
//    # Set the ARM architecture to 64-bit. This causes the kernel to be relocated to 0x80000.
//    # If kernel= is not set and we use the default kernel name of kernel8.img, we don't have to set this as it
//    # relocates the kernel to 0x80000 anyway.
//    arm_64bit=1
//
//    # Set the disable_commandline_tags command to 1 to stop start4.elf from filling in ATAGS (memory from 0x100)
//    # before launching the kernel.
//    # Boot fails with kernel_old=1 if this is not set because it overwrites the kernel.
//    disable_commandline_tags=1
//
//    # Specifies how much memory, in megabytes, to reserve for the exclusive use of the GPU
//    # 76MB is the default if more than 1GB of RAM is installed. Setting explicitly to 76MB to avoid confusion.
//    # The top of the video core memory is at 0x0_4000_0000 (1024 MiB) so setting this to 76 means the video core
//    # memory will start at 0x0_3B40_0000 (948 MiB).
//    gpu_mem=76
//
//    # Set the device tree address. By default it loads at 0x100 which was being overwritten by the kernel when
//    # kernel_old=1.
//    # Setting to much higher address like 0x3AC00000 (just below video core memory) results in the following error:
//    # "dterror: not a valid FDT - err -9"
//    device_tree_address=0x20000000 # 512 MiB
//
//    # disable bluetooth so that UART0 (PL011) can be the primary UART. Otherwise, the mini UART is the primary UART
//    # and it is not as full-featured as the PL011.
//    dtoverlay=disable-bt
//
//    # log start4.elf debug output to UART
//    uart_2ndstage=1
//
//    # Disable pull downs (required for JTAG)
//    gpio=22-27=np
//
//    # Enable JTAG pins (i.e. GPIO22-GPIO27)
//    enable_jtag_gpio=1
//
// For remote debugging with a JTAG debug probe and GDB, it is useful to have an ELF file with the start address
// changed to 0x80000 which is where the 64-bit Arm core expects to find the kernel. This file can be used with
// the GDB load command:
//
//     aarch64-none-elf-objcopy -O elf64-littleaarch64 --change-address 0x80000 kernel.o kernel.elf
//
// References:
// 1. ARM Cortex-A72 MPCore Processor Technical Reference Manual: https://developer.arm.com/documentation/100095/0003/
// 2. Arm Architecture Reference Manual for A-profile architecture: https://developer.arm.com/documentation/ddi0487/latest/
// 3. AArch64 memory management examples: https://developer.arm.com/documentation/102416/0100
//
// Notes:
// - use 4 byte alignment for all functions
// - not using bss section since we are not using C so no need to zero out memory
// - use `bl` instead of `b` even for 'no return' functions so the link register gets set for debugging
//
// Use simple code to allow bootstrapping with a minimal assembler:
// - no linking
// - no macros
// - no sections
// - minimal number of mnemonics and directives
// - no expressions (using them for string lengths and similar for now, but can replace with constants)
// - no local labels or local symbol names
// - no pseudo-ops like ldr x1,=label, manually create literal pools when needed
//
// Register Usage:
// - x0-x7: argument registers
// - x8: indirect result location register and used for syscall number
// - x9-x15: temporary registers
// - x16-x17: intra-procedure-call temporary registers (can be used by linker generated call veneers and Procedure
//            Linkage Table code). Not planning to use these, but avoid using them in case they are needed.
// - x18: platform register (not currently using this for anything, but avoid this register in case it is needed)
// - x19-x28: callee-saved registers
// - x29 (fp): frame pointer
// - x30 (lr): link register
// - x31 (xzr): zero register

// Constants ////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

	// Append the exception level to the message.
	mov	x0,x19	// x0 = exception level
	adr	x1,buffer	// buffer to store exception level
	mov	x2,buffer_size	// size of buffer
	bl	uint64_to_ascii_hex	// convert exception level to ASCII hexadecimal
	mov	x1,x0	// x1 = number of characters written
	adr	x0,buffer	// buffer
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

	// Append the exception level to the message.
	mov	x0,x19	// x0 = exception level
	adr	x1,buffer	// buffer to store exception level
	mov	x2,buffer_size	// size of buffer
	bl	uint64_to_ascii_hex	// convert exception level to ASCII hexadecimal
	mov	x1,x0	// x1 = number of characters written
	adr	x0,buffer	// buffer
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

	// Append the exception level to the message.
	mov	x0,x19	// x0 = exception level
	adr	x1,buffer	// buffer to store exception level
	mov	x2,buffer_size	// size of buffer
	bl	uint64_to_ascii_hex	// convert exception level to ASCII hexadecimal
	mov	x1,x0	// x1 = number of characters written
	adr	x0,buffer	// buffer
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
//	copy_balign_16 - copy 16-bit aligned bytes
//
// SYNOPIS
//	error copy_balign_16(void *src, void *dst, size size);
//
// DESCRIPTION
//	copy_balign_16() copies 16-bit aligned bytes from src to dst for size bytes.
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
	stp    fp,lr,[sp,-16]!	// save frame pointer and link register

	// Set the Base address
	// ---------------------
	adr      x0, tt_l1_base                  // Get address of level 1 for TTBR0_EL1
	MSR      TTBR0_EL1, x0


	// Set up memory attributes
	// -------------------------
	// This equates to:
	// 0 = b01000100 = Normal, Inner/Outer Non-Cacheable
	// 1 = b11111111 = Normal, Inner/Outer WB/WA/RA
	// 2 = b00000000 = Device-nGnRnE
	MOV      x0, #0x000000000000FF44
	MSR      MAIR_EL1, x0


	// Set up TCR_EL1
	// ---------------
	MOV      x0, #0x19                        // T0SZ=0b011001 Limits VA space to 39 bits, translation starts @ l1
	ORR      x0, x0, #(0x1 << 8)              // IGRN0=0b01    Walks to TTBR0 are Inner WB/WA
	ORR      x0, x0, #(0x1 << 10)             // OGRN0=0b01    Walks to TTBR0 are Outer WB/WA
	ORR      x0, x0, #(0x3 << 12)             // SH0=0b11      Inner Shareable
	ORR      x0, x0, #(0x1 << 23)             // EPD1=0b1      Disable table walks from TTBR1
		    // TBI0=0b0
		    // TG0=0b00      4KB granule for TTBR0 (Note: Different encoding to TG0)
		    // A1=0          TTBR0 contains the ASID
		    // AS=0          8-bit ASID
		    // IPS=0         32-bit IPA space
	MSR      TCR_EL1, x0

	// NOTE: We don't need to set up T1SZ/TBI1/ORGN1/IRGN1/SH1, as we've set EPD==1 (disabling walks from TTBR1)


	// Ensure changes to system register are visible before MMU enabled
	ISB


	// Invalidate TLBs
	// ----------------
	TLBI     VMALLE1
	DSB      SY
	ISB


	// Generate Translation Table
	// ---------------------------
	// First fill table with faults
	// NOTE: The way the space for the tables is reserved pre-fills it with zeros
	// When loading the image into a simulation this saves time.  On real hardware
	// you would want this zeroing loop.
	//  LDR      x1, =tt_l1_base                   // Address of L1 table
	//  MOV      w2, #512                          // Number of entries
	//1:
	//  STP      xzr, xzr, [x1], #16               // 0x0 (Fault) into table entries
	//  SUB      w2, w2, #2                        // Decrement count by 2, as we are writing two entries at once
	//  CBNZ     w2, 1b


	//
	// Very basic translation table
	//
	adr      x1, tt_l1_base                   // Address of L1 table

	// TT block entries templates   (L1 and L2, NOT L3)
	// Assuming table contents:
	// 0 = b01000100 = Normal, Inner/Outer Non-Cacheable
	// 1 = b11111111 = Normal, Inner/Outer WB/WA/RA
	// 2 = b00000000 = Device-nGnRnE
	.equ TT_S1_FAULT,           0x0
	.equ TT_S1_NORMAL_NO_CACHE, 0x00000000000000401    // Index = 0, AF=1
	.equ TT_S1_NORMAL_WBWA,     0x00000000000000405    // Index = 1, AF=1
	.equ TT_S1_DEVICE_nGnRnE,   0x00600000000000409    // Index = 2, AF=1, PXN=1, UXN=1
	.equ TT_S1_NON_SHARED,      (0 << 8)               // Non-shareable
	.equ TT_S1_INNER_SHARED,    (3 << 8)               // Inner-shareable
	.equ TT_S1_OUTER_SHARED,    (2 << 8)               // Outer-shareable
	.equ TT_S1_PRIV_RW,         (0x0)
	.equ TT_S1_PRIV_RO,         (0x2 << 6)
	.equ TT_S1_USER_RW,         (0x1 << 6)
	.equ TT_S1_USER_RO,         (0x3 << 6)
	// [0]: 0x0000,0000 - 0x3FFF,FFFF
	LDR      x0, =TT_S1_NORMAL_WBWA            // Entry template
	ORR      x0, x0, #TT_S1_INNER_SHARED       // 'OR' with inner-shareable attribute
		     // Don't need to OR in address, as it is 0
		     // AP=0b00, EL1 RW, EL0 No Access
	STR      x0, [x1]

	// [1]: 0x4000,0000 - 0x7FFF,FFFF
	LDR      x0, =TT_S1_NORMAL_WBWA            // Entry template
	ORR      x0, x0, #TT_S1_INNER_SHARED       // 'OR' with inner-shareable attribute
	ORR      x0, x0, #TT_S1_USER_RO
	ORR      x0, x0, #0x40000000               // 'OR' template with base physical address
		     // AP=0b00, EL1 RW, EL0 No Access
	STR      x0, [x1, #8]
	// [1]: 0x8000,0000 - 0xBFFF,FFFF
	LDR      x0, =TT_S1_DEVICE_nGnRnE          // Entry template
	ORR      x0, x0, #0x80000000               // 'OR' template with base physical address
		     // AP=0b00, EL1 RW, EL0 No Access
	STR      x0, [x1, #16]
	// [1]: 0xC000,0000 - 0xFFFF,FFFF
	LDR      x0, =TT_S1_DEVICE_nGnRnE          // Entry template
	ORR      x0, x0, #0xC0000000               // 'OR' template with base physical address
		     // AP=0b00, EL1 RW, EL0 No Access
	STR      x0, [x1, #24]

	DSB      SY


	// Enable MMU
	// -----------
	MOV      x0, #(1 << 0)                     // M=1           Enable the stage 1 MMU
	ORR      x0, x0, #(1 << 2)                 // C=1           Enable data and unified caches
	ORR      x0, x0, #(1 << 12)                // I=1           Enable instruction fetches to allocate into unified caches
		     // A=0           Strict alignment checking disabled
		     // SA=0          Stack alignment checking disabled
		     // WXN=0         Write permission does not imply XN
		     // EE=0          EL3 data accesses are little endian
	orr	x0,x0,0x10000	// (bit 16 = 1): allow EL0 to use wfi instruction
	orr	x0,x0,0x40000	// (bit 18 = 1): allow EL0 to use wfe instruction

	MSR      SCTLR_EL1, x0
	ISB

	//
	// MMU is now enabled
	//

	NOP
	NOP
	NOP
	NOP

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
//	void write_bytes_pri_uart(byte const * const data, size const count);
//
// DESCRIPTION
//	write_bytes_pri_uart() writes count bytes to the primary UART. It blocks until count bytes are
//	written to the UART data register.
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
tt_l1_base:
  .fill 4096 , 1 , 0

	.balign	8
buffer:	.skip	16
	.set	buffer_size,(. - buffer)


// RO Data /////////////////////////////////////////////////////////////////////////////////////////////////////////////

	.balign	8
stack_top_el3_0:	.quad	0x20000000	// 512MB
stack_top_el3_1:	.quad	0x1FFFE000	// 8KB below
stack_top_el3_2:	.quad	0x1FFFC000	// 8KB below
stack_top_el3_3:	.quad	0x1FFFA000	// 8KB below
stack_top_el2_0:	.quad	0x1FFF8000	// 8KB below
stack_top_el2_1:	.quad	0x1FFF6000	// 8KB below
stack_top_el2_2:	.quad	0x1FFF4000	// 8KB below
stack_top_el2_3:	.quad	0x1FFF2000	// 8KB below
stack_top_el1_0:	.quad	0x1FFF0000	// 8KB below
stack_top_el1_1:	.quad	0x1FFEE000	// 8KB below
stack_top_el1_2:	.quad	0x1FFEC000	// 8KB below
stack_top_el1_3:	.quad	0x1FFEA000	// 8KB below

stack_top_el0_0:	.quad	0x60000000	// 1.5GB
stack_top_el0_1:	.quad	0x5FFE6000	// 8KB below
stack_top_el0_2:	.quad	0x5FFE4000	// 8KB below
stack_top_el0_3:	.quad	0x5FFE2000	// 8KB below
	.balign	8
	.balign	8
pri_uart_dr:	.quad	0xFE201000	// primary UART data register (RPi4B)
	.balign	8
init_el3_msg:	.ascii	"INFO: initlizing EL3...\r\n"
	.set	init_el3_msg_size,(. - init_el3_msg)
	.balign	8
init_el3_done_msg:	.ascii	"INFO: EL3 initialization complete\r\n"
	.set	init_el3_done_msg_size,(. - init_el3_done_msg)
	.balign	8
not_el3_msg:	.ascii	"ERROR: not EL3, putting core to sleep: EL="
	.set	not_el3_msg_size,(. - not_el3_msg)
	.balign	8
init_el2_msg:	.ascii	"INFO: initlizing EL2...\r\n"
	.set	init_el2_msg_size,(. - init_el2_msg)
	.balign	8
init_el2_done_msg:	.ascii	"INFO: EL2 initialization complete\r\n"
	.set	init_el2_done_msg_size,(. - init_el2_done_msg)
	.balign	8
not_el2_msg:	.ascii	"ERROR: not EL2, putting core to sleep: EL="
	.set	not_el2_msg_size,(. - not_el2_msg)
	.balign	8
init_el1_msg:	.ascii	"INFO: initlizing EL1...\r\n"
	.set	init_el1_msg_size,(. - init_el1_msg)
	.balign	8
init_el1_done_msg:	.ascii	"INFO: EL1 initialization complete\r\n"
	.set	init_el1_done_msg_size,(. - init_el1_done_msg)
	.balign	8
not_el1_msg:	.ascii	"ERROR: not EL1, putting core to sleep: EL="
	.set	not_el1_msg_size,(. - not_el1_msg)
	.balign	8
init_el0_msg:	.ascii	"INFO: initlizing EL0...\r\n"
	.set	init_el0_msg_size,(. - init_el0_msg)
	.balign	8
init_el0_done_msg:	.ascii	"INFO: EL0 initialization complete\r\n"
	.set	init_el0_done_msg_size,(. - init_el0_done_msg)
	.balign	8
not_el0_msg:	.ascii	"ERROR: not EL0, putting core to sleep: EL="
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
userspace_addr:	.quad	0x40000000	// dest to copy userspace - must be 16 byte aligned
	.balign	8

// Userspace ///////////////////////////////////////////////////////////////////////////////////////////////////////////

// For ease of initial development, we will put the userspace code right after the kernel code then copy it.. Ultimately it will be
// compiled separately and loaded at the appropriate address.
	.balign	16	// userspace_start must be 16-byte aligned
userspace_start:	

// NAME
//	init - userspace entrypoint
//
// SYNOPIS
//	void init(void);
//
// DESCRIPTION
//	init() is the entrypoint for userspace. It is called by the kernel after the kernel has initialized the exception levels.
// RETURN VALUE
//	Does not return.
init:
	// TODO: remove
.init_loop:
	mov x0, 0x3
	nop
	nop
	mov x1, 0x4
	wfe	// wait for an event
	b	.init_loop

	.balign	16	// userspace_size must be 16-byte aligned
	.set	userspace_size,(. - userspace_start)

// vim: set ts=20:
