// kernel.s
//
// An AArch64 kernel for Raspberry Pi 4 Model B (RPi4B).
//
// This program must be compiled and the binary dumped and loaded at address 0x80000 which is where the RPi4B
// bootloader expects a kernel. No linking is required.
//
// References:
// - ARM Cortex-A72 MPCore Processor Technical Reference Manual: https://developer.arm.com/documentation/100095/0003/
//
// Notes:
// - use 4 byte alignment for all functions
// - not using bss section since we are not using C so no need to zero out memory
//
// Use simple code to allow bootstrapping with a minimal assembler:
// - no linking
// - no macros
// - no sections
// - minimal number of mnemonics and directives
// - no expressions (using them for string lengths for now, but can replace with constants)
// - no local labels or local symbol names
// - no pseudo-ops like ldr x1,=label, manually create literal pools when needed

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

	// Setup EL3 and prepare to jump to EL2.
	ldr	x9,stack_top_el3_0
	mov	sp,x9	// set EL3 stack pointer (early as possible so can safely do nested bl)

	adr	x9,el3_vector_table
	msr	vbar_el3,x9	// set EL3 vector base address

	msr	sctlr_el2,xzr	// set EL2 system control register to safe defaults
	msr	hcr_el2,xzr	// set EL2 hypervisor configuration register to safe defaults

	mrs	x9,scr_el3	// read EL3secure configuration register
	orr	x9,x9,0b10000000000	// RW (bit 10): set next lower EL (EL2) execution state to AArch64
	orr	x9,x9,0b1	// NS (bit 1): set EL1 and EL2 security state to non-secure
	msr	scr_el3, x9	// set secure configuration register

	mov	x9,0b01001	// DAIF=0000, M[4]=0 (AArch64), M[3:0]=1001 (EL2h: EL2 with SP_EL2)
	msr	spsr_el3, x9	// set EL3 saved program status register

	adr	x9,init_el2
	msr	elr_el3,x9	// set EL3 exception link register to start EL2 at init_el2

	// Check that we have a device tree where we expect it.
	bl	verify_device_tree	// verify device tree magic bytes
	cmp	x0,ERROR_NONE	// check if device tree is valid
	b.eq	.init_el3_done	// if valid, continue
	bl	sleep_forever	// else, sleep forever (verify_device_tree logs error)

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


	// Setup EL2 and prepare to jump to EL1.
	ldr	x9,stack_top_el2_0
	mov	sp,x9	// set EL2 stack pointer

	adr	x9,el2_vector_table
	msr	vbar_el2,x9	// set EL2 vector base address

	msr	sctlr_el1,xzr	// set EL1 system control register to safe defaults

	mrs	x9,hcr_el2	// read hypervisor configuration register
	orr	x9,x9,0x80000000	// RW (bit 31): set EL1 execution state to AArch64
	msr	hcr_el2,x9	// write hypervisor configuration register

	mov	x9,0b00101	// DAIF=0000, M[4]=0 (AArch64), M[3:0]=0101 (EL1h: EL1 with SP_EL1)
	msr	spsr_el2,x9	// set EL2 saved program status register

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


	// Setup EL1 and prepare to jump to EL0.
	ldr	x9,stack_top_el1_0
	mov	sp,x9	// set EL1 stack pointer

	adr	x9,el1_vector_table
	msr	vbar_el1,x9	// set EL1 vector base address

	ldr	x9,stack_top_el0_0
	msr	sp_EL0,x9	// set EL0 stack pointer (cannot do it in EL0 so have to do here)

	mov	x9,0x10000	// (bit 16 = 1): allow EL0 to use wfi instruction
	orr	x9,x9,0x40000	// (bit 18 = 1): allow EL0 to use wfe instruction
	msr	sctlr_el1,x9	// set EL1 system control register

	mov	x9,0b00000	// DAIF=0000, M[4]=0 (AArch64), M[3:0]=0000 (EL0: EL0 with SP_EL0)
	msr	spsr_el1,x9	// set EL1 saved program status register

	adr	x9,init_el0
	msr	elr_el1,x9	// set EL1 exception link register to start EL0 at init_el0

.init_el1_done:	// log EL1 initialization complete
	adr	x0,init_el1_done_msg
	mov	x1,init_el1_done_msg_size
	bl	write_bytes_pri_uart

	eret		// return to EL0

// NAME
//	init_el0 - initialize exception level 0 (EL0)
//
// SYNOPIS
//	void init_el0(void);
//
// DESCRIPTION
//	init_el0() initializes exception level 0 (EL0) then branches to userspace.
// RETURN VALUE
//	Does not return.
init_el0:
	// Cannot make sure we are in EL0 b/c cannot read currentel in EL0

	// Log that we are initializing EL0.
	adr	x0,init_el0_msg
	mov	x1,init_el0_msg_size
	bl	write_bytes_pri_uart

	// log EL0 initialization complete
	adr	x0,init_el0_done_msg
	mov	x1,init_el0_done_msg_size
	bl	write_bytes_pri_uart

	// Start userspace
	ldr	x9,userspace_addr	// x0 = userspace entrypoint
	br	x9	// branch to userspace entrypoint


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
stack_top_el0_0:	.quad	0x1FFF8000	// 8KB below
stack_top_el0_1:	.quad	0x1FFE6000	// 8KB below
stack_top_el0_2:	.quad	0x1FFE4000	// 8KB below
stack_top_el0_3:	.quad	0x1FFE2000	// 8KB below
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
userspace_addr:	.quad	0x88000	// userspace entrypoint: ultimately this will be something like 0x1000000 (16MB)

// Userspace ///////////////////////////////////////////////////////////////////////////////////////////////////////////

// For ease of initial development, we will put the userspace code right after the kernel code. Ultimately it will be
// compiled separately and loaded at userspace_addr which will be set to something like 0x1000000 (16MB).
	.org	0x8000	// 32KB - make sure page aligned (4KB alignment) and update userspace_addr

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
	bl	sleep_forever	// sleep forever


// vim: set ts=20:
