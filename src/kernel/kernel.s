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
	// verify device tree magic bytes
	// device tree address defined in config.txt: device_tree_address=0x20000000
	// must contain 0xD00DFEED (big-endian), 0xEDFE0DD0 (little-endian) in first 4 bytes
	mov	x1,0x20000000	// address of device tree

.init_el3_dtb:	ldrb	w0,[x1],1	// read first byte of device tree and increment address
	cmp	x0,0xD0	// check if first byte is 0xD0
	b.ne	.init_el3_bad_dtb	// if not, log
	ldrb	w0,[x1],1	// read second byte of device tree and increment address
	cmp	x0,0x0D	// check if second byte is 0x0D
	b.ne	.init_el3_bad_dtb	// if not, log
	ldrb	w0,[x1],1	// read third byte of device tree and increment address
	cmp	x0,0xFE	// check if third byte is 0xFE
	b.ne	.init_el3_bad_dtb	// if not, log
	ldrb	w0,[x1],1	// read fourth byte of device tree and increment address
	cmp	x0,0xED	// check if fourth byte is 0xED
	b.ne	.init_el3_bad_dtb	// if not, log
	b	.init_el3_good_dtb	// if all bytes are correct, continue

.init_el3_try_qemu:	// try QEMU address for dtb
	mov	x1,0	// address of device tree for qemu
	b	.init_el3_dtb	// repeat device tree check


.init_el3_bad_dtb:	// log that we found an invalid device tree and sleep
	cmp	x1,0	// check which address we were reading from
	b.ne	.init_el3_try_qemu	// try QEMU address for dtb
	adr	x0,bad_dtb_msg
	mov	x1,bad_dtb_msg_size
	bl	write_bytes_pri_uart
	b	sleep_forever

.init_el3_good_dtb:	// log that we found a valid device tree
	adr	x0,good_dtb_msg
	mov	x1,good_dtb_msg_size
	bl	write_bytes_pri_uart

.init_el3_check_el:	// check exception level
	mrs	x19,currentel	// read current exception level
	lsr	x19,x19,2	// shift right to get EL
	and	x19,x19,0x3	// mask bits 0-1
	cmp	x19,3	// check if we are at EL3
	b.eq	.init_el3_good_el	// if we are at EL3, continue

.init_el3_bad_el:	// log an error message
	adr	x0,not_el3_msg
	mov	x1,not_el3_msg_size
	bl	write_bytes_pri_uart

	// append the exception level to the message
	mov	x0,x19	// x0 = exception level
	adr	x1,buffer	// buffer to store exception level
	mov	x2,buffer_size	// size of buffer
	bl	uint64_to_ascii_hex	// convert exception level to ASCII hexadecimal
	mov	x1,x0	// x1 = number of characters written
	adr	x0,buffer	// buffer
	bl	write_bytes_pri_uart

	// put the core to sleep
	b	sleep_forever

.init_el3_good_el:	// Log a message
	adr	x0,init_el3_msg
	mov	x1,init_el3_msg_size
	bl	write_bytes_pri_uart

	// TODO: initialize EL3 and branch to init_el2

	b	sleep_forever
	b	.	// should never reach here

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
pri_uart_dr:	.quad	0xFE201000	// primary UART data register (RPi4B)

	.balign	8
init_el3_msg:	.ascii	"init EL3\n"
	.set	init_el3_msg_size,(. - init_el3_msg)

	.balign	8
bad_dtb_msg:	.ascii	"ERROR: no valid device tree found\n"
	.set	bad_dtb_msg_size,(. - bad_dtb_msg)

	.balign	8
good_dtb_msg:	.ascii	"INFO: found valid device tree \n"
	.set	good_dtb_msg_size,(. - good_dtb_msg)

	.balign	8
not_el3_msg:	.ascii	"ERROR: not EL3, putting core to sleep: EL="
	.set	not_el3_msg_size,(. - not_el3_msg)
// vim: set ts=20:
