/**********************************************************************************************************************
init.s

init - Userspace Entrypoint
===========================

init is the entrypoint for userspace. It is called by the kernel after the kernel has initialized the exception levels.

The binary image of init is appended to the kernel image. The kernel copies it to 0x20000000 and jumps to it.
**********************************************************************************************************************/

// Defines /////////////////////////////////////////////////////////////////////////////////////////////////////////////
	.set	SYSCALL_READ,63
	.set	SYSCALL_WRITE,64
	.set	SYSCALL_EXIT,93
	.set	SYSCALL_BRK,214

// Entry point /////////////////////////////////////////////////////////////////////////////////////////////////////////

// NAME
//	init - userspace entrypoint
//
// SYNOPIS
//	void init(void);
//
// DESCRIPTION
//	init() is the entrypoint for userspace. It is called by the kernel after the kernel has initialized
//	the exception levels.
// RETURN VALUE
//	Does not return.
init:

	mov	x0,1	// file descriptor
	adr	x1,begin_init_msg	// buffer
	mov	x2,begin_init_msg_size	// size
	bl	syscall_write
	cmp	x0,0
	b.lt	.init_write_error

	// test read
	mov	x0,1	// file descriptor
	adr	x1,press_key_msg	// buffer
	mov	x2,press_key_msg_size	// size
	bl	syscall_write
	cmp	x0,0
	b.lt	.init_write_error

	mov	x0,0	// file descriptor
	mov	x1,0x20200000	// buffer
	mov	x2,1	// size
	bl	syscall_read

	mov	x0,1	// file descriptor
	adr	x1,you_pressed_msg	// buffer
	mov	x2,you_pressed_msg_size	// size
	bl	syscall_write
	cmp	x0,0
	b.lt	.init_write_error

	mov	x0,1	// file descriptor
	mov	x1,0x20200000	// buffer
	mov	x2,2	// size
	bl	syscall_write

	// test syscall_brk
	mov	x0,0	// bad address
	bl	syscall_brk
	add	x0,x0,0x1000	// add 4k
	bl	syscall_brk
	mov	x9,x0	// save brk

	b	.init_loop
.init_write_error:
	b	.	// no point logging if we can't write

.init_loop:
	mov	x0,0x3	// just to test
	mov	x1,0x4	// just to test
	wfe		// wait for an event
	b	.init_loop

// Standard Library ////////////////////////////////////////////////////////////////////////////////////////////////////

// NAME
//	syscall_read - read from a file descriptor
//
// SYNOPIS
//	size_or_error syscall_read(int64 const fd, void const * const buffer, size const count);
//
// DESCRIPTION
//	syscall_read() reads count bytes from file descriptor fd to buffer.
// RETURN VALUE
//	On success, the number of bytes read is returned. On error, a negated error code is returned.
syscall_read:
	stp	fp,lr,[sp,-16]!	// save frame pointer and link register
			// x0 = fd (set by caller)
			// x1 = buffer (set by caller)
			// x2 = count (set by caller)
	mov	w8,SYSCALL_READ
	svc	0	// make system call
	ldp	x29,x30,[sp],16	// restore frame pointer and link register
	ret

// NAME
//	syscall_write - write to a file descriptor
//
// SYNOPIS
//	size_or_error syscall_write(int64 const fd, void const * const buffer, size const count);
//
// DESCRIPTION
//	syscall_write() writes count bytes from buffer to the file descriptor fd.
// RETURN VALUE
//	On success, the number of bytes written is returned. On error, a negated error code is returned.
syscall_write:
	stp	fp,lr,[sp,-16]!	// save frame pointer and link register
			// x0 = fd (set by caller)
			// x1 = buffer (set by caller)
			// x2 = count (set by caller)
	mov	w8,SYSCALL_WRITE
	svc	0	// make system call
	ldp	x29,x30,[sp],16	// restore frame pointer and link register
	ret

// NAME
//	syscall_brk - set the program break
//
// SYNOPIS
//	void * syscall_brk(void const * const addr);
//
// DESCRIPTION
//	syscall_brk() sets the program break to addr.
// RETURN VALUE
//	On success, the previous program break is returned. On error, the current program break is returned.
syscall_brk:
	stp	fp,lr,[sp,-16]!	// save frame pointer and link register
			// x0 = addr (set by caller)
	mov	w8,SYSCALL_BRK
	svc	0	// make system call
	ldp	x29,x30,[sp],16	// restore frame pointer and link register
	ret

// RO Data /////////////////////////////////////////////////////////////////////////////////////////////////////////////

	.balign	8
begin_init_msg:	.ascii	"INFO: initializing userspace...\r\n"
	.set	begin_init_msg_size,(. - begin_init_msg)
	.balign	8
write_error_msg:	.ascii	"ERROR: syscall_write\r\n"
	.set	write_error_msg_size,(. - write_error_msg)
	.balign	8
press_key_msg:	.ascii	"press any key to continue...\r\n"
	.set	press_key_msg_size,(. - press_key_msg)
you_pressed_msg:	.ascii	"you pressed: "
	.set	you_pressed_msg_size,(. - you_pressed_msg)

