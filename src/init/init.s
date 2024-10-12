/**********************************************************************************************************************
init.s

init - Userspace Entrypoint
===========================

init is the entrypoint for userspace. It is called by the kernel after the kernel has initialized the exception levels.

The binary image of init is appended to the kernel image. The kernel copies it to 0x20000000 and jumps to it.
**********************************************************************************************************************/

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
	// TODO: remove
.init_loop:
	mov	x0,0x3	// just to test
	nop
	nop
	mov	x1,0x4	// just to test
	wfe		// wait for an event
	b	.init_loop
// TODO: if we have any RW data for userspace it needs to be copied to a RW block from the kernel translation table.
