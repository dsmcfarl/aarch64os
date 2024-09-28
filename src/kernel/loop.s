// loop.s
//
// An AArch64 kernel for Raspberry Pi 4 Model B (RPi4B).
//
// This just loops for ever so that a remote debugger can attach and load a real kernel; it must be compiled and
// the binary dumped and loaded at address 0x80000 which is where the RPi4B bootloader expects a kernel. No linking
// is required.

sleep_forever:
	wfe		// wait for an event
	b	sleep_forever
// vim: set ts=20:
