.text

.global usb_uart_read
# void usb_uart_read(void *buffer, int bytes, int timeout)
usb_uart_read:
	push	{lr}
	ldr	r3, =0xfff00b65
	blx	r3
	pop	{pc}

.global usb_uart_write
# void usb_uart_write(char *data, int bytes, int timeout)
usb_uart_write:
	push	{lr}
	ldr	r3, =0xfff00b7f
	blx	r3
	pop	{pc}

.global usb_uart_flush
# void usb_uart_flush(void)
usb_uart_flush:
	push	{lr}
	ldr	r3, =0xfff01cb1
	blx	r3
	pop	{pc}
