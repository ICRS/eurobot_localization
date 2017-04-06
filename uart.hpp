#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>


int uart_set_interface_attribs (int fd, int speed, int parity);
void uart_set_blocking (int fd, int should_block);
