/*
 * arch/arm/mach-s5pv210/aries.h
 */

#ifndef __P1_H__
#define __P1_H__

struct uart_port;

void p1_bt_uart_wake_peer(struct uart_port *port);
extern void s3c_setup_uart_cfg_gpio(unsigned char port);

#endif
