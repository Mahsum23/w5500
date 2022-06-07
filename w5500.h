#ifndef __W5500__
#define __W5500__

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "string.h"
#include "stdint.h"

// PINS AND PORTS
#define w5500_PORT GPIOA
#define W5500_CS GPIO_PIN_4
#define INT_PORT GPIOD
#define INT_PIN GPIO_PIN_3
#define RESET_PORT GPIOE
#define RESET_PIN GPIO_PIN_11
#define W5500_SELECT() HAL_GPIO_WritePin(w5500_PORT, W5500_CS, GPIO_PIN_RESET)
#define W5500_UNSELECT() HAL_GPIO_WritePin(w5500_PORT, W5500_CS, GPIO_PIN_SET)

// W5500 REGISTERS
#define SIMR 0x0018
#define RWB_READ 0
#define RWB_WRITE 1
#define OM_NB 0x00
#define OM_1B 0x01
#define OM_2B 0x02
#define OM_4B 0x03
#define BSB_COMMON 0x00
#define BSB_S0 0x01
#define BSB_S0_TX 0x02
#define BSB_S0_RX 0x03
#define Sn_MR 0x0000
#define Sn_SR 0x0003
#define Sn_IR 0x0002 // Interrupt  (write 1 to clean)
#define Sn_CR 0x0001
#define Sn_DIPR0 0x000C
#define Sn_DIPR1 0x000D
#define Sn_DIPR2 0x000E
#define Sn_DIPR3 0x000F
#define Sn_DPORT0 0x0010
#define Sn_DPORT1 0x0011
#define Sn_RXBUF_SIZE 0x001E // size of RX buffer
#define Sn_TXBUF_SIZE 0x001F // size of TX buffer
#define Sn_TX_FSR0 0x0020 // TX free size
#define Sn_TX_FSR1 0x0021 // TX free size
#define Sn_TX_RD0 0x0022 // TX buffer read pointer
#define Sn_TX_RD1 0x0023 // TX buffer read pointer
#define Sn_TX_WR0 0x0024 // TX buffer write pointer
#define Sn_TX_WR1 0x0025 // TX buffer write pointer
#define Sn_RX_RSR0 0x0026 // RX buffer received size
#define Sn_RX_RSR1 0x0027 // RX buffer received size
#define Sn_RX_RD0 0x0028 // RX read pointer
#define Sn_RX_RD1 0x0029 // RX read pointer
#define Sn_RX_WR0 0x002A // RX write pointer
#define Sn_RX_WR1 0x002B // RX write pointer
#define Sn_IMR 0x002C // Interrupt Mask


// MAC
#define SHAR0 0x0009
#define SHAR1 0x000A
#define SHAR2 0x000B
#define SHAR3 0x000C
#define SHAR4 0x000D
#define SHAR5 0x000E
// IP
#define GAR0 0x0001
#define GAR1 0x0002
#define GAR2 0x0003
#define GAR3 0x0004
#define SUBR0 0x0005
#define SUBR1 0x0006
#define SUBR2 0x0007
#define SUBR3 0x0008
#define SIPR0 0x000F
#define SIPR1 0x0010
#define SIPR2 0x0011
#define SIPR3 0x0012
// Sockets ports
#define Sn_PORT0 0x0004
#define Sn_PORT1 0x0005
// Socket status
#define SOCK_CLOSED 0x00
#define SOCK_INIT 0x13
#define SOCK_LISTEN 0x14
#define SOCK_ESTABLISHED 0x17
#define SOCK_CLOSE_WAIT 0x1C
#define SOCK_UDP 0x22
#define SOCK_MACRAW 0x42
// Socket modes
#define TCP_MODE 0x01
#define UDP_MODE 0x02
#define CLOSE_SOCK 0x00
#define MACRAW_MODE 0x04
// Socket command
#define OPEN 0x01
#define RECV 0x40
#define SEND 0x20

#define RXdata (RXbuf+8) // to remove offset because of service bytes

// FUNCTIONS
void w5500_init(void);
uint8_t w5500_readSnIMR(void);
void w5500_setSnIMR(void);
uint8_t w5500_readSIMR(void);
uint8_t w5500_getStatus(void);
void w5500_enSockInt(void);
void w5500_SockIntClr(void);
void w5500_receive(uint8_t* rxbuf);
void w5500_receive_Nbytes(uint8_t* rxbuf, uint8_t num_of_bytes);
void w5500_send(uint8_t* txbuf, uint8_t n_of_bytes);
void w5500_setDestIP(uint8_t sock, uint8_t* ip);
void w5500_setDestPORT(uint8_t sock, uint16_t port);
#endif
