#include "w5500.h"

/*
UDP data is specially structured in the RX memory, like this:
PACKET-INFO
4 bytes: destination IP address
2 bytes: destination port number
2 bytes: byte size (N) of data packet
DATA-PACKET
N bytes: real data
*/
extern SPI_HandleTypeDef hspi1;
extern uint8_t IP[4];
extern uint8_t MAC[6];
extern uint16_t PORT;
extern uint8_t GATE[4];
extern uint8_t MASK[4];
extern uint8_t snimr; //debug
extern uint8_t simr; //debug
uint8_t st; //debug
uint8_t delme[128]; //debug
uint16_t data_size = 0; //debug
uint16_t rPointer = 0; //debug
uint8_t DATA = 0; //debug
uint8_t debug_buf[10]; //debug

void w5500_writeByte(uint8_t op, uint16_t address, uint8_t data)
{
	uint8_t send[] = { (address >> 8), address, op|(RWB_WRITE << 2), data };
	W5500_SELECT();
	HAL_SPI_Transmit(&hspi1, send, 4, HAL_MAX_DELAY);
	W5500_UNSELECT();
}

void w5500_writeBytes(uint8_t op, uint16_t address, uint8_t* data, uint8_t n_of_bytes)
{
	op &= ~(0x03); // hmm
	uint8_t n = n_of_bytes + 3;
	uint8_t send[n];
	memset(send, 0, n);
	send[0] = (address >> 8);
	send[1] = address;
	send[2] = op|(RWB_WRITE << 2);
	memcpy(send+3, data, n_of_bytes);
	W5500_SELECT();
	HAL_SPI_Transmit(&hspi1, send, n, HAL_MAX_DELAY);
	W5500_UNSELECT();
}

uint8_t w5500_readByte(uint8_t op, uint16_t address)
{
	uint8_t send[] = { (address >> 8), address, op|(RWB_READ << 2), 0x00 };
	uint8_t ans[4]; ans[3] = 0xFF;
	W5500_SELECT();
	HAL_SPI_TransmitReceive(&hspi1, send, ans, 4, HAL_MAX_DELAY);
	//HAL_SPI_Receive(&hspi1, &ans, 1, HAL_MAX_DELAY);
	W5500_UNSELECT();
	return ans[3];
}

void w5500_readBytes(uint8_t op, uint16_t address, uint8_t* buff, uint8_t n_of_bytes)
{
	op &= ~(0x03); // hmm
	uint8_t send[] = { (address >> 8), address, op|(RWB_READ << 2), 0x00 };
	W5500_SELECT();
	HAL_SPI_Transmit(&hspi1, send, 3, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, buff, n_of_bytes, HAL_MAX_DELAY);
	W5500_UNSELECT();
}

uint8_t w5500_getStatus(void)
{
	uint8_t op = 0;
	uint8_t ans = 0xFF;
	op = (BSB_S0 << 3) | OM_1B;
	ans = w5500_readByte( op, Sn_SR ); 
	return ans;
}

void w5500_openSocket(uint8_t sock_num, uint8_t mode)
{
	uint8_t op = 0;
	op = (((sock_num << 2) | BSB_S0) << 3) | OM_1B;
	w5500_writeByte( op, Sn_MR, mode);
	w5500_writeByte( op, Sn_CR, OPEN);
}

void w5500_recvSocket(uint8_t sock_num)
{
	uint8_t op = 0;
	op = (((sock_num << 2) | BSB_S0) << 3) | OM_1B;
	w5500_writeByte( op, Sn_CR, RECV);
}

void w5500_sendSocket(uint8_t sock_num)
{
	uint8_t op = 0;
	op = (((sock_num << 2) | BSB_S0) << 3) | OM_1B;
	w5500_writeByte( op, Sn_CR, SEND);
}

void w5500_init(void) 
{
	uint8_t op = 0;
	// hard reset
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_Delay(70);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(70);
	// soft reset
	op = (BSB_COMMON << 3) | OM_1B;
	w5500_writeByte( op, Sn_MR, 0x80 );
	HAL_Delay(100);
	op = (BSB_COMMON << 3) | OM_NB;
	// MAC config
	w5500_writeBytes( op, SHAR0, MAC, 6);
	// IP config
	w5500_writeBytes( op, SIPR0, IP, 4 );
	// SUBNET config
	w5500_writeBytes( op, SUBR0, MASK, 4);
	// GATE config
	w5500_writeBytes( op, GAR0, GATE, 4);
	// PORT config
	op = (BSB_S0<<3) | OM_NB;
	uint8_t port[2] = { PORT >> 8, PORT };
	w5500_writeBytes( op, Sn_PORT0, port, 2); 
	
	//w5500_setDestIP(0);
	//w5500_setDestPORT(0);
  w5500_openSocket( 0, UDP_MODE );
	w5500_enSockInt();
	w5500_setSnIMR();
	//w5500_SockIntClr();
	//wait until socket init
	while (1)
	{
		st = w5500_getStatus();
		if (st == SOCK_UDP)
		{
			break;
		}
	}
}

void w5500_enSockInt(void)
{
	uint8_t op = 0;
	op = (BSB_COMMON << 3) | OM_1B;
	w5500_writeByte(op, SIMR, 0x01); // Socket 0
}

void w5500_setSnIMR(void)
{
	uint8_t op = 0;
	op = (BSB_S0 << 3) | OM_1B;
	w5500_writeByte(op, Sn_IMR, 0x04); // SENDOK - OFF, RECV - ON (page 57)
}

void w5500_SockIntClr(void) 
{
	uint8_t op = 0;
	op = (BSB_S0 << 3) | OM_1B;
	w5500_writeByte(op, Sn_IR, 0x04); // 0x04: 2 bit - Recv interrupt // 0x1f: all interrupts
}

uint8_t w5500_readSIMR(void) // which socket has interrupt enabled
{
	uint8_t op = 0;
	op = (BSB_COMMON << 3) | OM_1B;
	uint8_t ans = w5500_readByte(op, SIMR);
	simr = ans;
	return ans;
}

uint8_t w5500_readSnIMR(void) // which socket interrupt enabled
{
	uint8_t op = 0;
	op = (BSB_S0 << 3) | OM_1B;
	uint8_t ans = w5500_readByte(op, Sn_IMR);
	snimr = ans;
	return ans;
}

void w5500_setDestIP(uint8_t sock, uint8_t* ip)
{
	uint8_t op = 0;
	op = (((sock << 2) | BSB_S0) << 3) | OM_NB;
	//uint8_t ip[4] = {192, 168, 1, 12};
	w5500_writeBytes(op, Sn_DIPR0, ip, 4);
	//w5500_readBytes(op, Sn_DIPR0, debug_buf, 4);
}

void w5500_setDestPORT(uint8_t sock, uint16_t port)
{
	uint8_t op = 0;
	op = (((sock << 2) | BSB_S0) << 3) | OM_NB;
	//uint8_t port[2] = { 4001 >> 8, 4001 };
	uint8_t portb[2] = { port >> 8, port };
	w5500_writeBytes(op, Sn_DPORT0, portb, 2);
	//w5500_readBytes(op, Sn_DPORT0, debug_buf+5, 2);
}
	

void w5500_receive(uint8_t* rxbuf)
{
	uint8_t op = 0;
	uint16_t rdA = 0;
	op = (BSB_S0 << 3) | OM_1B;
	data_size = (w5500_readByte(op,Sn_RX_RSR0)<<8|w5500_readByte(op,Sn_RX_RSR1));
  //data_size = w5500_readByte(op, Sn_RX_RSR0);
	//data_size <<= 8;
	//data_size |= w5500_readByte(op, Sn_RX_RSR1);
	rdA = (w5500_readByte(op,Sn_RX_RD0)<<8|w5500_readByte(op,Sn_RX_RD1));
	rPointer = rdA;
	op = (BSB_S0_RX << 3) | OM_NB;
	w5500_readBytes(op, rdA, rxbuf, data_size);
  rdA = rdA + data_size;
	rdA = ((rdA << 8)&0xFF00) | ((rdA >> 8)&0xFF);
	op = (BSB_S0 << 3) | OM_NB;
	w5500_writeBytes(op, Sn_RX_RD0, (uint8_t*)&rdA, 2);
	w5500_recvSocket(0);
}

void w5500_receive_Nbytes(uint8_t* rxbuf, uint8_t num_of_bytes)
{
	uint8_t op = 0;
	uint16_t rdA = 0;
	op = (BSB_S0 << 3) | OM_1B;
	rdA = (w5500_readByte(op,Sn_RX_RD0)<<8|w5500_readByte(op,Sn_RX_RD1));
	rPointer = rdA;
	op = (BSB_S0_RX << 3) | OM_NB;
	w5500_readBytes(op, rdA, rxbuf, num_of_bytes);
  rdA = rdA + num_of_bytes;
	rdA = ((rdA << 8)&0xFF00) | ((rdA >> 8)&0xFF);
	op = (BSB_S0 << 3) | OM_NB;
	w5500_writeBytes(op, Sn_RX_RD0, (uint8_t*)&rdA, 2);
	w5500_recvSocket(0);
}
	
void w5500_send(uint8_t* txbuf, uint8_t n_of_bytes)
{
	uint8_t op = 0;
	uint16_t wrA = 0;
	op = (BSB_S0 << 3) | OM_1B;
	wrA = (w5500_readByte(op,Sn_TX_WR0)<<8|w5500_readByte(op,Sn_TX_WR1));
	op = (BSB_S0_TX << 3) | OM_NB;
	w5500_writeBytes(op, wrA, txbuf, n_of_bytes);
	wrA = wrA + n_of_bytes;
	wrA = ((wrA << 8)&0xFF00) | ((wrA >> 8)&0xFF);
	op = (BSB_S0 << 3) | OM_NB;
	w5500_writeBytes(op, Sn_TX_WR0, (uint8_t*)&wrA, 2);
	w5500_sendSocket(0);
}
