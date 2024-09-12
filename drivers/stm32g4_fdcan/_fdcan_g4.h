#ifndef INTERNAL_FDCAN_H
#define INTERNAL_FDCAN_H

#include <stdint.h>

typedef struct {
	volatile uint32_t CREL;
	volatile uint32_t ENDN;
	volatile uint32_t RESERVED1;
	volatile uint32_t DBTP;
	volatile uint32_t TEST;
	volatile uint32_t RWD;
	volatile uint32_t CCCR;
	volatile uint32_t NBTP;
	volatile uint32_t TSCC;
	volatile uint32_t TSCV;
	volatile uint32_t TOCC;
	volatile uint32_t TOCV;
	volatile uint32_t RESERVED2[4];
	volatile uint32_t ECR;
	volatile uint32_t PSR;
	volatile uint32_t TDCR;
	volatile uint32_t RESERVED3;
	volatile uint32_t IR;
	volatile uint32_t IE;
	volatile uint32_t ILS;
	volatile uint32_t ILE;
	volatile uint32_t RESERVED4[8];
	volatile uint32_t RXGFC;
	volatile uint32_t XIDAM;
	volatile uint32_t HPMS;
	volatile uint32_t RESERVED5;
	volatile uint32_t RXF0S;
	volatile uint32_t RXF0A;
	volatile uint32_t RXF1S;
	volatile uint32_t RXF1A;
	volatile uint32_t RESERVED6[8];
	volatile uint32_t TXBC;
	volatile uint32_t TXFQS;
	volatile uint32_t TXBRP;
	volatile uint32_t TXBAR;
	volatile uint32_t TXBCR;
	volatile uint32_t TXBTO;
	volatile uint32_t TXBCF;
	volatile uint32_t TXBTIE;
	volatile uint32_t TXBCIE;
	volatile uint32_t TXEFS;
	volatile uint32_t TXEFA;
} fdcan_registers;

typedef struct {
	volatile uint32_t RXFxS;
	volatile uint32_t RXFxA;
} fdcan_rxfifo_regs;

typedef struct {
	volatile uint32_t f0;
	volatile uint32_t f1;
} fdcan_ext_filt_elem;

#define EFEC_OFFSET 29
#define EFT_OFFSET 30

#define FDCAN_RXFIFO_ELEM_LEN (64 / 4 + 2)
typedef struct {
	volatile uint32_t r[FDCAN_RXFIFO_ELEM_LEN];
} fdcan_rxfifo_elem;

#define FDCAN_TXFIFO_ELEM_LEN (64 / 4 + 2)
typedef struct {
	volatile uint32_t t[FDCAN_TXFIFO_ELEM_LEN];
} fdcan_tx_buf_elem;

#define DLC_OFFSET 16

#define FDCAN1 ((fdcan_registers *) FDCAN1_ADDR)
#define FDCAN2 ((fdcan_registers *) FDCAN2_ADDR)
#define FDCAN3 ((fdcan_registers *) FDCAN3_ADDR)

#define SRAMCAN_START 			0x4000A400U
#define SRAMCAN_SIZE			0x0350U
#define SRAMCAN_EXT_FILT_OFFSET 0x0070U
#define SRAMCAN_RXFIFO0_OFFSET 	0x00B0U
#define SRAMCAN_RXFIFO1_OFFSET 	0x0188U
#define SRAMCAN_RXFIFO_SIZE 	(SRAMCAN_RXFIFO1_OFFSET - SRAMCAN_RXFIFO0_OFFSET)
#define SRAMCAN_TXB_OFFSET		0x0278U

#define DRONECAN_MESSAGEID_FILTER 0xFFFF80

#define FDCAN_NUM_EXT_FILTERS 8

#endif
