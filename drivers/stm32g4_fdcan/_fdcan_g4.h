#ifndef INTERNAL_FDCAN_H
#define INTERNAL_FDCAN_H

#include <stdint.h>

/* ------------------- SRAM layout --------------------------------------------------------------------------------- */

typedef struct __attribute__((packed, aligned(4))) {
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

/* Helper to iterate over RXFxS/A */
typedef struct __attribute__((packed, aligned(4))) {
	volatile uint32_t RXFxS;
	volatile uint32_t RXFxA;
} fdcan_rxfifo_regs;

/* ------------------- SRAM layout --------------------------------------------------------------------------------- */
/* Note: bitfields only to get a register-like view in GDB. Don't use them directly, it explodes.                    */

typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t t0;
        struct {
            uint32_t id             : 29;
            uint32_t rtr            : 1; /* remote frame */
            uint32_t xtd            : 1; /* extended frame */
            uint32_t esi            : 1; /* Force error passive flag */
        };
    };

    union {
        uint32_t t1;
        struct {
            uint32_t reserved0      : 16;
            uint32_t dlc            : 4; /* DLC */
            uint32_t brs            : 1; /* Bit Rate Switching Frame */
            uint32_t fdf            : 1; /* CAN FD frame */
            uint32_t reserved1      : 1;
            uint32_t efc            : 1; /* store tx events */
            uint32_t mm             : 8; /* message marker (tx events) */
        };
    };
    uint32_t data[64 / sizeof(uint32_t)];
} fdcan_tx_buf_element;

typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t r0;
        struct {
            uint32_t id             : 29;
            uint32_t rtr            : 1; /* remote */
            uint32_t xtd            : 1; /* extended */
            uint32_t esi            : 1; /* transmitter error passive */
        };
    };

    union {
        uint32_t r1;
        struct {
            uint32_t rxts           : 16;/* timestamp */
            uint32_t dlc            : 4;
            uint32_t brs            : 1; /* Bit Rate Switching Frame */
            uint32_t fdf            : 1; /* CAN FD frame */
            uint32_t reserved0      : 1;
            uint32_t fidx           : 7; /* filter index */
            uint32_t anmf           : 1; /* no filter matched */
        };
    };
    uint32_t data[64 / sizeof(uint32_t)];
} fdcan_rx_buf_element;

typedef struct __attribute__((packed, aligned(4))) {
    /* not detailed, standard IDs not used for DroneCAN */
    uint32_t stub;
} fdcan_stdid_filter_element;

typedef struct __attribute__((packed, aligned(4))) {
    /* not detailed, tx events are not used */
    uint32_t stub[2];
} fdcan_tx_event_element;

typedef struct __attribute__((packed, aligned(4))) {
    union {
        uint32_t f0;
        struct {
            uint32_t efid1      : 29;
            enum {
                FILT_ELEM_EFEC_DISABLE,
                FILT_ELEM_EFEC_STORE_RXFIFO0,
                FILT_ELEM_EFEC_STORE_RXFIFO1,
                FILT_ELEM_EFEC_REJECT,
                FILT_ELEM_EFEC_SET_PRIORITY_STORE_RXFIFO0,
                FILT_ELEM_EFEC_SET_PRIORITY_STORE_RXFIFO1,
                FILT_ELEM_EFEC_RESERVED,
            } efec              : 3;
        };
    };
    union {
        uint32_t f1;
        struct {
            uint32_t efid2      : 29;
            uint32_t reserved0  : 1;
            enum {
                FILT_ELEM_EFT_RANGE,
                FILT_ELEM_EFT_DUAL_ID,
                FILT_ELEM_EFT_ID_MASK,
                FILT_ELEM_EFT_RANGE_NO_XIDAM
            } eft               : 2;
        };
    };
} fdcan_extid_filter_element;

/* Figure 669 RM0440 */
#define FDCAN_NUM_STDID_FE              28
#define FDCAN_NUM_EXTID_FE              8
#define FDCAN_NUM_RXFIFO_ELEMENTS       3
#define FDCAN_NUM_TX_EVENT_FIFOS        3
#define FDCAN_NUM_TX_BUF                3

typedef struct __attribute__((packed, aligned(4))) {
    fdcan_stdid_filter_element stdid_filter_element[FDCAN_NUM_STDID_FE];    // 28   // 28
    fdcan_extid_filter_element extid_filter_element[FDCAN_NUM_EXTID_FE];    // 16   // 44
    fdcan_rx_buf_element rxfifo0[FDCAN_NUM_RXFIFO_ELEMENTS];                // 54   // 98
    fdcan_rx_buf_element rxfifo1[FDCAN_NUM_RXFIFO_ELEMENTS];                // 54   // 152
    fdcan_tx_event_element txevent_fifo[FDCAN_NUM_TX_EVENT_FIFOS];          // 6    // 158
    fdcan_tx_buf_element txbuf[FDCAN_NUM_TX_BUF];                           // 54   // 212 == 0x350
} fdcan_sram;

static_assert(sizeof(fdcan_sram) == 0x350);

#define DLC_OFFSET 16

#define FDCAN1 ((fdcan_registers *) FDCAN1_ADDR)
#define FDCAN2 ((fdcan_registers *) FDCAN2_ADDR)
#define FDCAN3 ((fdcan_registers *) FDCAN3_ADDR)

#define SRAMCAN_START 			0x4000A400U

#endif
