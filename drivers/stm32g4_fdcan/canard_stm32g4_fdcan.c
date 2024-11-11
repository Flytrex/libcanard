/*
 * canard_stm32g4_fdcan.c
 *
 *  Created on: Sep 10, 2024
 *      Author: grishar
 */

#include "canard_stm32g4_fdcan.h"

#include "_fdcan_g4.h"
#include "../stm32/canard_stm32.h" /* for CanardSTM32ComputeCANTimings */

/* Local  */
__attribute__((const))
static inline uint32_t fdcan_ram(const fdcan_registers *r);
static void clear_and_handle_faults(canard_stm32g4_fdcan_driver *driver);
static void canard_frame_to_tx_buf_elem(const CanardCANFrame* const frame, fdcan_tx_buf_element *ele);
static int rxfifo_get_first_elem_index(fdcan_rxfifo_regs *rxf);
static void rxfifo_receive_frame(fdcan_registers *regs, CanardCANFrame* const out_frame, fdcan_rx_buf_element *ele);
static int rxfifo_get_fill_level(fdcan_rxfifo_regs *rxf);
static int rxfifo_get_read_index(fdcan_rxfifo_regs *rxf);
static void rxfifo_ack_frame(fdcan_rxfifo_regs *rxfifo_regs, int ack_index);

__attribute__((const))
static inline int dlc_decode(int dlc_value, int fd);

__attribute__((const))
static inline int dlc_encode(int data_len, int fd);

#define EXT_ID_FILTER 0x1FFFFFFF

/* Module */

int canard_stm32g4fdcan_init(canard_stm32g4_fdcan_driver *driver, int bitrate_bps, int fdbitrate_bps, int periph_clock_rate)
{
	/* Calculate timings based on the provided bitrates */
	CanardSTM32CANTimings nominal_timings = {0};
	int rc = canardSTM32ComputeCANTimings(periph_clock_rate, bitrate_bps, &nominal_timings);

    if ((nominal_timings.bit_rate_prescaler < 1) || (nominal_timings.bit_rate_prescaler > 1024) ||
        (nominal_timings.max_resynchronization_jump_width < 1) || (nominal_timings.max_resynchronization_jump_width > 4) ||
        (nominal_timings.bit_segment_1 < 1) || (nominal_timings.bit_segment_1 > 16) ||
        (nominal_timings.bit_segment_2 < 1) || (nominal_timings.bit_segment_2 > 8))
    {
        return -CANARD_ERROR_INVALID_ARGUMENT;
    }

	if (rc < 0)
		return rc;

#if CANARD_ENABLE_CANFD
	CanardSTM32CANTimings brs_timings = {0};
	rc = canardSTM32ComputeCANTimings(periph_clock_rate, fdbitrate_bps, &brs_timings);

    if ((brs_timings.bit_rate_prescaler < 1) || (brs_timings.bit_rate_prescaler > 1024) ||
        (brs_timings.max_resynchronization_jump_width < 1) || (brs_timings.max_resynchronization_jump_width > 4) ||
        (brs_timings.bit_segment_1 < 1) || (brs_timings.bit_segment_1 > 16) ||
        (brs_timings.bit_segment_2 < 1) || (brs_timings.bit_segment_2 > 8))
    {
        return -CANARD_ERROR_INVALID_ARGUMENT;
    }

	if (rc < 0)
		return rc;
#endif

	switch ((uint32_t) driver->fdcan) {
	case FDCAN1_ADDR:
	case FDCAN2_ADDR:
	case FDCAN3_ADDR:
		break;
	default:
		CANARD_ASSERT(0);
	}

	fdcan_registers *fdcan = driver->fdcan;
	CANARD_ASSERT(fdcan->ENDN == 0x87654321);		/* Also confirms that driver->fdcan
													 * is probably pointing at something sensible */

	/* Enter configuration mode */
	fdcan->CCCR &= ~(1 << 4);						/* Exit from sleep mode */
	while (fdcan->CCCR & (1 << 3));					/* Wait until exited from sleep mode */
	fdcan->CCCR |=  (1 << 0);						/* Get to INIT mode */
	while (!(fdcan->CCCR & (1 << 0)));				/* Wait until init mode sets */
	fdcan->CCCR |=  (1 << 1);                       /* CCE=1: config change enable */
	while (!(fdcan->CCCR & (1 << 1)));              /* Wait intil CCE sets */

	/* General config */
	fdcan->CCCR |=  (1 << 6) |						/* DAR=1: automatic retransmission should be disabled
													 * before DroneCAN Node ID is assigned */
					(1 << 13)|						/* EFBI=1: edge filtering enabled */
					(1 << 14);						/* TXP=1: Enable transmit pause -- should be generally beneficial. */


#if CANARD_ENABLE_CANFD
	fdcan->CCCR |= (1 << 8) | (1 << 9); 			/* Enable FD and Bit Rate Switching */
#endif

	/* Filter config */
	fdcan->RXGFC =
			(8 << 24) 	|				/* LSE = 8: number of Extended ID filters */
			(0 << 16) 	|				/* LSS = 0: number of Standard ID filters */
			(0 << 9) 	| (0 << 8) |	/* F0OM = F1OM = 0: FIFO to blocking mode  */
			(2 << 4)	|				/* ANFS = 2: Reject non-matched standard frames */
			(1 << 2)	| 				/* ANFE = 1: Non-matched extended frames go to FIFO1 */
			(1 << 1) 	| (1 << 0);		/* RRFS = RRFE = 1: reject remote frames */

	fdcan->XIDAM = 0xFFFF00;		    /* Only consider the DroneCAN Message type ID part in the CAN frame.
										 * This allows to setup each filter element as a dual ID filter
										 * and have 16 message rules in total.
										 * Matches only broadcast messages (serviceNotMessage bit
										 * will never match with anything in the filters). */
	/* Timing config */
	/* Set nominal timings */
	fdcan->NBTP = ((nominal_timings.max_resynchronization_jump_width - 1) 	<< 25U) |
				  ((nominal_timings.bit_segment_2 - 1) 						<< 0U) |
				  ((nominal_timings.bit_segment_1 - 1) 						<< 8U) |
				  ((nominal_timings.bit_rate_prescaler - 1)					<< 16U);

#if CANARD_ENABLE_CANFD
	/* Set Bit Rate Switching timings */
	fdcan->DBTP = (1 << 23) | /* TDC = 1: enable transceiver delay compensation */
			 	  ((brs_timings.max_resynchronization_jump_width - 1) 	<< 0U) |
				  ((brs_timings.bit_segment_2 - 1) 						<< 4U) |
				  ((brs_timings.bit_segment_1 - 1) 						<< 8U) |
				  ((brs_timings.bit_rate_prescaler - 1)					<< 16U);
#endif

	/* Set TX buffer to queue mode (let it care about the priority inversion) */
	fdcan->TXBC = (1 << 24); /* TFQM = 1 */

	/* Enable potentially useful interrupts:
	 * - on fifo0/1 received/lost
	 * - on bus-off */
	/* 			 bus-off  | rx1 lost | rx1 new  | rx0 lost | rx0 new */
	fdcan->IE = (1 << 19) | (1 << 5) | (1 << 3) | (1 << 2) | (1 << 0);
	driver->fdcan_sram = (void *) fdcan_ram(fdcan);
	memset(driver->fdcan_sram, 0, sizeof(fdcan_sram));
	return 0;
}

#define STM32G4_FDCAN_NUM_EXT_FILTER_ELEMENTS 8
#define EFEC_OFFSET 29
#define EFEC_MASK 0x7
#define EFT_OFFSET 30

static int filt_elem_get_efec(fdcan_extid_filter_element *fe)
{
    return (fe->f0 & (EFEC_MASK << EFEC_OFFSET)) >> EFEC_OFFSET;
}

int canard_stm32g4fdcan_type_id_filter(canard_stm32g4_fdcan_driver *driver,
										int dronecan_type_id1, int dronecan_type_id2, int accept_not_reject)
{
    fdcan_sram *sram = driver->fdcan_sram;
	for (int i = 0; i < STM32G4_FDCAN_NUM_EXT_FILTER_ELEMENTS; ++i) {
	    fdcan_extid_filter_element *filt = &sram->extid_filter_element[i];
	    if (filt_elem_get_efec(filt) == FILT_ELEM_EFEC_DISABLE) {
	        filt->f0 = dronecan_type_id1 << 8;
	        filt->f1 = (FILT_ELEM_EFT_DUAL_ID << EFT_OFFSET) | (dronecan_type_id2 << 8);
	        filt->f0 |= accept_not_reject ? FILT_ELEM_EFEC_STORE_RXFIFO0 << EFEC_OFFSET :
	                                            FILT_ELEM_EFEC_REJECT << EFEC_OFFSET;
	        return 0;
	    }
	}
	return -CANARD_ERROR_STM32_FDCAN_OUT_OF_FILTER_SPACE;
}

void canard_stm32g4fdcan_start(canard_stm32g4_fdcan_driver *driver)
{
	fdcan_registers *fdcan = driver->fdcan;
	fdcan->CCCR &= ~((1 << 0) | (1 << 1)); 		/* Clear INIT and CCE */
	while ((fdcan->CCCR & (1 << 0)));	        /* Wait until we leave init mode */
}

int canard_stm32g4fdcan_transmit(canard_stm32g4_fdcan_driver *driver, const CanardCANFrame* const frame)
{
	clear_and_handle_faults(driver);
	fdcan_registers *fdcan = driver->fdcan;
	fdcan_sram *sram = driver->fdcan_sram;
	if (fdcan->TXFQS & (1 << 21)) {
		/* TFQF set: TX queue full */
		return 0;
	}
	int put_index = (fdcan->TXFQS & (3 << 16)) >> 16;
	canard_frame_to_tx_buf_elem(frame, &sram->txbuf[put_index]);
	fdcan->TXBAR |= (1 << put_index);
	return 1;
}

int canard_stm32g4fdcan_receive(canard_stm32g4_fdcan_driver *driver, CanardCANFrame* const out_frame)
{
	clear_and_handle_faults(driver);
	fdcan_registers *fdcan = driver->fdcan;
	fdcan_sram *sram = driver->fdcan_sram;
	fdcan_rxfifo_regs *rxfifo[2] = {(fdcan_rxfifo_regs *) &fdcan->RXF0S, (fdcan_rxfifo_regs *) &fdcan->RXF1S};
	int index;
	index = rxfifo_get_first_elem_index(rxfifo[0]);
	if (index != -1) {
	    rxfifo_receive_frame(fdcan, out_frame, &sram->rxfifo0[index]);
	    rxfifo_ack_frame(rxfifo[0], index);
	    return 1;
	}

	index = rxfifo_get_first_elem_index(rxfifo[1]);
    if (index != -1) {
        rxfifo_receive_frame(fdcan, out_frame, &sram->rxfifo1[index]);
        rxfifo_ack_frame(rxfifo[1], index);
        return 1;
    }

	return 0;
}

void canard_stm32g4fdcan_enable_automatic_retransmission(canard_stm32g4_fdcan_driver *driver)
{
	fdcan_registers *fdcan = driver->fdcan;
	fdcan->CCCR &= ~(1 << 6);
}

void canard_stm32g4fdcan_get_protocol_state(canard_stm32g4_fdcan_driver *driver, canard_stm32g4fdcan_protocol_state *s)
{
	fdcan_registers *fdcan = driver->fdcan;
	s->bus_off = (fdcan->PSR & (1 << 7)) > 0;
	s->warning = (fdcan->PSR & (1 << 6)) > 0;
	s->error_passive = (fdcan->PSR & (1 << 5)) > 0;
	s->tec = fdcan->ECR & 0xFF;
	s->rec = (fdcan->ECR & 0xF00) >> 16;
}

/* Local */

static void canard_frame_to_tx_buf_elem(const CanardCANFrame* const frame, fdcan_tx_buf_element *ele)
{
    CANARD_ASSERT(frame->data_len); /* DLC = 0 not permitted in DroneCAN */

    /* See Table 408 RM0440 */
    /*           ESI = 0   29-bit id   not remote   CAN frame ID */
    ele->t0 = (0 << 31) | (1 << 30) | (0 << 29) | (frame->id & EXT_ID_FILTER);

#if CANARD_ENABLE_CANFD
    /* don't store events  is this an FD frame    use bit rate switch                    DLC                               */
    ele->t1 = (0 << 23) | (frame->canfd << 21) |  (frame->canfd << 20)  | (dlc_encode(frame->data_len, frame->canfd) << 16);
#else
    ele->t1 = (0 << 23) | (0 << 21)            |  (0 << 20)             | (dlc_encode(frame->data_len, 0) << 16);
#endif

    uint32_t *tx_data_frame = (uint32_t *) frame->data;
    ele->data[0] = tx_data_frame[0];
    ele->data[1] = tx_data_frame[1];
#if CANARD_ENABLE_CANFD
    for (size_t i = 2; i < frame->data_len / sizeof(uint32_t); ++i) {
        ele->data[i] = tx_data_frame[i];
    }
#endif
}

static void clear_and_handle_faults(canard_stm32g4_fdcan_driver *driver)
{
	fdcan_registers *fdcan = driver->fdcan;
	uint32_t psr = fdcan->PSR; /* reading clears PSR */

	if (psr & (1 << 7)) {
		/* Bus-off -- we need to clear INIT which is set by hardware in this case.
		 * See RM0440 44.4.13 */
		canard_stm32g4fdcan_start(driver);
		driver->statistics.bus_off_events++;
	}
	if (psr & (1 << 14)) { /* PXE */
		driver->statistics.protocol_exceptions++;
	}

	uint32_t ir = fdcan->IR;
	if (ir & (1 << 2)) {
	    /* RF0L */
	    driver->statistics.rx_fifo0_overruns++;
	}
	if (ir & (1 << 5)) {
	    /* RF1L */
		driver->statistics.rx_fifo1_overruns++;
	}
	if (ir & (1 << 12))  {
	    /* TEFL */
		driver->statistics.tx_fifo_overruns++;
	}
	fdcan->IR = 0xFFFFFF; /* clear IR */
}

#define DLC_OFFSET 16
#define DLC_MASK 0xF

static int rxfifo_get_dlc(fdcan_rx_buf_element *ele)
{
    return (ele->r1 & (DLC_MASK << DLC_OFFSET)) >> DLC_OFFSET;
}

static void rxfifo_receive_frame(fdcan_registers *regs, CanardCANFrame* const out_frame, fdcan_rx_buf_element *ele)
{
	/* Don't need to check if this is a standard or remote frame (not needed for DroneCAN),
	 * they're rejected by the filter configuration */
	out_frame->id = ele->r0 & EXT_ID_FILTER;
	out_frame->id |= CANARD_CAN_FRAME_EFF; /* canardHandleRxFrame() fails if this bit is not set. We set it manually,
	                                        * no standard frames are received by the filter configuration */
	out_frame->iface_id = ((uint32_t) regs - FDCAN1_ADDR) / sizeof(fdcan_registers);
#if CANARD_ENABLE_CANFD
	out_frame->canfd = (ele->r0 & (1 << 21)) > 0;
	out_frame->data_len = dlc_decode(rxfifo_get_dlc(ele), out_frame->canfd);
	CANARD_ASSERT(out_frame->data_len <= 64);
#else
	out_frame->data_len = dlc_decode(rxfifo_get_dlc(ele), 0);
	CANARD_ASSERT(out_frame->data_len <= 8);
#endif

    uint32_t *out_data = (uint32_t *) out_frame->data;
    /* It's faster to move two words around regardless of the actual DLC */
    out_data[0] = ele->data[0];
    out_data[1] = ele->data[1];


#if CANARD_ENABLE_CANFD
    if (out_frame->data_len >= 8) {
        int limit = out_frame->data_len / sizeof(uint32_t);
        for (int i = 0; i < limit; ++i) {
            out_data[i] = ele->data[i];
        }
    }
#endif
}

__attribute__((const))
static inline uint32_t fdcan_ram(const fdcan_registers *r)
{
	return SRAMCAN_START + ((uint32_t) r - FDCAN1_ADDR) / sizeof(fdcan_registers) * sizeof(fdcan_sram);
}

static int rxfifo_get_first_elem_index(fdcan_rxfifo_regs *rxf)
{
    if (rxfifo_get_fill_level(rxf)) {
        return rxfifo_get_read_index(rxf);
    }
    return -1;
}

static int rxfifo_get_fill_level(fdcan_rxfifo_regs *rxf)
{
	return rxf->RXFxS & 0xF;
}

static int rxfifo_get_read_index(fdcan_rxfifo_regs *rxf)
{
	return (rxf->RXFxS & (3 << 8)) >> 8;
}

__attribute__((const))
static inline int dlc_decode(int dlc_value, int fd)
{
    if (fd) {
        if (dlc_value <= 8) {
            return dlc_value;
        }
        else {
            switch (dlc_value) {
            case 9:
                return 12;
            case 10:
                return 16;
            case 11:
                return 20;
            case 12:
                return 24;
            case 13:
                return 32;
            case 14:
                return 48;
            case 15:
                return 64;
            default:
                CANARD_ASSERT(0);
                return 0;
            }
        }
    }
    else {
        if (dlc_value <= 8) {
            return dlc_value;
        }
        else {
            CANARD_ASSERT(0);
        }
    }
}

__attribute__((const))
static inline int dlc_encode(int data_len, int fd)
{
	if (data_len <= 8) {
		return data_len;
	}
	else if (fd) {
		switch (data_len) {
		case 12:
			return 9;
		case 16:
			return 10;
		case 20:
			return 11;
		case 24:
			return 12;
		case 32:
			return 13;
		case 48:
			return 14;
		case 64:
			return 15;
		default:
			CANARD_ASSERT(0);
			return 0;
		}
	}
	else {
		CANARD_ASSERT(0);
		return 0;
	}
}

static void rxfifo_ack_frame(fdcan_rxfifo_regs *rxfifo_regs, int ack_index)
{
	rxfifo_regs->RXFxA |= 7 & ack_index;
}
