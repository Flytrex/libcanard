/*
 * canard_stm32g4_fdcan.c
 *
 *  Created on: Sep 10, 2024
 *      Author: grishar
 */

#include "drivers/stm32g4_fdcan/canard_stm32g4_fdcan.h"

#include "drivers/stm32g4_fdcan/_fdcan_g4.h"
#include "../stm32/canard_stm32.h" /* for CanardSTM32ComputeCANTimings */

/* Local  */
__attribute__((const))
static inline uint32_t fdcan_ram(const fdcan_registers *r);

static void clear_and_handle_faults(canard_stm32g4_fdcan_driver *driver);

static fdcan_tx_buf_elem *get_tx_buf_element(canard_stm32g4_fdcan_driver *driver, int index);
static void canard_frame_to_tx_buf_elem(const CanardCANFrame* const frame, fdcan_tx_buf_elem *ele);

static void rxfifo_receive_frame(fdcan_registers *regs, CanardCANFrame* const out_frame, fdcan_rxfifo_elem *ele);

__attribute__((const))
static inline fdcan_rxfifo_elem *get_rxfifo_elem(const canard_stm32g4_fdcan_driver *driver,
								fdcan_rxfifo_regs *fifo_regs, size_t elemindex);
static int rxfifo_elem_is_fd_frame(fdcan_rxfifo_elem *rxf);
static int rxfifo_elem_get_ext_id(fdcan_rxfifo_elem *rxf);
static volatile uint32_t *rxfifo_elem_get_payload(fdcan_rxfifo_elem *rxf);
static int rxfifo_get_fill_level(fdcan_rxfifo_regs *rxf);
static int rxfifo_get_read_index(fdcan_rxfifo_regs *rxf);
static fdcan_rxfifo_elem *rxfifo_get_first_elem(canard_stm32g4_fdcan_driver *driver, fdcan_rxfifo_regs *rxf, int *elem_index);
static int rxfifo_elem_get_dlc(fdcan_rxfifo_elem *rxf);
static void rxfifo_ack_frame(fdcan_rxfifo_regs *rxfifo_regs, int ack_index);

__attribute__((const))
static inline fdcan_ext_filt_elem* get_ext_filt_elem(const fdcan_registers *r, size_t index);
static int filt_elem_active(const fdcan_ext_filt_elem * const fe);
static void filt_elem_accept_dual_id(fdcan_ext_filt_elem *fe, uint32_t frame_id1, uint32_t frame_id2);
static void filt_elem_reject_dual_id(fdcan_ext_filt_elem *fe, uint32_t frame_id1, uint32_t frame_id2);

__attribute__((const))
static inline int dlc_decode(int dlc_value, int fd);

__attribute__((const))
static inline int dlc_encode(int data_len, int fd);


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

#ifdef CANARD_ENABLE_CANFD
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

	fdcan->CCCR &= ~(1 << 4);						/* Exit from sleep mode */
	while (fdcan->CCCR & (1 << 3));					/* Wait until exited from sleep mode */

	fdcan->CCCR |=  (1 << 0);						/* Get to INIT mode */
	while (!(fdcan->CCCR & (1 << 0)));				/* Wait until init mode sets */

	/* General config */
	fdcan->CCCR |=  (1 << 1) | 						/* CCE=1: config change enable */
					(1 << 6) |						/* DAR=1: automatic retransmission should be disabled
													 * before DroneCAN Node ID is assigned */
					(1 << 13)|						/* EFBI=1: edge filtering enabled */
					(1 << 14);						/* TXP=1: Enable transmit pause -- should be generally beneficial. */


#ifdef CANARD_ENABLE_CANFD
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

	fdcan->XIDAM = 0xFFFF80 << 7;		/* Only consider the DroneCAN Message type ID part in the CAN frame.
										 * This allows to setup each filter element as a dual ID filter
										 * and have 16 message rules in total.
										 * Match only broadcast messages (serviceNotMessage bit
										 * will never match with anything in the filters). */
	/* Timing config */
	/* Set nominal timings */
	fdcan->NBTP = ((nominal_timings.max_resynchronization_jump_width - 1) 	<< 25U) |
				  ((nominal_timings.bit_segment_2 - 1) 						<< 0U) |
				  ((nominal_timings.bit_segment_1 - 1) 						<< 8U) |
				  ((nominal_timings.bit_rate_prescaler - 1)					<< 16U);

#ifdef CANARD_ENABLE_CANFD
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

	driver->fdcan_sram_base = fdcan_ram(fdcan);	/* Precalc this address to save on divisions later on */
	return 0;
}

#define STM32G4_FDCAN_NUM_EXT_FILTER_ELEMENTS 8

int canard_stm32g4fdcan_type_id_filter(canard_stm32g4_fdcan_driver *driver,
										int dronecan_type_id1, int dronecan_type_id2, int accept_not_reject)
{
	for (int i = 0; i < STM32G4_FDCAN_NUM_EXT_FILTER_ELEMENTS; ++i) {
		fdcan_ext_filt_elem *fe = get_ext_filt_elem(driver->fdcan, i);
		if (!filt_elem_active(fe)) {
			if (accept_not_reject) {
				filt_elem_accept_dual_id(fe, dronecan_type_id1, dronecan_type_id2);
			}
			else {
				filt_elem_reject_dual_id(fe, dronecan_type_id1, dronecan_type_id2);
			}
			return 0;
		}
	}
	return -CANARD_ERROR_STM32_FDCAN_OUT_OF_FILTER_SPACE;
}

void canard_stm32g4fdcan_start(canard_stm32g4_fdcan_driver *driver)
{
	fdcan_registers *fdcan = driver->fdcan;
	fdcan->CCCR &= ~((1 << 0) | (1 << 1)); 		/* Clear INIT and CCE */
	while ((fdcan->CCCR & (1 << 0)));	/* Wait until we leave init mode */
}

int canard_stm32g4fdcan_transmit(canard_stm32g4_fdcan_driver *driver, const CanardCANFrame* const frame)
{
	clear_and_handle_faults(driver);
	fdcan_registers *fdcan = driver->fdcan;
	if (fdcan->TXFQS & (1 << 21)) {
		/* TFQF set: TX queue full */
		return 0;
	}
	int put_index = fdcan->TXFQS & (3 << 16) >> 16;
	canard_frame_to_tx_buf_elem(frame, get_tx_buf_element(driver, put_index));
	fdcan->TXBAR = (1 << put_index);
	return 0;
}

int canard_stm32g4fdcan_receive(canard_stm32g4_fdcan_driver *driver, CanardCANFrame* const out_frame)
{
	clear_and_handle_faults(driver);
	fdcan_registers *fdcan = driver->fdcan;
	fdcan_rxfifo_regs *rxfifo[2] = {(fdcan_rxfifo_regs *) &fdcan->RXF0S, (fdcan_rxfifo_regs *) &fdcan->RXF1S};
	int ele_index = -1;
	fdcan_rxfifo_elem *ele = NULL;
	ele = rxfifo_get_first_elem(driver, rxfifo[0], &ele_index);
	if (ele) {
		rxfifo_receive_frame(fdcan, out_frame, ele);
		rxfifo_ack_frame(rxfifo[0], ele_index);
		return 1;
	}
	ele = rxfifo_get_first_elem(driver, rxfifo[1], &ele_index);
	if (ele) {
		rxfifo_receive_frame(fdcan, out_frame, ele);
		rxfifo_ack_frame(rxfifo[1], ele_index);
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

static fdcan_tx_buf_elem *get_tx_buf_element(canard_stm32g4_fdcan_driver *driver, int index)
{
	return (fdcan_tx_buf_elem *) driver->fdcan_sram_base + SRAMCAN_TXB_OFFSET + sizeof(fdcan_tx_buf_elem) * index;
}

static void canard_frame_to_tx_buf_elem(const CanardCANFrame* const frame, fdcan_tx_buf_elem *ele)
{
	/* See Table 408 RM0440 */
	/* 			 ESI = 0   29-bit id   not remote	CAN frame ID */
	ele->t[0] = (0 << 31) | (1 << 30) | (0 << 29) | (frame->id & 0x1FFFFFFF);
#ifdef CANARD_ENABLE_CANFD
	/* don't store events  is this an FD frame    use bit rate switch		DLC                        */
	ele->t[1] = (0 << 23) | (frame->canfd << 21) |  (1 << 20) | (dlc_encode(frame->data_len, frame->canfd));
#else
	ele->t[1] = (0 << 23) | (0 << 21)            |  (0 << 20) | (dlc_encode(frame->data_len, 0));
#endif

	volatile uint32_t *tx_data_element = (volatile uint32_t *) &ele->t[2];
	volatile uint32_t *tx_data_frame = (volatile uint32_t *) frame->data;

	if (frame->data_len) {
		tx_data_element[0] = tx_data_frame[0];
		tx_data_element[1] = tx_data_frame[1];
#ifdef CANARD_ENABLE_CANFD
		for (int i = 2; i < frame->data_len / sizeof(uint32_t); ++i) {
			tx_data_element[i] = tx_data_frame[i];
		}
#endif
	}
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
	if (ir & ((1 << 2) | (1 << 5))) { /* RF0L | RF1L */
		driver->statistics.rx_fifo_overruns++;
	}
	if (ir & (1 << 12))  { /* TEFL */
		driver->statistics.tx_fifo_overruns++;
	}
	fdcan->IR = 0xFFFFFF; /* clear IR */
}

static void rxfifo_receive_frame(fdcan_registers *regs, CanardCANFrame* const out_frame, fdcan_rxfifo_elem *ele)
{
	/* Don't need to check if this is a standard or remote frame (not needed for DroneCAN),
	 * they're rejected by the filter configuration */
	out_frame->id = rxfifo_elem_get_ext_id(ele);
	out_frame->data_len = rxfifo_elem_get_dlc(ele);
	out_frame->iface_id = ((uint32_t) regs - FDCAN1_ADDR) / sizeof(fdcan_registers);
#ifdef CANARD_ENABLE_CANFD
	out_frame->canfd = rxfifo_elem_is_fd_frame(ele);
	CANARD_ASSERT(out_frame->data_len <= 64);
#else
	CANARD_ASSERT(out_frame->data_len <= 8);
#endif
	volatile uint32_t *payload = rxfifo_elem_get_payload(ele);
	if (out_frame->data_len) {
		uint32_t *out_data = (uint32_t *) out_frame->data;
		/* SRAMCAN is accessed in words, not bytes, so no memcpy().
		 * For dlc <= 8, two words are always copied.*/
		if (out_frame->data_len <= 8) {
			out_data[0] = payload[0];
			out_data[1] = payload[1];
		}
		else {
			int limit = out_frame->data_len / sizeof(uint32_t);
			for (int i = 0; i < limit; ++i) {
				out_data[i] = payload[i];
			}
		}
	}
}

__attribute__((const))
static inline uint32_t fdcan_ram(const fdcan_registers *r)
{
	return SRAMCAN_START + ((uint32_t) r - FDCAN1_ADDR) / sizeof(fdcan_registers) * SRAMCAN_SIZE;
}

__attribute__((const))
static inline fdcan_rxfifo_elem *get_rxfifo_elem(const canard_stm32g4_fdcan_driver *driver,
								fdcan_rxfifo_regs *fifo_regs, size_t elemindex)
{
	fdcan_registers *fdcan = driver->fdcan;
	int fifonr = ((uint32_t) fifo_regs) == ((uint32_t) &fdcan->RXF0S) ? 0 : 1;
	return (fdcan_rxfifo_elem *)
			(driver->fdcan_sram_base + SRAMCAN_RXFIFO0_OFFSET 	/* Base address */
					+ SRAMCAN_RXFIFO_SIZE * fifonr 				/* FIFO0 or 1? */
					+ sizeof(fdcan_rxfifo_elem) * elemindex); 	/* Which element */
}

static int rxfifo_elem_is_fd_frame(fdcan_rxfifo_elem *rxf)
{
	return (rxf->r[1] & (1 << 21)) > 0;
}

static int rxfifo_elem_get_ext_id(fdcan_rxfifo_elem *rxf)
{
	return rxf->r[0] & 0x1FFFFFFF;
}

static volatile uint32_t *rxfifo_elem_get_payload(fdcan_rxfifo_elem *rxf)
{
	return &rxf->r[2];
}

static int rxfifo_get_fill_level(fdcan_rxfifo_regs *rxf)
{
	return rxf->RXFxS & 0xF;
}

static int rxfifo_get_read_index(fdcan_rxfifo_regs *rxf)
{
	return (rxf->RXFxS & (3 << 8)) >> 8;
}

static fdcan_rxfifo_elem *rxfifo_get_first_elem(canard_stm32g4_fdcan_driver *driver, fdcan_rxfifo_regs *rxf, int *elem_index)
{
	if (rxfifo_get_fill_level(rxf)) {
		*elem_index = rxfifo_get_read_index(rxf);
		return get_rxfifo_elem(driver, rxf, *elem_index);
	}
	*elem_index = -1;
	return NULL;
}

__attribute__((const))
static inline fdcan_ext_filt_elem* get_ext_filt_elem(const fdcan_registers *r, size_t index)
{
	return (fdcan_ext_filt_elem *) (fdcan_ram(r) + SRAMCAN_EXT_FILT_OFFSET + sizeof(fdcan_ext_filt_elem) * index);
}

static int filt_elem_active(const fdcan_ext_filt_elem * const fe)
{
	return (fe->f0 & (0x7 << 29)) > 0;
}

static void filt_elem_accept_dual_id(fdcan_ext_filt_elem *fe, uint32_t frame_id1, uint32_t frame_id2)
{
	fe->f0 = (1 << EFEC_OFFSET) | frame_id1;
	fe->f1 = (1 << EFT_OFFSET) | frame_id2;
}

static void filt_elem_reject_dual_id(fdcan_ext_filt_elem *fe, uint32_t frame_id1, uint32_t frame_id2)
{
	fe->f0 = (3 << EFEC_OFFSET) | frame_id1;
	fe->f1 = (1 << EFT_OFFSET) | frame_id2;
}

__attribute__((const))
static inline int dlc_decode(int dlc_value, int fd)
{
	if (dlc_value <= 8) {
		return dlc_value;
	}
	if (fd) {
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
	else {
		return 8;
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

static int rxfifo_elem_get_dlc(fdcan_rxfifo_elem *rxf)
{
	return dlc_decode(((0xF << DLC_OFFSET) & rxf->r[1]) >> DLC_OFFSET, rxfifo_elem_is_fd_frame(rxf));
}

static void rxfifo_ack_frame(fdcan_rxfifo_regs *rxfifo_regs, int ack_index)
{
	rxfifo_regs->RXFxA |= 7 & ack_index;
}
