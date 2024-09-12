/*
 * canard_stm32g4_fdcan.h
 *
 *  Created on: Sep 10, 2024
 *      Author: grishar
 */

#ifndef CANARD_STM32G4_FDCAN_H_
#define CANARD_STM32G4_FDCAN_H_

#include <stdint.h>

#include <canard.h>

#define CANARD_ERROR_STM32_FDCAN_OUT_OF_FILTER_SPACE -1200

#define FDCAN1_ADDR 0x40006400U
#define FDCAN2_ADDR 0x40006800U
#define FDCAN3_ADDR 0x40006C00U

typedef struct {
	void *fdcan;
	struct {
		uint32_t rx_fifo_overruns;
		uint32_t tx_fifo_overruns;
		uint32_t warning_events;
		uint32_t bus_off_events;
		uint32_t protocol_exceptions;
	} statistics;
	uint32_t fdcan_sram_base;
} canard_stm32g4_fdcan_driver;

/**
 * Initializes the driver and the hardware. Doesn't start FDCAN (you're expected
 * to configure filters with `canard_stm32g4fdcan_type_id_filter` after it).
 * To start the hardware, call canard_stm32g4fdcan_start after it.
 * If CANARD_ENABLE_CANFD is defined, the hardware will be initialized to CAN FD mode
 * (long frame + bit rate switch).
 *
 * @param[in] driver.fdcan		Must be set to FDCAN1_ADDRESS, FDCAN2_ADDRESS or FDCAN3_ADDRESS.
 *   		  					It's the user's responsibility to check if the hardware has them.
 * @param[in] bitrate_bps		Nominal bitrate
 * @param[in] fdbitrate_bps		FD bitrate (ignored if CANARD_ENABLE_CANFD is not defined).
 * @param[in] periph_clock  	The clock rate of the hardware (set to 80 MHz for best results).
 * @retval 0 -- ok
 * @retval negative -- error
 */
int canard_stm32g4fdcan_init(canard_stm32g4_fdcan_driver *driver, int bitrate_bps, int fdbitrate_bps, int periph_clock_rate);

/**
 * Starts the hardware after initialization and configuration.
 */
void canard_stm32g4fdcan_start(canard_stm32g4_fdcan_driver *driver);

/**
 * Creates a filter based on broadcast message IDs. Pass DroneCAN message IDs to create a filter.
 * Due to internal organization, these filters come in pairs: just set to 0 if not needed.
 * The messages that match the filter will either get into FIFO0 or rejected, depending on the value of
 * @param[in] accept_not_reject
 *
 * The messages that don't match the filter end up in FIFO1.
 *
 */
int canard_stm32g4fdcan_type_id_filter(canard_stm32g4_fdcan_driver *driver, int dronecan_type_id1, int dronecan_type_id2, int accept_not_reject);

/**
 * Pushes one frame into the TX buffer, if there is space.
 * Note that proper care is taken to ensure that no inner priority inversion is taking place.
 * This function does never block.
 *
 * @retval      1               Transmitted successfully
 * @retval      0               No space in the buffer
 * @retval      negative        Error
 */
int canard_stm32g4fdcan_transmit(canard_stm32g4_fdcan_driver *driver, const CanardCANFrame* const frame);

/**
 * Reads one frame from the hardware RX FIFO, unless all FIFO are empty.
 * This function does never block.
 *
 * @retval      1               Read successfully
 * @retval      0               The buffer is empty
 * @retval      negative        Error
 */
int canard_stm32g4fdcan_recieve(canard_stm32g4_fdcan_driver *driver, CanardCANFrame* const out_frame);

/**
 *	Reads the instant standard CAN state variables from the hardware.
 */
typedef struct {
	uint8_t tec, rec;
	uint8_t bus_off, error_passive, warning;
	uint8_t delay_comp_value;
} canard_stm32g4fdcan_protocol_state;

void canard_stm32g4fdcan_get_protocol_state(canard_stm32g4_fdcan_driver *driver, canard_stm32g4fdcan_protocol_state *s);


/**
 *	Enables automatic retransmission.
 *	Call this after the node gets its Node ID.
 */
void canard_stm32g4fdcan_enable_automatic_retransmission(canard_stm32g4_fdcan_driver *driver);


#endif /* CANARD_STM32G4_FDCAN_H_ */
