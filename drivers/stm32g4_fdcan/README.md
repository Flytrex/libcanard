# STM32G4 FDCAN driver for libcanard/DroneCAN

This is a bare-bones driver for the variety of FDCAN that's found in STM32G4 series.

* TX FIFO is used in queue mode
* Incoming messages are prioritized based on filters (see below). 
* Multiple instances are supported
* No dependencies, no interrupts (unless you want them)
* Specific for DroneCAN.

FDCANs are slightly different between STM32G4, STM32H7 (this has a very configurable SRAM) and STM32G0. 
So this is only for G4, but can probably be extended to support everything.

## General usage

### Initialization

1. Store a `canard_stm32g4_fdcan_driver` somewhere and set the `fdcan` pointer 
to one of `FDCAN1_ADDR`/`FDCAN2_ADDR`/`FDCAN3_ADDR`, then
call `canard_stm32g4fdcan_init`.

2. Then configure filters by calling `canard_stm32g4fdcan_type_id_filter` as many times as needed (8 double
rules can be created all in all).

3. Call `canard_stm32g4fdcan_start`.

4. After a Node ID has been assigned (or if you know it beforehand), call `canard_stm32g4fdcan_enable_automatic_retransmission`.

It's the user's responsibility to provide clock to the chosen FDCAN peripheral and configure I/O pins.

### Reception

The driver sets up two RX FIFOs, FIFO0 for "more important" and FIFO1 for "less important" messages.

Three things can happen to a message on reception:
1. The DroneCAN data type ID is in the "accept" filter (`canard_stm32fdcan_accept_filter`) -> goes to FIFO0
2. The DroneCAN data type ID is in the "reject" filter (`canard_stm32fdcan_reject_filter`) -> discarded
3. The DroneCAN data type ID is in neither -> goes to FIFO1

`canard_stm32fdcan_recieve` reads one message at a time and prefers FIFO0.

For instance, a ESC 
1. would prioritize `1031.RawCommand` above anything else - so this ID should be accepted.
2. would reject spammy `1034.Status` messages from other ESCs - so this ID should be rejected.

The filter functions are used by DroneCAN data type IDs, not CAN frame IDs.

Note that all service requests will automatically go to FIFO1.

## Config

If `CANARD_ENABLE_CANFD` is defined, FDCAN will be configured in long frame mode + bitrate switch mode.
