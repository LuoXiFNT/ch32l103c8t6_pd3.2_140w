/**
 * @file board_link.h
 * @brief Public interface for exchanging measurements and status between boards.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#ifndef BOARD_LINK_H
#define BOARD_LINK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Measurement and status payload for one power channel. */
typedef struct {
    uint16_t vbus_raw; /**< Raw INA226 bus-voltage register value. */
    uint16_t curr_raw; /**< Raw INA226 current register value. */
    uint16_t pwr_raw;  /**< Raw INA226 power register value. */
    uint8_t fault;     /**< Non-zero when the channel is faulted. */
    uint8_t protocol;  /**< Active board-link protocol identifier. */
} board_link_channel_t;

/** @brief Protocol identifiers encoded in a channel payload. */
typedef enum {
    BOARD_LINK_PROTOCOL_NONE = 0u, /**< No valid connection. */
    BOARD_LINK_PROTOCOL_TYPEC,     /**< Type-C connection without a contract. */
    BOARD_LINK_PROTOCOL_SPR_FIXED, /**< USB PD SPR fixed supply. */
    BOARD_LINK_PROTOCOL_EPR_FIXED, /**< USB PD EPR fixed supply. */
    BOARD_LINK_PROTOCOL_SPR_PPS,   /**< USB PD SPR programmable supply. */
    BOARD_LINK_PROTOCOL_SPR_AVS,   /**< USB PD SPR adjustable supply. */
    BOARD_LINK_PROTOCOL_EPR_AVS,   /**< USB PD EPR adjustable supply. */
    BOARD_LINK_PROTOCOL_BC12        /**< Legacy BC1.2 supply. */
} board_link_protocol_t;

/** @brief Latest data received from the opposite board. */
typedef struct {
    board_link_channel_t c; /**< C-channel measurements and status. */
    board_link_channel_t a; /**< A-channel measurements and status. */
    uint16_t fault_bits;         /**< Aggregated remote fault bits. */
    uint8_t seq;                 /**< Sequence number of the latest frame. */
    bool valid;                  /**< A valid frame has been received. */
} board_link_remote_t;

/** @brief Diagnostic counters for the UART board link. */
typedef struct {
    uint32_t rx_frame_ok;       /**< Frames that passed checksum and header checks. */
    uint32_t rx_checksum_error; /**< Frames rejected by the checksum. */
    uint32_t rx_header_error;   /**< Frames with a wrong version or payload length. */
    uint32_t rx_resync;         /**< Times the parser resynchronised inside a frame. */
    uint32_t rx_usart_error;    /**< USART overrun/framing/noise interrupts. */
    uint32_t rx_ore;            /**< Overrun errors. */
    uint32_t rx_ne;             /**< Noise errors. */
    uint32_t rx_fe;             /**< Framing errors. */
    uint32_t tx_frame_start;    /**< Frames handed to the transmitter. */
    uint32_t tx_frame_done;     /**< Frames completely shifted out. */
    uint32_t tx_busy_drop;      /**< New frames skipped because the previous was busy. */
} board_link_stats_t;

/** @brief Advance board-link timers. @param elapsed_ms Elapsed milliseconds. */
void BoardLink_TickMs(uint16_t elapsed_ms);
/** @brief Run the periodic board-link transmit task. */
void BoardLink_Task(void);
/** @brief Service the USART1 interrupt used by the board link. */
void BoardLink_USART1_IRQHandler(void);

/** @brief Store local channel data for the next transmitted frame. */
void BoardLink_SetLocalData(const board_link_channel_t *c,
                            const board_link_channel_t *a,
                            uint16_t fault_bits);
/** @brief Set the local display fault flags for channels C and A. */
void BoardLink_SetFaults(uint8_t fault_c, uint8_t fault_a);
/** @brief Build local channel snapshots from PD and INA226 services. */
void BoardLink_BuildChannels(board_link_channel_t *c,
                             board_link_channel_t *a);
/** @brief Copy the latest remote board data into the caller's structure. */
void BoardLink_GetRemoteData(board_link_remote_t *remote);

/**
 * @brief Report whether the other board is still sending frames.
 *
 * A frame that stops arriving is treated as a disconnect: the last snapshot stays in
 * memory but must not be presented as live data. The check is time based, so it does
 * not depend on the frame counter having been read in between.
 *
 * @return uint8_t 1 while the remote board is online, 0 on the sub board.
 */
bool BoardLink_RemoteOnline(void);

/** @brief Copy board-link diagnostic counters. */
void BoardLink_GetStats(board_link_stats_t *stats);


#ifdef __cplusplus
}
#endif

#endif /* BOARD_LINK_H */
