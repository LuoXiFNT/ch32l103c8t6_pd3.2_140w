/**
 * @file board_link.h
 * @brief Public interface for exchanging measurements and status between boards.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#ifndef BOARD_LINK_H
#define BOARD_LINK_H

#include "debug.h"

/** @brief Measurement and status payload for one power channel. */
typedef struct {
    u16 vbus_raw; /**< Raw INA226 bus-voltage register value. */
    u16 curr_raw; /**< Raw INA226 current register value. */
    u16 pwr_raw;  /**< Raw INA226 power register value. */
    u8 fault;     /**< Non-zero when the channel is faulted. */
    u8 protocol;  /**< Active board-link protocol identifier. */
} board_link_channel_t;

/** @brief Protocol identifiers encoded in a channel payload. */
typedef enum {
    BOARD_LINK_PROTOCOL_NONE = 0U, /**< No valid connection. */
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
    u16 fault_bits;         /**< Aggregated remote fault bits. */
    u8 seq;                 /**< Sequence number of the latest frame. */
    u8 valid;                /**< Non-zero after at least one valid frame. */
} board_link_remote_t;

/** @brief Diagnostic counters for the UART board link. */
typedef struct {
    u32 rx_frame_ok;
    u32 rx_checksum_error;
    u32 rx_header_error;
    u32 rx_resync;
    u32 rx_usart_error;
    u32 rx_ore;
    u32 rx_ne;
    u32 rx_fe;
    u32 tx_frame_start;
    u32 tx_frame_done;
    u32 tx_busy_drop;
} board_link_stats_t;

/** @brief Advance board-link timers. @param elapsed_ms Elapsed milliseconds. */
void BoardLink_TickMs(u16 elapsed_ms);
/** @brief Run the periodic board-link transmit task. */
void BoardLink_Task(void);
/** @brief Service the USART1 interrupt used by the board link. */
void BoardLink_USART1_IRQHandler(void);

/** @brief Store local channel data for the next transmitted frame. */
void BoardLink_SetLocalData(const board_link_channel_t *c,
                            const board_link_channel_t *a,
                            u16 fault_bits);
/** @brief Set the local display fault flags for channels C and A. */
void BoardLink_SetFaults(u8 fault_c, u8 fault_a);
/** @brief Build local channel snapshots from PD and INA226 services. */
void BoardLink_BuildChannels(board_link_channel_t *c,
                             board_link_channel_t *a);
/** @brief Copy the latest remote board data into the caller's structure. */
void BoardLink_GetRemoteData(board_link_remote_t *remote);
/** @brief Copy board-link diagnostic counters. */
void BoardLink_GetStats(board_link_stats_t *stats);

#endif /* BOARD_LINK_H */
