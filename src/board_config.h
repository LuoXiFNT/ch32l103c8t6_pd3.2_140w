/**
 * @file board_config.h
 * @brief Build-time selection of the main and secondary board roles.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/*
 * Build role selector.
 *
 * Default build is BOARD_MAIN. Define BOARD_SUB in compiler options to build
 * the secondary PCB firmware from the same source tree.
 */
#if !defined(BOARD_MAIN) && !defined(BOARD_SUB)
#define BOARD_MAIN // Choose whether to build the main board firmware or the sub board firmware.
#endif

#if defined(BOARD_MAIN) && defined(BOARD_SUB)
#error "Define only one board role: BOARD_MAIN or BOARD_SUB"
#endif

#if defined(BOARD_SUB)
#define BOARD_IS_SUB   1
#define BOARD_IS_MAIN  0
#else
#define BOARD_IS_SUB   0
#define BOARD_IS_MAIN  1
#endif

#endif /* BOARD_CONFIG_H */
