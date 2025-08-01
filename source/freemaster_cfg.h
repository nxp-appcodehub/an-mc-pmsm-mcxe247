/*
    * Copyright 2025 NXP
    *
    * SPDX-License-Identifier: BSD-3-Clause
*/
/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/*
 * FreeMASTER Communication Driver - User Configuration File
 */

#ifndef __FREEMASTER_CFG_H
#define __FREEMASTER_CFG_H


#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#define FMSTR_GENERATED_CONFIGURATION    1   // Generated FreeMASTER configuration 

#define FMSTR_PLATFORM_CORTEX_M  1   /* Used platform (see freemaster.h for list of all supported platforms) */

//! Enable/Disable FreeMASTER support as a whole
#define FMSTR_DISABLE           0   // To disable all FreeMASTER functionalities

//! Select interrupt or poll-driven serial communication
#define FMSTR_LONG_INTR         0   // Complete message processing in interrupt
#define FMSTR_SHORT_INTR        0   // Queuing done in interrupt
#define FMSTR_POLL_DRIVEN       1   // No interrupt needed, polling only

//! Select communication interface
#define FMSTR_TRANSPORT         FMSTR_SERIAL  // Select FMSTR_SERIAL, FMSTR_CAN or FMSTR_PDBDM transport interface
#define FMSTR_SERIAL_DRV        FMSTR_SERIAL_MCUX_LPUART   // When using FMSTR_SERIAL: select Serial low-level communication driver

//! Define communication interface base address or leave undefined for runtime setting
#define FMSTR_SERIAL_BASE       LPUART2

//! Input/output communication buffer size
#define FMSTR_COMM_BUFFER_SIZE  0   // Set to 0 for "automatic"

//! Receive FIFO queue size (use with FMSTR_SHORT_INTR only)
#define FMSTR_COMM_RQUEUE_SIZE  0  // Set to 0 for "default"

//! Support for Application Commands
#define FMSTR_USE_APPCMD 1        //!< Enable/disable App.Commands support
#define FMSTR_APPCMD_BUFF_SIZE 32 //!< App.Command data buffer size
#define FMSTR_MAX_APPCMD_CALLS 4  //!< How many app.cmd callbacks? (0=disable)

//! Oscilloscope support
#define FMSTR_USE_SCOPE         1   // Specify number of supported oscilloscopes
#define FMSTR_MAX_SCOPE_VARS    8   // Specify maximum number of scope variables per one oscilloscope

//! Recorder support
#define FMSTR_USE_RECORDER 1 //!< Specify number of supported recorders

//! Built-in recorder buffer
#define FMSTR_REC_BUFF_SIZE 2048 //!< Built-in buffer size. Set to zero to disable using embedded buffer for recorder 0.

//! Recorder time base, specifies how often the recorder is called in the user app.
#define FMSTR_REC_TIMEBASE FMSTR_REC_BASE_NANOSEC(100000) //!< 0 = "unknown"
#define FMSTR_REC_FLOAT_TRIG 1                        //!< Enable/disable floating point triggering

// Target-side address translation (TSA)
#define FMSTR_USE_TSA           0   // Enable TSA functionality
#define FMSTR_USE_TSA_INROM     0   // TSA tables declared as const (put to ROM)
#define FMSTR_USE_TSA_SAFETY    0   // Enable/Disable TSA memory protection
#define FMSTR_USE_TSA_DYNAMIC   0   // Enable/Disable TSA entries to be added also in runtime

// Pipes as data streaming over FreeMASTER protocol
#define FMSTR_USE_PIPES         0   // Specify number of supported pipe objects

// Enable/Disable read/write memory commands
#define FMSTR_USE_READMEM       1   // Enable read memory commands
#define FMSTR_USE_WRITEMEM      1   // Enable write memory commands
#define FMSTR_USE_WRITEMEMMASK  1   // Enable write memory bits commands

////////////////////////////////////////////////////////////////////////////////
// Debugging options
////////////////////////////////////////////////////////////////////////////////

#define FMSTR_DEBUG_LEVEL       0  // Driver debugging print level (0=none, 1=errors, 2=normal, 3=verbose)

// Debug-TX mode is a special mode used to test or debug the data transmitter. Our driver
// will be sending test frames periodically until a first valid command is received from the
// PC Host. You can hook a logic analyzer to transmission pins to determine port and baudrate.
// Or you can connect the FreeMASTER tool and run the connection wizard to listen for the
// dummy frames.
#define FMSTR_DEBUG_TX          0

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* __FREEMASTER_CFG_H */

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
