// SPDX-License-Identifier: GPL-2.0-only
/* This file is part of lide.device
 * Copyright (C) 2023 Matthew Harlum <matt@harlum.net>
 */
#define DBG_INFO  1
#define DBG_WARN  2
#define DBG_TRACE 4
#define DBG_CMD   8
#define DBG_CLI   16

#if DEBUG
    #if DEBUG & DBG_CLI
        #include <stdio.h>
    #else
        #include <clib/debug_protos.h>
    #endif
#endif

#if DEBUG & DBG_INFO
    #if DEBUG & DBG_CLI
        #define Info printf
    #else
        #define Info KPrintF
    #endif
#else
    #define Info
#endif

#if DEBUG & DBG_WARN
    #if DEBUG & DBG_CLI
        #define Warn printf
    #else
        #define Warn KPrintF
    #endif
#else
    #define Warn
#endif

#if DEBUG & DBG_TRACE
    #if DEBUG & DBG_CLI
        #define Trace printf
    #else
        #define Trace KPrintF
    #endif
#else
    #define Trace
#endif

#if DEBUG & DBG_CMD
    #if DEBUG & DBG_CLI
        #define traceCommand
    #else
        void traceCommand(struct IOStdReq *req);
    #endif
#else
    #define traceCommand
#endif
