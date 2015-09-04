/******************************************************************************
 * Project      : WaterOptimizer Irrigation System (WOIS)
 * Organization : WaterOptimizer, LLC
 * Module       : debug.h
 * Description  : This file defines debug configuration for use during
 *                development.
 *
 *****************************************************************************/
#ifndef __debug_H
#define __debug_H

#define DEBUG_ENABLED 0

#if DEBUG_ENABLED
  #warning "Debug Build Enabled (DEBUG_ENABLED != 0)"
  #define DEBUG_TIMINGS_ENABLED
#endif // DEBUG_ENABLED

#endif
