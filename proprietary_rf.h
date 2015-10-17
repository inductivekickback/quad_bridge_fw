#ifndef __PROPRIETARY_H
#define __PROPRIETARY_H

#include "syma_218_bkt.h"

#define TS_UPDATE_INTERVAL_US (SYMA_UPDATE_INTERVAL_US)
#define TS_LEN_US             (1200UL)

/**
 *
 */
void proprietary_rf_init(void);

/**
 *
 */
void timeslot_missed(void);

#endif
