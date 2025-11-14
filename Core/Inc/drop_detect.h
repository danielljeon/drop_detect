/*******************************************************************************
 * @file drop_detect.h
 * @brief IMU based drop detection compute and state machine.
 *******************************************************************************
 */

#ifndef DROP_DETECT_H
#define DROP_DETECT_H

/** Definitions. **************************************************************/

#define DROP_DETECT_USE_STATUS_LED // Define to enable PWM LED status.

/** Public functions. *********************************************************/

void compute_drop_detect(void);

#endif
