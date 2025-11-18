/*******************************************************************************
 * @file drop_detect.h
 * @brief IMU based drop detection compute and state machine.
 *******************************************************************************
 */

#ifndef DROP_DETECT_H
#define DROP_DETECT_H

/** STM32 port and pin configs. ***********************************************/

#define DROP_DETECT_GPIO_PORT GPIOC
#define DROP_DETECT_GPIO_PIN GPIO_PIN_15

/** Definitions. **************************************************************/

#define DROP_DETECT_USE_STATUS_LED // Define to enable PWM LED status.

/** Public functions. *********************************************************/

void compute_drop_detect(void);

#endif
