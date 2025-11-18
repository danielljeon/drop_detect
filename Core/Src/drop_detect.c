/*******************************************************************************
 * @file drop_detect.c
 * @brief IMU based drop detection compute and state machine.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "drop_detect.h"
#include "bno085_runner.h"
#include "ws2812b_hal_pwm.h"
#include <math.h>
#include <stdbool.h>

/** Definitions. **************************************************************/

#define G_CONSTANT 9.80665f     // m/s^2.
#define FREEFALL_G_THRESH 0.25f // < 0.25 g means possible free-fall.
#define IMPACT_G_THRESH 3.5f    // > 3.5 g means impact detected.
#define EXIT_G_THRESH 0.40f     // Cancels candidate if above.
#define MIN_FREE_FALL_SAMPLES 4 // 4 samples at 200 Hz = 20 ms.
#define MIN_RESET_SAMPLES 20    // 20 samples at 200 Hz = 100 ms.
#define TICKS_TILL_RESET 400    // 400 samples at 200 Hz = 2000 ms = 2 s.

/** Private types. ************************************************************/

typedef enum {
  FREE_FALL_IDLE = 0,
  FREE_FALL_LATCHED = 1,
  FREE_FALL_DELATCH = 2,
  PENDING_IMPACT = 3,
  IMPACT_LANDED = 4,
  DROP_DETECT_RESET = 5,
} free_fall_state_t;

/** Private variables. ********************************************************/

static free_fall_state_t detect_state = FREE_FALL_IDLE;
static int low_g_count = 0;
static int trigger_count = 0;
static int pending_reset_count = 0;

/** Public functions. *********************************************************/

void compute_drop_detect(void) {
  const float a_mag = sqrtf(
      bno085_accel_x * bno085_accel_x + bno085_accel_y * bno085_accel_y +
      bno085_accel_z * bno085_accel_z); // Acceleration magnitude in m/s^2.

  const float g_mag = a_mag / G_CONSTANT; // Convert m/s^2 to gs.

  bool accel_low = false;

  // State machine switch case.
  switch (detect_state) {

  case FREE_FALL_IDLE:
    // Active potential free-falling condition candidate (low acceleration).
    accel_low = g_mag < FREEFALL_G_THRESH;

    // Acceleration within (positive) detection margins.
    if (accel_low) {
      low_g_count++;

      // Detection instances surpass positive detection count.
      if (low_g_count >= MIN_FREE_FALL_SAMPLES) {

        // Fall now detected.
        // Trigger GPIO high.
        HAL_GPIO_WritePin(DROP_DETECT_GPIO_PORT, DROP_DETECT_GPIO_PIN,
                          GPIO_PIN_SET);

#ifdef DROP_DETECT_USE_STATUS_LED
        ws2812b_set_colour(0, 3, 0, 0); // Red.
        ws2812b_update();
#endif

        detect_state = FREE_FALL_LATCHED; // Enable next state.
      }

    } else {
      // Acceleration outside detection margins (negative condition).
      // Reset detection instance counter.
      low_g_count = 0;
    }
    break;

  case FREE_FALL_LATCHED:
    // Delay for the minimum trigger time.
    trigger_count++;
    if (trigger_count >= MIN_RESET_SAMPLES) {
      detect_state = FREE_FALL_DELATCH;
    }
    break;

  case FREE_FALL_DELATCH:
    // Trigger GPIO low.
    HAL_GPIO_WritePin(DROP_DETECT_GPIO_PORT, DROP_DETECT_GPIO_PIN,
                      GPIO_PIN_RESET);

    detect_state = PENDING_IMPACT;
    pending_reset_count = 0;
    break;

  case PENDING_IMPACT:
    // Impact detection.
    if (g_mag > IMPACT_G_THRESH) {

#ifdef DROP_DETECT_USE_STATUS_LED
      ws2812b_set_colour(0, 0, 3, 0); // Green.
      ws2812b_update();
#endif

      detect_state = IMPACT_LANDED;
      pending_reset_count = 0;
    }

    // Check for drop detect full system reset condition.
    pending_reset_count++;
    if (pending_reset_count >= TICKS_TILL_RESET) {
      detect_state = DROP_DETECT_RESET;
    }
    break;

  case IMPACT_LANDED:
    // Wait for fix time until reset.
    pending_reset_count++;
    if (pending_reset_count >= TICKS_TILL_RESET) {
      detect_state = DROP_DETECT_RESET;
    }
    break;

  case DROP_DETECT_RESET:
    ws2812b_set_colour(0, 2, 1, 3); // Init colour.
    ws2812b_update();

    trigger_count = 0;
    pending_reset_count = 0;
    low_g_count = 0;
    detect_state = FREE_FALL_IDLE;
    break;

  default:
    low_g_count = 0;
    detect_state = FREE_FALL_IDLE;
    break;
  }
}
