#ifndef REGULATOR_H
#define REGULATOR_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// ESP-IDF headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_adc/adc_oneshot.h"

#include "driver/ledc.h"

// Regulator parameters
#define DEFAULT_SETPOINT 30.0f          // Default setpoint value for the regulator (celsius)
#define DEFAULT_DEADBAND 1.0f           // Default deadband for the regulator (celsius)
#define REGULATOR_INTERVAL_MS 500       // Interval for the regulator control loop (milliseconds)
#define REGULATOR_MAX_TEMPERATURE 50.0f // Hard safety cutoff temperature (celsius)

// Pin definitions
#define COOLING_PWM_PIN 27
#define COOLING_PWM_CHANNEL LEDC_CHANNEL_0
#define HEATING_PWM_PIN 25
#define HEATING_PWM_CHANNEL LEDC_CHANNEL_1
#define NTC_ADC_UNIT ADC_UNIT_1
#define NTC_ADC_CHANNEL ADC_CHANNEL_0
#define NTC_ADC_ATTEN ADC_ATTEN_DB_12   // ~0-3.1V input range on most ESP32 variants
#define NTC_ADC_BITWIDTH ADC_BITWIDTH_DEFAULT

// PWM configuration
#define PWM_FREQUENCY_HZ 1000 // Frequency of the PWM signal (Hz)
#define PWM_BIT_WIDTH 12      // Bit width of the PWM timer
#define PWM_MAX_DUTY ((1u << PWM_BIT_WIDTH) - 1u)

// NTC readout parameters
#define NTC_READOUT_INTERVAL_MS 100 // Interval for NTC readout (milliseconds)
#define NTC_READOUT_QUEUE_SIZE 16   // Size of the moving average window
#define NTC_ADC_VREF_V 3.3f         // Approximate supply/reference voltage - adjust to your board
#define NTC_ADC_MAX_RAW 4095.0f     // 2^PWM... no: 2^12 - 1 for the default 12-bit ADC width

// Two-point linear calibration (voltage -> temperature)
// Measured these two points by hand with a thermometer + multimeter on the NTC divider.
#define NTC_CAL_VOLTAGE1 0.758f
#define NTC_CAL_VOLTAGE2 1.4f
#define NTC_CAL_TEMP1 23.6f
#define NTC_CAL_TEMP2 35.6f

/*
 * ---- Ziegler-Nichols hand-tuned PID gains: open-loop reaction-curve method ----
 *
 * Since the heater can only heat and the cooler can only cool, the classic
 * closed-loop (sustained oscillation) method doesn't apply cleanly. Instead,
 * this uses the open-loop step-response method.
 *
 *   K (process gain)   = (final_temp - baseline_temp) / step_duty_percent
 *   L (dead time)      = apparent delay before the temperature starts moving
 *   T (time constant)  = how long it then takes to complete about 63% of the response
 *
 * Do this once with a heater step (cooler off) and once with a cooler step
 * (heater off) - fill in the six numbers below - then:
 *   Kp = 1.2 * T / (K * L)
 *   Ti = 2 * L        ->  Ki = Kp / Ti
 *   Td = 0.5 * L      ->  Kd = Kp * Td
 *
 * For more information, see: https://blog.opticontrols.com/ziegler-nichols-tuning-rules/
 */

// Calibrated values for the heater side of the split-range control. 
#define PROCESS_GAIN_HOT 0.6598    // K, degrees C per % heater duty
#define DEAD_TIME_HOT_S 12.0       // L, seconds 
#define TIME_CONST_HOT_S 301.0     // T, seconds

#define PID_KP_HOT (1.2 * TIME_CONST_HOT_S / (PROCESS_GAIN_HOT * DEAD_TIME_HOT_S))
#define PID_TI_HOT (2.0 * DEAD_TIME_HOT_S)
#define PID_KI_HOT (PID_KP_HOT / PID_TI_HOT)
#define PID_TD_HOT (0.5 * DEAD_TIME_HOT_S)
#define PID_KD_HOT (PID_KP_HOT * PID_TD_HOT)

#define DEFAULT_COOLER_DUTY 100 // % duty to apply to the cooler during normal operation (bang-bang logic, on/off)

// --- Tuning-task parameters (regulator_pid_tune_task) ---
#define TUNE_HEATER_DUTY 10             // Step duty applied to heater during tuning (0-100 %)
#define TUNE_LOG_INTERVAL_MS 1000       // How often to print a temperature log line
#define TUNE_SETTLE_WINDOW_MS 60000     // Window over which we check whether temp has stopped moving
#define TUNE_SETTLE_EPSILON_C 0.1       // Max change (C) over that window to call it "settled"
#define TUNE_TIMEOUT_MS 600000          // Give up on a single step test after this long (10 min)
#define TUNE_COOLDOWN_MS 90000          // Pause between the heater test and the cooler test

// Generic PID structure, now carrying its own integrator/derivative state
// and output clamp so the anti-windup logic has somewhere to live.
typedef struct PID
{
    double kp;
    double ki;
    double kd;
    double integral;   // Accumulated integral term (clamped by anti-windup)
    double prev_error; // Previous error, used for the derivative term
    double out_min;    // Output clamp - lower bound (e.g. 0)
    double out_max;    // Output clamp - upper bound (e.g. 100)
} PID_t;

typedef struct ntc_readout
{
    double input_temperature[NTC_READOUT_QUEUE_SIZE]; // Circular buffer of recent readings
    size_t index;                                     // Next write position in the circular buffer
    bool buffer_filled;                               // Whether the buffer has wrapped at least once
    SemaphoreHandle_t lock;                           // Mutex protecting this struct
} ntc_readout_t;

// Task-specific regulator structure, for temperature control via a heater
// and a fan/cooler, driven from opposite sides of the same setpoint.
typedef struct regulator
{
    PID_t pid_hot;             // PID parameters + state for heating
                               // no PID parameters for cooling, works on bang-bang logic (on/off) 
    ntc_readout_t ntc_readout; // NTC readout structure
    double setpoint;           // Desired target value (celsius)
    int output_hot;            // Last commanded heater output, 0-100 (%)
    int output_cold;           // Last commanded cooler output, 0-100 (%)
    double deadband;           // Deadband around the setpoint for split-range control (celsius)
    SemaphoreHandle_t lock;    // Mutex protecting the regulator structure
} regulator_t;

typedef struct regulator_args
{
    regulator_t *regulator;               // Pointer to the regulator structure
    adc_oneshot_unit_handle_t adc_handle; // ADC handle for reading NTC values
} regulator_args_t;

/**
 * @brief Initializes the regulator: ADC unit/channel, LEDC PWM outputs for
 * the heater and cooler, and the PID gains derived from the ZN defines above.
 */
void regulator_init(regulator_args_t *reg_args);

/**
 * @brief Gets the values of the setpoint and deadband from the regulator structure. This is a convenience function for the web server to call.
 */
void regulator_get_params(regulator_t *reg, double *setpoint, double *deadband);

/** 
 * @brief Sets the regulator setpoint and deadband. This is a convenience function for the web server to call.
 */
void regulator_set_params(regulator_t *reg, double setpoint, double deadband);

/**
 * @brief Main control loop task. Reads the averaged NTC temperature, runs
 * split-range control (heater PID, cooler on/off, never both), and drives the
 * corresponding LEDC PWM output. Includes anti-windup and a hard safety
 * cutoff at REGULATOR_MAX_TEMPERATURE.
 */
void regulator_task(void *args);

/**
 * @brief Open-loop step-response tuning task. Drives the heater to
 * TUNE_HEATER_DUTY, logs the temperature curve (tagged "TUNE,HEATER,...")
 * until it settles or times out. Read K/L/T off the logged
 * curves (see the comment block above) and fill them into the
 * PROCESS_GAIN_*\DEAD_TIME_*\TIME_CONST_* defines in this header.
 *
 * Do not run this at the same time as regulator_task - they'll fight over
 * the same PWM outputs. In main.c, run only one of the two at a time; once
 * you're done tuning, comment this task out and switch back to
 * regulator_task.
 */
void regulator_pid_tune_task(void *args);

/**
 * @brief Initializes the NTC readout: configures the ADC channel and
 * creates the mutex protecting the moving-average buffer.
 */
bool ntc_readout_init(regulator_args_t *reg_args);

/**
 * @brief Periodically samples the NTC, converts it to a temperature via the
 * two-point linear calibration, and pushes it into the moving-average buffer.
 */
void ntc_readout_task(void *args);

#endif // REGULATOR_H