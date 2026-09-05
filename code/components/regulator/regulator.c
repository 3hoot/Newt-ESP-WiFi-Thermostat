#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <math.h>
#include <float.h>

// ESP-IDF headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"

// User defined headers
#include "regulator.h"

static const char *TAG = "regulator";

// ---------------------------------------------------------------------------
// PID helpers
// ---------------------------------------------------------------------------

static void pid_reset(PID_t *pid)
{
    pid->integral = 0.0;
    pid->prev_error = 0.0;
}

/**
 * @brief Compute one PID step with conditional-integration anti-windup.
 *
 * The integral term only accumulates further in the direction that would
 * saturate the output; if the output is already pinned at out_min/out_max
 * and the new error would push it further past that limit, the integral is
 * left alone for this step. If the error would instead pull the output back
 * into range, integration resumes immediately. This avoids the classic
 * windup problem (integral ballooning while the actuator is saturated,
 * causing a big overshoot once the error finally reverses).
 *
 * @param pid   PID instance (gains + persistent state)
 * @param error setpoint - measurement (already sign-adjusted by the caller)
 * @param dt    time step in seconds
 * @return clamped output, in [pid->out_min, pid->out_max]
 */
static double pid_update(PID_t *pid, double error, double dt)
{
    double proportional = pid->kp * error;
    double tentative_integral = pid->integral + error * dt;
    double integral_term = pid->ki * tentative_integral;
    double derivative = (dt > 0.0) ? pid->kd * (error - pid->prev_error) / dt : 0.0;

    double output = proportional + integral_term + derivative;

    if (output > pid->out_max)
    {
        output = pid->out_max;
        // Only keep integrating if it would pull the output back down.
        if (error < 0.0)
        {
            pid->integral = tentative_integral;
        }
    }
    else if (output < pid->out_min)
    {
        output = pid->out_min;
        // Only keep integrating if it would pull the output back up.
        if (error > 0.0)
        {
            pid->integral = tentative_integral;
        }
    }
    else
    {
        pid->integral = tentative_integral;
    }

    pid->prev_error = error;
    return output;
}

// ---------------------------------------------------------------------------
// NTC readout
// ---------------------------------------------------------------------------

bool ntc_readout_init(regulator_args_t *reg_args)
{
    ntc_readout_t *ntc = &reg_args->regulator->ntc_readout;

    memset(ntc->input_temperature, 0, sizeof(ntc->input_temperature));
    ntc->index = 0;
    ntc->buffer_filled = false;

    ntc->lock = xSemaphoreCreateMutex();
    if (ntc->lock == NULL)
    {
        ESP_LOGE(TAG, "Failed to create NTC readout mutex");
        return false;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = NTC_ADC_ATTEN,
        .bitwidth = NTC_ADC_BITWIDTH,
    };
    esp_err_t res = adc_oneshot_config_channel(reg_args->adc_handle, NTC_ADC_CHANNEL, &chan_cfg);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(res));
        return false;
    }

    ESP_LOGI(TAG, "NTC readout initialized");
    return true;
}

/**
 * @brief Convert a raw ADC reading to a temperature using the two-point
 * linear calibration (voltage -> temperature) defined in regulator.h.
 *
 * This intentionally does NOT use adc_cali_handle_t / the ESP-IDF ADC
 * calibration scheme - just a fixed Vref assumption and a hand-measured
 * two-point line, since that's all this application needs.
 */
static double raw_to_temperature(int raw)
{
    double voltage = (raw / NTC_ADC_MAX_RAW) * NTC_ADC_VREF_V;
    double slope = (NTC_CAL_TEMP2 - NTC_CAL_TEMP1) / (NTC_CAL_VOLTAGE2 - NTC_CAL_VOLTAGE1);
    return NTC_CAL_TEMP1 + (voltage - NTC_CAL_VOLTAGE1) * slope;
}

void ntc_readout_task(void *args)
{
    regulator_args_t *reg_args = (regulator_args_t *)args;
    ntc_readout_t *ntc = &reg_args->regulator->ntc_readout;

    while (true)
    {
        int raw = 0;
        esp_err_t res = adc_oneshot_read(reg_args->adc_handle, NTC_ADC_CHANNEL, &raw);
        if (res == ESP_OK)
        {
            double temperature = raw_to_temperature(raw);

            if (xSemaphoreTake(ntc->lock, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                ntc->input_temperature[ntc->index] = temperature;
                ntc->index = (ntc->index + 1) % NTC_READOUT_QUEUE_SIZE;
                if (ntc->index == 0)
                {
                    ntc->buffer_filled = true;
                }
                xSemaphoreGive(ntc->lock);
            }
            else
            {
                ESP_LOGW(TAG, "Could not acquire NTC lock, dropping sample");
            }
        }
        else
        {
            ESP_LOGW(TAG, "ADC read failed: %s", esp_err_to_name(res));
        }

        vTaskDelay(pdMS_TO_TICKS(NTC_READOUT_INTERVAL_MS));
    }
}

/**
 * @brief Average the samples currently in the moving-average buffer.
 * Returns false (and leaves *out untouched) if no samples are available yet.
 */
static bool get_average_temperature(ntc_readout_t *ntc, double *out)
{
    bool have_data = false;

    if (xSemaphoreTake(ntc->lock, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        size_t count = ntc->buffer_filled ? NTC_READOUT_QUEUE_SIZE : ntc->index;
        if (count > 0)
        {
            double sum = 0.0;
            for (size_t i = 0; i < count; i++)
            {
                sum += ntc->input_temperature[i];
            }
            *out = sum / (double)count;
            have_data = true;
        }
        xSemaphoreGive(ntc->lock);
    }

    return have_data;
}

// ---------------------------------------------------------------------------
// PWM (LEDC) setup
// ---------------------------------------------------------------------------

static void pwm_channel_init(ledc_channel_t channel, int gpio_pin, ledc_timer_t timer)
{
    ledc_channel_config_t channel_cfg = {
        .gpio_num = gpio_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
}

static void set_pwm_duty_percent(ledc_channel_t channel, int percent)
{
    if (percent < 0)
        percent = 0;
    if (percent > 100)
        percent = 100;

    uint32_t duty = (uint32_t)((percent / 100.0) * PWM_MAX_DUTY);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

// ---------------------------------------------------------------------------
// Regulator init + control loop
// ---------------------------------------------------------------------------

void regulator_init(regulator_args_t *reg_args)
{
    regulator_t *reg = reg_args->regulator;

    // --- ADC unit ---
    adc_oneshot_unit_init_cfg_t adc_init_cfg = {
        .unit_id = NTC_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_cfg, &reg_args->adc_handle));

    if (!ntc_readout_init(reg_args))
    {
        ESP_LOGE(TAG, "NTC readout init failed, aborting regulator init");
        return;
    }

    // --- LEDC timer shared by both PWM channels ---
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = PWM_BIT_WIDTH,
        .freq_hz = PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    pwm_channel_init(HEATING_PWM_CHANNEL, HEATING_PWM_PIN, LEDC_TIMER_0);
    pwm_channel_init(COOLING_PWM_CHANNEL, COOLING_PWM_PIN, LEDC_TIMER_0);
    set_pwm_duty_percent(HEATING_PWM_CHANNEL, 0);
    set_pwm_duty_percent(COOLING_PWM_CHANNEL, 0);

    // --- PID gains, from the hand-tuned Ziegler-Nichols defines ---
    reg->pid_hot = (PID_t){
        .kp = PID_KP_HOT,
        .ki = PID_KI_HOT,
        .kd = PID_KD_HOT,
        .integral = 0.0,
        .prev_error = 0.0,
        .out_min = 0.0,
        .out_max = 100.0,
    };
    reg->pid_cold = (PID_t){
        .kp = PID_KP_COLD,
        .ki = PID_KI_COLD,
        .kd = PID_KD_COLD,
        .integral = 0.0,
        .prev_error = 0.0,
        .out_min = 0.0,
        .out_max = 100.0,
    };

    reg->setpoint = DEFAULT_SETPOINT;
    reg->output_hot = 0;
    reg->output_cold = 0;

    ESP_LOGI(TAG, "Regulator initialized. Setpoint = %.1f C", reg->setpoint);
    ESP_LOGI(TAG, "PID hot:  Kp=%.4f Ki=%.4f Kd=%.4f", reg->pid_hot.kp, reg->pid_hot.ki, reg->pid_hot.kd);
    ESP_LOGI(TAG, "PID cold: Kp=%.4f Ki=%.4f Kd=%.4f", reg->pid_cold.kp, reg->pid_cold.ki, reg->pid_cold.kd);
}

void regulator_task(void *args)
{
    regulator_args_t *reg_args = (regulator_args_t *)args;
    regulator_t *reg = reg_args->regulator;
    const double dt = REGULATOR_INTERVAL_MS / 1000.0;

    while (true)
    {
        double avg_temp;
        if (!get_average_temperature(&reg->ntc_readout, &avg_temp))
        {
            // No samples yet - stay safe, do nothing this cycle.
            ESP_LOGW(TAG, "No NTC samples yet, skipping control step");
            vTaskDelay(pdMS_TO_TICKS(REGULATOR_INTERVAL_MS));
            continue;
        }

        // Hard safety cutoff, independent of the PID logic: never allow the
        // heater to run above the max temperature, no matter what the
        // controller computed.
        if (avg_temp >= REGULATOR_MAX_TEMPERATURE)
        {
            ESP_LOGE(TAG, "Temperature %.2f C >= max %.2f C, forcing heater off and cooler on",
                     avg_temp, REGULATOR_MAX_TEMPERATURE);
            reg->output_hot = 0;
            reg->output_cold = 100;
            pid_reset(&reg->pid_hot);
            pid_reset(&reg->pid_cold);
            set_pwm_duty_percent(HEATING_PWM_CHANNEL, reg->output_hot);
            set_pwm_duty_percent(COOLING_PWM_CHANNEL, reg->output_cold);
            vTaskDelay(pdMS_TO_TICKS(REGULATOR_INTERVAL_MS));
            continue;
        }

        double error = reg->setpoint - avg_temp;

        // Split-range control: only one of heater/cooler is ever active.
        // The idle side's PID is reset so it doesn't wind up while unused,
        // and so we get a bumpless start when control switches sides.
        if (error > 0.0)
        {
            double output = pid_update(&reg->pid_hot, error, dt);
            pid_reset(&reg->pid_cold);
            reg->output_hot = (int)output;
            reg->output_cold = 0;
        }
        else
        {
            double output = pid_update(&reg->pid_cold, -error, dt);
            pid_reset(&reg->pid_hot);
            reg->output_hot = 0;
            reg->output_cold = (int)output;
        }

        set_pwm_duty_percent(HEATING_PWM_CHANNEL, reg->output_hot);
        set_pwm_duty_percent(COOLING_PWM_CHANNEL, reg->output_cold);

        ESP_LOGI(TAG, "T=%.2fC setpoint=%.2fC heat=%d%% cool=%d%%",
                 avg_temp, reg->setpoint, reg->output_hot, reg->output_cold);

        vTaskDelay(pdMS_TO_TICKS(REGULATOR_INTERVAL_MS));
    }
}

// ---------------------------------------------------------------------------
// Open-loop step-response tuning
// ---------------------------------------------------------------------------

/**
 * @brief Apply a fixed duty step to one PWM channel and log the resulting
 * temperature curve until it settles, hits the safety limit, or times out.
 *
 * Every line is logged as "TUNE,<label>,<elapsed_s>,<temperature>" so you
 * can grep the serial log and paste it straight into a spreadsheet/plot to
 * read off K, L and T (see the comment block in regulator.h).
 */
static void run_step_test(const char *label, ledc_channel_t channel, int duty_percent, ntc_readout_t *ntc)
{
    double baseline_temp = 0.0;
    get_average_temperature(ntc, &baseline_temp);
    ESP_LOGI(TAG, "TUNE,%s,BASELINE,%.3f", label, baseline_temp);
    ESP_LOGI(TAG, "TUNE,%s,STEP_DUTY,%d", label, duty_percent);

    TickType_t start_tick = xTaskGetTickCount();
    TickType_t last_check_tick = start_tick;
    double last_check_temp = baseline_temp;

    set_pwm_duty_percent(channel, duty_percent);

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(TUNE_LOG_INTERVAL_MS));

        double temp;
        if (!get_average_temperature(ntc, &temp))
        {
            continue;
        }

        double elapsed_s = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS / 1000.0;
        ESP_LOGI(TAG, "TUNE,%s,%.1f,%.3f", label, elapsed_s, temp);

        // Never let a tuning run cook (or freeze) the plant.
        if (temp >= REGULATOR_MAX_TEMPERATURE)
        {
            ESP_LOGW(TAG, "TUNE,%s,ABORT_MAX_TEMP,%.3f", label, temp);
            break;
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - last_check_tick) * portTICK_PERIOD_MS >= TUNE_SETTLE_WINDOW_MS)
        {
            if (fabs(temp - last_check_temp) < TUNE_SETTLE_EPSILON_C)
            {
                ESP_LOGI(TAG, "TUNE,%s,SETTLED,%.3f", label, temp);
                break;
            }
            last_check_tick = now;
            last_check_temp = temp;
        }

        if ((now - start_tick) * portTICK_PERIOD_MS >= TUNE_TIMEOUT_MS)
        {
            ESP_LOGW(TAG, "TUNE,%s,TIMEOUT,%.3f", label, temp);
            break;
        }
    }

    set_pwm_duty_percent(channel, 0);
}

void regulator_pid_tune_task(void *args)
{
    regulator_args_t *reg_args = (regulator_args_t *)args;
    ntc_readout_t *ntc = &reg_args->regulator->ntc_readout;

    // Make sure both outputs start off, and give the NTC buffer a moment to
    // fill so the first baseline reading is meaningful.
    set_pwm_duty_percent(HEATING_PWM_CHANNEL, 0);
    set_pwm_duty_percent(COOLING_PWM_CHANNEL, 0);
    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGI(TAG, "TUNE,START,HEATER");
    run_step_test("HEATER", HEATING_PWM_CHANNEL, TUNE_HEATER_DUTY, ntc);

    ESP_LOGI(TAG, "TUNE,COOLDOWN,%d", TUNE_COOLDOWN_MS);
    vTaskDelay(pdMS_TO_TICKS(TUNE_COOLDOWN_MS));

    ESP_LOGI(TAG, "TUNE,START,COOLER");
    run_step_test("COOLER", COOLING_PWM_CHANNEL, TUNE_COOLER_DUTY, ntc);

    ESP_LOGI(TAG, "TUNE,DONE - read K/L/T off the TUNE,HEATER,... and TUNE,COOLER,... lines above, "
                  "fill in PROCESS_GAIN_*/DEAD_TIME_*/TIME_CONST_* in regulator.h, then swap this task "
                  "back out for regulator_task in main.c");

    vTaskDelete(NULL);
}