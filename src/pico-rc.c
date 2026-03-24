// A bluetooth connected motor and LED driver
// Suitable for an RC car

#include "btclient.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

typedef enum {
    LIGHT_OFF,
    LIGHT_LOW,
    LIGHT_HIGH
} light_state_t;

int main()
{
    if (btclient_setup()) {
        return -1;
    }

    const uint SERVO_1PIN_ENABLE = 1;
    const uint SERVO_1PIN_SCALE = 8;
    const uint SERVO_1PIN_MIN_PULSE_US = 500;
    const uint SERVO_1PIN_MAX_PULSE_US = 2500;
    const uint SERVO_1PIN_FREQ_HZ = (SERVO_1PIN_SCALE * 256 * 1000000) / (SERVO_1PIN_MAX_PULSE_US - SERVO_1PIN_MIN_PULSE_US);
    const uint SERVO_1PIN_MIN_PULSE_CLKS = SERVO_1PIN_MIN_PULSE_US * SERVO_1PIN_FREQ_HZ / 1000000;
    const uint SERVO_1PIN_MAX_PULSE_CLKS = SERVO_1PIN_MAX_PULSE_US * SERVO_1PIN_FREQ_HZ / 1000000;
    const uint SERVO_1PIN_PERIOD_US = 20000;
    const uint SERVO_1PIN_PERIOD_CLKS = SERVO_1PIN_PERIOD_US * SERVO_1PIN_FREQ_HZ / 1000000;

    const int GEAR_MAX_FORWARD = 4;
    const int GEAR_MAX_REVERSE = -1;

    const uint DRIVE_MOTOR_PIN_BASE = 12;
    const uint DRIVE_SERVO_PIN_BASE = 18;
    const uint DRIVE_LIGHT_PIN_BASE = 14;
    const uint DRIVE_AUX_LIGHT_PIN_BASE = 16;
    const uint DRIVE_SERVO_POWER_PIN_BASE = 20;

    const uint PWM_FREQ = 320000;
    const uint SERVO_PWM_FREQ = (SERVO_1PIN_ENABLE) ? SERVO_1PIN_FREQ_HZ : PWM_FREQ;
    const uint SERVO_PWM_WRAP = (SERVO_1PIN_ENABLE) ? (SERVO_1PIN_PERIOD_CLKS - 1) : 255;

    uint DRIVE_MOTOR_PWM_SLICE = pwm_gpio_to_slice_num(DRIVE_MOTOR_PIN_BASE);
    uint DRIVE_SERVO_PWM_SLICE = pwm_gpio_to_slice_num(DRIVE_SERVO_PIN_BASE);
    uint DRIVE_LIGHT_PWM_SLICE = pwm_gpio_to_slice_num(DRIVE_LIGHT_PIN_BASE);
    uint DRIVE_AUX_LIGHT_PWM_SLICE = pwm_gpio_to_slice_num(DRIVE_AUX_LIGHT_PIN_BASE);
    uint DRIVE_SERVO_POWER_PWM_SLICE = pwm_gpio_to_slice_num(DRIVE_SERVO_POWER_PIN_BASE);

    const uint LIGHT_INTENSITY_OFF = 0;
    const uint LIGHT_INTENSITY_LOW = 64;
    const uint LIGHT_INTENSITY_HIGH = 256;

    // set pins to PWM
    gpio_set_function(DRIVE_MOTOR_PIN_BASE, GPIO_FUNC_PWM);
    gpio_set_function(DRIVE_MOTOR_PIN_BASE + 1, GPIO_FUNC_PWM);
    gpio_set_function(DRIVE_SERVO_PIN_BASE, GPIO_FUNC_PWM);
    gpio_set_function(DRIVE_SERVO_PIN_BASE + 1, GPIO_FUNC_PWM);
    gpio_set_function(DRIVE_LIGHT_PIN_BASE, GPIO_FUNC_PWM);
    gpio_set_function(DRIVE_LIGHT_PIN_BASE + 1, GPIO_FUNC_PWM);
    gpio_set_function(DRIVE_AUX_LIGHT_PIN_BASE, GPIO_FUNC_PWM);
    gpio_set_function(DRIVE_SERVO_POWER_PIN_BASE, GPIO_FUNC_PWM);

    // set clocks for PWM
    float SYSTEM_CLK = SYS_CLK_HZ;
    float PWM_CLK_DIV = SYSTEM_CLK / PWM_FREQ;
    float SERVO_PWM_CLK_DIV = SYSTEM_CLK / SERVO_PWM_FREQ;
    pwm_set_clkdiv(DRIVE_MOTOR_PWM_SLICE, PWM_CLK_DIV);
    pwm_set_clkdiv(DRIVE_SERVO_PWM_SLICE, SERVO_PWM_CLK_DIV);
    pwm_set_clkdiv(DRIVE_LIGHT_PWM_SLICE, PWM_CLK_DIV);
    pwm_set_clkdiv(DRIVE_AUX_LIGHT_PWM_SLICE, PWM_CLK_DIV);
    pwm_set_clkdiv(DRIVE_SERVO_POWER_PWM_SLICE, PWM_CLK_DIV);

    // set PWM counter wrap
    pwm_set_wrap(DRIVE_MOTOR_PWM_SLICE, 255);
    pwm_set_wrap(DRIVE_SERVO_PWM_SLICE, SERVO_PWM_WRAP);
    pwm_set_wrap(DRIVE_LIGHT_PWM_SLICE, 255);
    pwm_set_wrap(DRIVE_AUX_LIGHT_PWM_SLICE, 255);
    pwm_set_wrap(DRIVE_SERVO_POWER_PWM_SLICE, 255);

    // set all PWM to off
    pwm_set_both_levels(DRIVE_MOTOR_PWM_SLICE, 0, 0);
    pwm_set_both_levels(DRIVE_SERVO_PWM_SLICE, 0, 0);
    pwm_set_both_levels(DRIVE_LIGHT_PWM_SLICE, 0, 0);
    pwm_set_chan_level(DRIVE_AUX_LIGHT_PWM_SLICE, PWM_CHAN_A, 0);
    pwm_set_chan_level(DRIVE_SERVO_POWER_PWM_SLICE, PWM_CHAN_A, 0);

    // enable pwm
    pwm_set_enabled(DRIVE_MOTOR_PWM_SLICE, true);
    pwm_set_enabled(DRIVE_SERVO_PWM_SLICE, true);
    pwm_set_enabled(DRIVE_LIGHT_PWM_SLICE, true);
    pwm_set_enabled(DRIVE_AUX_LIGHT_PWM_SLICE, true);
    pwm_set_enabled(DRIVE_SERVO_POWER_PWM_SLICE, true);

    // initialize analog to digital converter
    const uint ADC_INPUT = 0;
    const uint ADC_PIN = ADC_INPUT + 26;

    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(ADC_INPUT);

    bool use_servo_feedback = (SERVO_1PIN_ENABLE == 0);

    // move the servo center, left and right, and check the servo feedback adc line
    // if thte line doesn't change (much), then assume no feedback is available, or not reliable

    const uint SERVO_CHECK_DELAY_MS = 600;
    uint servo_center_pos, servo_left_pos, servo_right_pos;
    if (use_servo_feedback) {
        // power up servo, and should already be at center
        // wait a small amount of time, and read the feedback
        pwm_set_chan_level(DRIVE_SERVO_POWER_PWM_SLICE, PWM_CHAN_A, 256);
        sleep_ms(SERVO_CHECK_DELAY_MS);
        servo_center_pos = adc_read();

        // move to left, wait, and read position
        pwm_set_both_levels(DRIVE_SERVO_PWM_SLICE, 0, 256);
        sleep_ms(SERVO_CHECK_DELAY_MS);
        servo_left_pos = adc_read();

        // Add a check to see if the input pin is disconnected
        // enable the pull up resistor, and read the value, and store it
        gpio_pull_up(ADC_PIN);
        // wait to make sure pull up resistor has been enabled
        sleep_ms(10);
        // ADC value may have changed due to enabling the pull up resistor
        uint pull_up_adc1 = adc_read();

        // move to right, wait and read position
        pwm_set_both_levels(DRIVE_SERVO_PWM_SLICE, 256, 0);
        sleep_ms(SERVO_CHECK_DELAY_MS);

        // Check if the value changed by a significant margin
        uint pull_up_adc2 = adc_read();
        uint pull_up_delta = (pull_up_adc1 > pull_up_adc2) ? pull_up_adc1 - pull_up_adc2 : pull_up_adc2 - pull_up_adc1;
        const uint SERVO_FEEDBACK_EXISTS_TOLERANCE = 500;
        use_servo_feedback = (pull_up_delta >= SERVO_FEEDBACK_EXISTS_TOLERANCE);
        // disable pull up resistor
        gpio_disable_pulls(ADC_PIN);
        // wait to make sure resistor has been disabled
        sleep_ms(10);

        servo_right_pos = adc_read();

        // move back to center, wait and power down
        pwm_set_both_levels(DRIVE_SERVO_PWM_SLICE, 0, 0);
        sleep_ms(SERVO_CHECK_DELAY_MS);

        if ((servo_center_pos == servo_left_pos) || (servo_center_pos == servo_right_pos) || (servo_left_pos == servo_right_pos)) {
            use_servo_feedback = false;
        }

        if (use_servo_feedback) {
            // if we're using feedback, power down the servo
            pwm_set_chan_level(DRIVE_SERVO_POWER_PWM_SLICE, PWM_CHAN_A, 0);
        }
    }

    const uint sleep_delay = 5;
    const uint blink_slow_delay = 500;
    const uint blink_med_delay = 250;
    const uint blink_fast_delay = 50;

#ifdef MOTOR_RAMP
    const uint motor_ramp_ms = 2000;  // Time in ms to ramp the motor to full throttle
    const uint motor_full_throttle = 256;
    const uint motor_ramp_iterations = (motor_ramp_ms + sleep_delay - 1) / sleep_delay;
    const uint motor_inc_per_iter = motor_full_throttle / motor_ramp_iterations;
    const uint motor_inc_remainder = motor_full_throttle - (motor_inc_per_iter * motor_ramp_iterations);
    const uint motor_remainder_inc1 = (motor_inc_remainder == 0) ? 0 : (motor_ramp_iterations / motor_inc_remainder);
    const uint motor_inc_remainder2 = (motor_remainder_inc1 == 0) ? 0 : (motor_inc_remainder - motor_ramp_iterations / motor_remainder_inc1);
    const uint motor_remainder_inc2 = (motor_inc_remainder2 == 0) ? 0 : (motor_ramp_iterations / motor_inc_remainder2);
#endif

    uint cur_blink_state = 0;
    uint cur_blink_iteration = 0;
    uint blink_iterations = blink_slow_delay / sleep_delay;
    light_state_t front_light_state = LIGHT_OFF;
    light_state_t aux_light_state = LIGHT_OFF;
    int gear_state = 1;   // Start in 1st gear

    hid_state_t gpad_state;
    uint center_count = 0;
#ifdef MOTOR_RAMP
    uint motor_count1 = 0;
    uint motor_count2 = 0;
    uint cur_motor_speed = 0;
    uint motor_inc;
#endif

    while (true) {
        // Read and process user input
        btclient_get_hid_state(&gpad_state);
        if (gpad_state.buttons & gpad_state.buttons_toggled & 0x1) {
            switch (front_light_state) {
            case LIGHT_OFF: front_light_state = LIGHT_LOW; break;
            case LIGHT_LOW: front_light_state = LIGHT_HIGH; break;
            default: front_light_state = LIGHT_OFF; break;
            }
        }
        if (gpad_state.buttons & gpad_state.buttons_toggled & 0x8) {
            switch (aux_light_state) {
            case LIGHT_OFF: aux_light_state = LIGHT_LOW; break;
            case LIGHT_LOW: aux_light_state = LIGHT_HIGH; break;
            default: aux_light_state = LIGHT_OFF; break;
            }
        }
        if (gpad_state.buttons & gpad_state.buttons_toggled & 0x10) {
            if (gear_state > 0) {
                gear_state--;
            } else if (gear_state < -1) {
                gear_state++;
            }
        }
        if (gpad_state.buttons & gpad_state.buttons_toggled & 0x20) {
            if (gear_state >= 0 && gear_state < GEAR_MAX_FORWARD) {
                gear_state++;
            } else if (gear_state < 0 && gear_state > GEAR_MAX_REVERSE) {
                gear_state--;
            }
        }
        switch (gpad_state.hat) {
        case 0:  // Up
            gear_state = 1;
            break;
        case 4:  // Down
            gear_state = -1;
            break;
        default:
            break;
        }

        // Drive motors and leds
        if (SERVO_1PIN_ENABLE) {
            uint32_t steer_amount = SERVO_1PIN_MIN_PULSE_CLKS + SERVO_1PIN_SCALE * gpad_state.lx;
            uint32_t gear_amount = (gear_state < 0) ? -gear_state : gear_state;
            gear_amount = SERVO_1PIN_MIN_PULSE_CLKS + SERVO_1PIN_SCALE * 64 * gear_amount;
            pwm_set_both_levels(DRIVE_SERVO_PWM_SLICE, steer_amount, gear_amount);
        } else if (use_servo_feedback) {
            const uint STEER_DEFAULT_TOLERANCE = 160;
            const uint STEER_CENTER_TOLERANCE = 192;
            const uint STEER_SIDE_TOLERANCE = 96;
            uint steer_amount = (gpad_state.lx < 128) ?
                ((servo_center_pos * gpad_state.lx) + (servo_left_pos * (128 - gpad_state.lx))) :
                ((servo_center_pos * (255 - gpad_state.lx)) + (servo_right_pos * (gpad_state.lx - 127)));
            steer_amount >>= 7;  // 0-4k range
            uint cur_steer = adc_read();
            uint steer_amount_from_center = (steer_amount > servo_center_pos) ? (steer_amount - servo_center_pos) : (servo_center_pos - steer_amount);
            uint steer_amount_from_left = (steer_amount > servo_left_pos) ? (steer_amount - servo_left_pos) : (servo_left_pos - steer_amount);
            uint steer_amount_from_right = (steer_amount > servo_right_pos) ? (steer_amount - servo_right_pos) : (servo_right_pos - steer_amount);
            uint steer_tolerance = STEER_DEFAULT_TOLERANCE;
            if (steer_amount_from_center < STEER_CENTER_TOLERANCE) {
                steer_amount = servo_center_pos;
                center_count++;
                //steer_tolerance = 0;
            } else {
                center_count = 0;
                if (steer_amount_from_left < STEER_SIDE_TOLERANCE) {
                    steer_amount = servo_left_pos;
                    //steer_tolerance = 0;
                } else if (steer_amount_from_right < STEER_SIDE_TOLERANCE) {
                    steer_amount = servo_right_pos;
                    //steer_tolerance = 0;
                } else if (steer_amount_from_left < steer_amount_from_right) {
                    int steer_left_range = (servo_center_pos - STEER_CENTER_TOLERANCE) - (servo_left_pos - STEER_SIDE_TOLERANCE);
                    int steer_weight = (steer_amount - (servo_left_pos - STEER_SIDE_TOLERANCE));
                    steer_weight = (steer_weight * 4096) / steer_left_range;
                    steer_amount = (servo_center_pos * steer_weight) + (servo_left_pos * (4096 - steer_weight));
                    steer_amount >>= 12;
                } else {
                    int steer_right_range = (servo_right_pos - STEER_SIDE_TOLERANCE) - (servo_center_pos + STEER_CENTER_TOLERANCE);
                    int steer_weight = (steer_amount - (servo_center_pos + STEER_CENTER_TOLERANCE));
                    steer_weight = (steer_weight * 4096) / steer_right_range;
                    steer_amount = (servo_right_pos * steer_weight) + (servo_center_pos * (4096 - steer_weight));
                    steer_amount >>= 12;
                }
            }
            uint steer_diff = (steer_amount < cur_steer) ? cur_steer - steer_amount : steer_amount - cur_steer;
            const uint SERVO_MIN_POWER = 192;
            const uint SERVO_MAX_POWER = 256;
            uint servo_power = (((SERVO_MAX_POWER - SERVO_MIN_POWER) * steer_diff) >> 12) + SERVO_MIN_POWER;
            if (steer_amount_from_left < STEER_SIDE_TOLERANCE ||
                steer_amount_from_right < STEER_SIDE_TOLERANCE) {
                servo_power = SERVO_MAX_POWER;
            } else if (steer_amount_from_center < STEER_CENTER_TOLERANCE) {
                servo_power = SERVO_MIN_POWER;
            }
            if (steer_diff < steer_tolerance) {
                // if we're close enough to the correct steer postion, turn off the servo
                servo_power = 0;
            }
            // set the direction correctly, and power on servo
            if (steer_amount_from_center < STEER_CENTER_TOLERANCE && center_count >= 10) {
                // center steering
                pwm_set_both_levels(DRIVE_SERVO_PWM_SLICE, 0, 0);
            } else if (cur_steer < steer_amount || steer_amount_from_right < STEER_SIDE_TOLERANCE) {
                pwm_set_both_levels(DRIVE_SERVO_PWM_SLICE, 256, 0);
            } else {
                pwm_set_both_levels(DRIVE_SERVO_PWM_SLICE, 0, 256);
            }
            pwm_set_chan_level(DRIVE_SERVO_POWER_PWM_SLICE, PWM_CHAN_A, servo_power);
        } else {
            uint32_t steer_amount = ((gpad_state.lx <= 128) ? (128 - gpad_state.lx) : (gpad_state.lx - 127));
            steer_amount <<= 1; // multiply by 2

            // 5% dead zone
            steer_amount = (steer_amount < 12) ? 0 : steer_amount;

            if (gpad_state.lx <= 128) {
                pwm_set_both_levels(DRIVE_SERVO_PWM_SLICE, 0, steer_amount);
            } else {
                pwm_set_both_levels(DRIVE_SERVO_PWM_SLICE, steer_amount, 0);
            }
        }

        // 5% dead zone
        uint DEADZONE = 12;
        bool brake = (gpad_state.l2 >= DEADZONE);
        uint32_t motor_speed = ((brake) ? gpad_state.l2 : gpad_state.r2) + 1;
        motor_speed = (motor_speed < DEADZONE) ? 0 : motor_speed;
#ifdef MOTOR_RAMP
        motor_inc = motor_inc_per_iter;
        if (motor_remainder_inc1) {
            if (motor_count1 == 0) motor_inc++;
            motor_count1++;
            if (motor_count1 >= motor_remainder_inc1) motor_count1 = 0;
        }
        if (motor_remainder_inc2) {
            if (motor_count2 == 0) motor_inc++;
            motor_count2++;
            if (motor_count2 >= motor_inc_remainder2) motor_count2 = 0;
        }
        if (motor_speed > cur_motor_speed) {
            if (cur_motor_speed + motor_inc > motor_speed) {
                cur_motor_speed = motor_speed;
            } else {
                cur_motor_speed += motor_inc;
            }
        } else {
            if (cur_motor_speed < motor_speed + motor_inc) {
                cur_motor_speed = motor_speed;
            } else {
                cur_motor_speed -= motor_inc;
            }
        }
#else
        uint32_t cur_motor_speed = motor_speed;
#endif
        if (brake) {
            cur_motor_speed = 0;
            pwm_set_both_levels(DRIVE_MOTOR_PWM_SLICE, motor_speed, motor_speed);
        } else if (gear_state < 0) {
            pwm_set_both_levels(DRIVE_MOTOR_PWM_SLICE, 0, cur_motor_speed);
        } else {
            pwm_set_both_levels(DRIVE_MOTOR_PWM_SLICE, cur_motor_speed, 0);
        }

        uint front_light_intensity = (front_light_state != LIGHT_OFF) ?
            ((front_light_state == LIGHT_HIGH) ? LIGHT_INTENSITY_HIGH : LIGHT_INTENSITY_LOW) : LIGHT_INTENSITY_OFF;
        uint tail_light_intensity = (brake) ? LIGHT_INTENSITY_HIGH :
            ((front_light_state != LIGHT_OFF) ? LIGHT_INTENSITY_LOW : LIGHT_INTENSITY_OFF);
        uint aux_light_intensity = (aux_light_state != LIGHT_OFF) ?
            ((aux_light_state == LIGHT_HIGH) ? LIGHT_INTENSITY_HIGH : LIGHT_INTENSITY_LOW) : LIGHT_INTENSITY_OFF;

        pwm_set_both_levels(DRIVE_LIGHT_PWM_SLICE, front_light_intensity, tail_light_intensity);
        pwm_set_chan_level(DRIVE_AUX_LIGHT_PWM_SLICE, PWM_CHAN_A, aux_light_intensity);

        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, cur_blink_state);
        cur_blink_iteration++;
        if (cur_blink_iteration >= blink_iterations) {
            cur_blink_state = !cur_blink_state;
            cur_blink_iteration = 0;
            switch (blink_state) {
            case BLINK_SLOW: blink_iterations = blink_slow_delay; break;
            case BLINK_MED:  blink_iterations = blink_med_delay; break;
            case BLINK_FAST: blink_iterations = blink_fast_delay; break;
            }
            blink_iterations /= sleep_delay;
        }
        sleep_ms(sleep_delay);
    }
}