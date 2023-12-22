#include "servo.h"

#define SERVO_PWM_MODULE TCC1
#define SERVO_PWM_CHANNEL 0
/*#define SERVO_PWM_OUTPUT TCC_MATCH_CAPTURE_CHANNEL_0*/
#define SERVO_PWM_PIN PIN_PA10E_TCC1_WO0
#define SERVO_PWM_MUX MUX_PA10E_TCC1_WO0

struct tcc_module tcc_instance;
struct tcc_config config_tcc;

void configure_tcc_for_pwm(void) {
	tcc_get_config_defaults(&config_tcc, SERVO_PWM_MODULE);

	config_tcc.counter.clock_source = GCLK_GENERATOR_0;
	config_tcc.counter.period = 1000000;
	config_tcc.counter.clock_prescaler = TCC_CTRLA_PRESCALER_DIV16;
	config_tcc.compare.match[SERVO_PWM_CHANNEL] = 20000;
	config_tcc.compare.wave_polarity[SERVO_PWM_CHANNEL] = 0;
	config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;
	config_tcc.compare.wave_ramp = TCC_RAMP_RAMP1;

	tcc_init(&tcc_instance, SERVO_PWM_MODULE, &config_tcc);

	// Ensure that the synchronization is complete before enabling the TCC
	while (tcc_is_syncing(&tcc_instance)) {
	}

	tcc_enable(&tcc_instance);
}

void configure_pwm_pin(void) {
	struct system_pinmux_config pinmux_config;

	system_pinmux_get_config_defaults(&pinmux_config);

	pinmux_config.mux_position = SERVO_PWM_MUX;
	pinmux_config.direction = SYSTEM_PINMUX_PIN_DIR_OUTPUT;

	// Use the correct function to set the pin configuration
	system_pinmux_pin_set_config(SERVO_PWM_PIN, &pinmux_config);
}

// Moves the servo to a specified position in degrees
void servo_set(uint32_t degrees) {
	tcc_set_compare_value(&tcc_instance, SERVO_PWM_CHANNEL, degrees);
	// Ensure that the synchronization is complete before proceeding
	while (tcc_is_syncing(&tcc_instance)) {

	}
	
}

void configure_clocks(void) {
	struct system_gclk_gen_config gclk_config;

	system_gclk_gen_get_config_defaults(&gclk_config);

	gclk_config.source_clock = SYSTEM_CLOCK_SOURCE_DFLL;
	gclk_config.division_factor = 1;

	// Use the correct function to set the GCLK configuration
	system_gclk_gen_set_config(GCLK_GENERATOR_0, &gclk_config);
	system_gclk_gen_enable(GCLK_GENERATOR_0);
}


