#include "driver/pcnt.h"

#define PULSE_PIN_CLK 18
#define PULSE_PIN_DT  19

void setup() {
  Serial.begin(115200);
  delay(1000);

  gpio_set_pull_mode(GPIO_NUM_18, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(GPIO_NUM_19, GPIO_PULLUP_ONLY);


  pcnt_config_t pcnt_config1 = {};
  pcnt_config1.pulse_gpio_num = PULSE_PIN_CLK;
  pcnt_config1.ctrl_gpio_num  = PULSE_PIN_DT;
  pcnt_config1.lctrl_mode     = PCNT_MODE_KEEP;
  pcnt_config1.hctrl_mode     = PCNT_MODE_REVERSE;
  pcnt_config1.pos_mode       = PCNT_COUNT_INC;
  pcnt_config1.neg_mode       = PCNT_COUNT_DEC;
  pcnt_config1.counter_h_lim  = 32767;
  pcnt_config1.counter_l_lim  = -32768;
  pcnt_config1.unit           = PCNT_UNIT_0;
  pcnt_config1.channel        = PCNT_CHANNEL_0;

  pcnt_config_t pcnt_config2 = {};
  pcnt_config2.pulse_gpio_num = PULSE_PIN_DT;
  pcnt_config2.ctrl_gpio_num  = PULSE_PIN_CLK;
  pcnt_config2.lctrl_mode     = PCNT_MODE_REVERSE;
  pcnt_config2.hctrl_mode     = PCNT_MODE_KEEP;
  pcnt_config2.pos_mode       = PCNT_COUNT_INC;
  pcnt_config2.neg_mode       = PCNT_COUNT_DEC;
  pcnt_config2.counter_h_lim  = 32767;
  pcnt_config2.counter_l_lim  = -32768;
  pcnt_config2.unit           = PCNT_UNIT_0;
  pcnt_config2.channel        = PCNT_CHANNEL_1;

  pcnt_unit_config(&pcnt_config1);
  pcnt_unit_config(&pcnt_config2);

  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);
}

void loop() {
  int16_t count = 0;
  pcnt_get_counter_value(PCNT_UNIT_0, &count);
  Serial.printf("%d\n", count);
  delay(10);
}
