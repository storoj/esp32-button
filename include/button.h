#ifndef ESP32_BUTTON_H
#define ESP32_BUTTON_H

#include <stdint.h>

#include "freertos/portmacro.h"
#include "hal/gpio_types.h"

#ifndef CONFIG_ESP32_BUTTON_LONG_PRESS_DURATION_MS
#define CONFIG_ESP32_BUTTON_LONG_PRESS_DURATION_MS (2000)
#endif

#ifndef CONFIG_ESP32_BUTTON_LONG_PRESS_REPEAT_MS
#define CONFIG_ESP32_BUTTON_LONG_PRESS_REPEAT_MS (50)
#endif

#ifndef CONFIG_ESP32_BUTTON_QUEUE_SIZE
#define CONFIG_ESP32_BUTTON_QUEUE_SIZE (4)
#endif

#ifndef CONFIG_ESP32_BUTTON_TASK_STACK_SIZE
#define CONFIG_ESP32_BUTTON_TASK_STACK_SIZE 3072
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BUTTON_DOWN,
    BUTTON_UP,
    BUTTON_HELD,
} button_event_type_t;

typedef struct {
    gpio_num_t pin;
    button_event_type_t event;
} button_event_t;

struct button;
typedef struct button *button_t;

button_t button_init(uint64_t pin_select);
button_t pulled_button_init(uint64_t pin_select, gpio_pull_mode_t pull_mode);
BaseType_t button_poll(button_t btn, button_event_t *e, TickType_t ticksToWait);
void button_free(button_t btn);

#ifdef __cplusplus
}
#endif

#endif
