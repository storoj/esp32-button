#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "button.h"

#define TAG "BUTTON"

typedef struct {
  gpio_num_t pin;
  bool inverted;
  uint16_t history;
  int64_t next_long_time;
} debounce_t;

int pin_count = -1;
debounce_t * debounce;
QueueHandle_t queue;

static void update_button(debounce_t *d) {
    d->history = (d->history << 1) | ((d->inverted ^ gpio_get_level(d->pin)) & 1);
}

#define MASK   0b1111000000111111
static bool button_down(debounce_t *d) {
    if ((d->history & MASK) == 0b0000000000111111) {
        d->history = 0xffff;
        return 1;
    }
    return 0;
}
static bool button_up(debounce_t *d) {
    if ((d->history & MASK) == 0b1111000000000000) {
        d->history = 0x0000;
        return 1;
    }
    return 0;
}

static void send_event(gpio_num_t pin, int ev) {
    button_event_t event = {
        .pin = pin,
        .event = ev,
    };
    xQueueSend(queue, &event, portMAX_DELAY);
}

static void button_task(void *pvParameter)
{
    for (;;) {
        for (int idx=0; idx<pin_count; idx++) {
            update_button(&debounce[idx]);
            if (button_up(&debounce[idx])) {
                debounce[idx].next_long_time = INT64_MAX;
                ESP_LOGI(TAG, "%d UP", debounce[idx].pin);
                send_event(debounce[idx].pin, BUTTON_UP);
            } else if (esp_timer_get_time() >= debounce[idx].next_long_time) {
                ESP_LOGI(TAG, "%d LONG", debounce[idx].pin);
                debounce[idx].next_long_time = debounce[idx].next_long_time + CONFIG_ESP32_BUTTON_LONG_PRESS_REPEAT_MS;
                send_event(debounce[idx].pin, BUTTON_HELD);
            } else if (button_down(&debounce[idx]) && debounce[idx].next_long_time == 0) {
                ESP_LOGI(TAG, "%d DOWN", debounce[idx].pin);
                debounce[idx].next_long_time = esp_timer_get_time() + CONFIG_ESP32_BUTTON_LONG_PRESS_DURATION_MS;
                send_event(debounce[idx].pin, BUTTON_DOWN);
            } 
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

QueueHandle_t button_init(uint64_t pin_select) {
    return pulled_button_init(pin_select, GPIO_FLOATING);
}


QueueHandle_t pulled_button_init(uint64_t pin_select, gpio_pull_mode_t pull_mode)
{
    if (pin_count != -1) {
        ESP_LOGI(TAG, "Already initialized");
        return NULL;
    }

    // Configure the pins
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = (pull_mode == GPIO_PULLUP_ONLY || pull_mode == GPIO_PULLUP_PULLDOWN);
    io_conf.pull_down_en = (pull_mode == GPIO_PULLDOWN_ONLY || pull_mode == GPIO_PULLUP_PULLDOWN);
    io_conf.pin_bit_mask = pin_select;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Scan the pin map to determine number of pins
    pin_count = 0;
    for (gpio_num_t pin=GPIO_NUM_0; pin<=GPIO_NUM_MAX; pin++) {
        if (BIT64(pin) & pin_select) {
            pin_count++;
        }
    }

    // Initialize global state and queue
    debounce = calloc(pin_count, sizeof(debounce_t));
    queue = xQueueCreate(CONFIG_ESP32_BUTTON_QUEUE_SIZE, sizeof(button_event_t));

    // Scan the pin map to determine each pin number, populate the state
    uint32_t idx = 0;
    for (gpio_num_t pin=GPIO_NUM_0; pin<=GPIO_NUM_MAX; pin++) {
        if (BIT64(pin) & pin_select) {
            ESP_LOGI(TAG, "Registering button input: %d", pin);
            debounce[idx].pin = pin;
            debounce[idx].next_long_time = INT64_MAX;
            debounce[idx].inverted = true;
            idx++;
        }
    }

    // Spawn a task to monitor the pins
    xTaskCreate(&button_task, "button_task", CONFIG_ESP32_BUTTON_TASK_STACK_SIZE, NULL, 10, NULL);

    return queue;
}
