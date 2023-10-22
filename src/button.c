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
  uint8_t history;
  int64_t next_long_time;
} debounce_t;

int pin_count = -1;
debounce_t * debounce;
QueueHandle_t queue;


static void send_event(QueueHandle_t queue, gpio_num_t pin, button_event_type_t ev) {
    button_event_t event = {
        .pin = pin,
        .event = ev,
    };
    xQueueSend(queue, &event, portMAX_DELAY);
}

static void button_task(void *pvParameter)
{
    debounce_t *d_begin = debounce;
    debounce_t *d_end = d_begin + pin_count;

    for (;;) {
        for (debounce_t *d=d_begin; d<d_end; d++) {
            uint8_t pressed = d->history & 0b10000000;
            d->history = (d->history << 1) | (d->inverted ^ gpio_get_level(d->pin)) | pressed;

            if (d->history == 0b10000000) {
              // up
              d->history = 0b00000000;
              ESP_LOGI(TAG, "%d UP", d->pin);
              d->next_long_time = INT64_MAX;
              send_event(queue, d->pin, BUTTON_UP);
            } else if (d->history == 0b01111111) {
              // down
              d->history = 0b11111111;
              ESP_LOGI(TAG, "%d DOWN", d->pin);
              d->next_long_time = esp_timer_get_time() + CONFIG_ESP32_BUTTON_LONG_PRESS_DURATION_MS;
              send_event(queue, d->pin, BUTTON_DOWN);
            } else if (esp_timer_get_time() >= d->next_long_time) {
              //repeat
              ESP_LOGI(TAG, "%d LONG", d->pin);
              d->next_long_time = d->next_long_time + CONFIG_ESP32_BUTTON_LONG_PRESS_REPEAT_MS;
              send_event(queue, d->pin, BUTTON_HELD);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

button_config_t button_init(uint64_t pin_select) {
    return pulled_button_init(pin_select, GPIO_FLOATING);
}


button_config_t pulled_button_init(uint64_t pin_select, gpio_pull_mode_t pull_mode)
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
