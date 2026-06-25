#include "button.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#define TAG            "BUTTON"
#define DEBOUNCE_MS    20       // ignore bounces shorter than this

// Button pin → command mapping
static const struct {
    gpio_num_t pin;
    uint8_t    cmd;
} BUTTONS[] = {
    { GPIO_NUM_2, CMD_BTN_UP   },
    { GPIO_NUM_3, CMD_BTN_DOWN },
    { GPIO_NUM_5, CMD_BTN_EXIT },
};
#define NUM_BUTTONS (sizeof(BUTTONS) / sizeof(BUTTONS[0]))

// ISR pushes raw GPIO number here
static QueueHandle_t s_isr_queue = NULL;

// Debounced commands go here — read by button_wait()
static QueueHandle_t s_cmd_queue = NULL;

// ----------------------------------------------------------
// ISR — runs in interrupt context, must be minimal
// ----------------------------------------------------------
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    gpio_num_t pin = (gpio_num_t)(uint32_t)arg;
    xQueueSendFromISR(s_isr_queue, &pin, NULL);
}

// ----------------------------------------------------------
// Debounce task — reads raw ISR events, filters bounces
// ----------------------------------------------------------
static void button_task(void *arg)
{
    gpio_num_t pin;

    while (1) {
        // Wait for an ISR event
        if (xQueueReceive(s_isr_queue, &pin, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // Wait for bounce to settle
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));

        // Only register if pin is still LOW (still pressed)
        if (gpio_get_level(pin) != 0) {
            continue;   // was a bounce — ignore
        }

        // Map pin → command
        for (int i = 0; i < NUM_BUTTONS; i++) {
            if (BUTTONS[i].pin == pin) {
                ESP_LOGI(TAG, "Button on GPIO%d → 0x%02X", pin, BUTTONS[i].cmd);
                xQueueSend(s_cmd_queue, &BUTTONS[i].cmd, 0);
                break;
            }
        }

        // Wait for release before accepting next press
        while (gpio_get_level(pin) == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// ----------------------------------------------------------
// Public API
// ----------------------------------------------------------
esp_err_t button_init(void)
{
    s_isr_queue = xQueueCreate(10, sizeof(gpio_num_t));
    s_cmd_queue = xQueueCreate(10, sizeof(uint8_t));

    if (!s_isr_queue || !s_cmd_queue) return ESP_ERR_NO_MEM;

    // Install shared GPIO ISR service
    gpio_install_isr_service(0);

    for (int i = 0; i < NUM_BUTTONS; i++) {
        gpio_config_t cfg = {
            .pin_bit_mask = (1ULL << BUTTONS[i].pin),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,    // active LOW buttons
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_NEGEDGE,     // falling edge = press
        };
        ESP_ERROR_CHECK(gpio_config(&cfg));
        ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTONS[i].pin,
                                             gpio_isr_handler,
                                             (void *)(uint32_t)BUTTONS[i].pin));
    }

    xTaskCreate(button_task, "buttons", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "Buttons ready (GPIO2=UP  GPIO3=DOWN  GPIO5=EXIT)");
    return ESP_OK;
}

bool button_wait(uint8_t *cmd, TickType_t timeout)
{
    return xQueueReceive(s_cmd_queue, cmd, timeout) == pdTRUE;
}