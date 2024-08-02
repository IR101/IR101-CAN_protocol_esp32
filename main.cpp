#include "mcp2515.hpp"

#include <cstring>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include "esp_intr_alloc.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define GPIO_INPUT_IO_0    static_cast<gpio_num_t>(5)
#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t gpio_evt_queue = nullptr;
static bool flag = false;

static void gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    flag = true;
}

extern "C" void app_main()
{
    // Configure GPIO
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << GPIO_INPUT_IO_0);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE, // Disable pull-down mode
    gpio_config(&io_conf);

    // Create a queue to handle GPIO events
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    // Install GPIO ISR service
    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);

    // Hook ISR handler for specific GPIO pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

    // Configure SPI bus
    spi_bus_config_t buscfg = {};
    buscfg.miso_io_num = 39;
    buscfg.mosi_io_num = 38;
    buscfg.sclk_io_num = 40;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 1 * 1000 * 1000; // 1 MHz
    devcfg.mode = 0;
    devcfg.spics_io_num = 36;
    devcfg.queue_size = 1;

    // Initialize the SPI bus
    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE("MCP2515", "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }

    // Add SPI device to bus
    spi_device_handle_t handle;
    ret = spi_bus_add_device(SPI3_HOST, &devcfg, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE("MCP2515", "Failed to add SPI device: %s", esp_err_to_name(ret));
        return;
    }

    // Initialize the MCP2515
    MCP2515 mcp2515(&handle);

    // Reset the MCP2515
    if (mcp2515.reset() != MCP2515::ERROR_OK) {
        ESP_LOGE("MCP2515", "MCP2515 reset failed");
        return;
    }

    // Set bitrate
    if (mcp2515.setBitrate(CAN_SPEED::CAN_1000KBPS, CAN_CLOCK::MCP_8MHZ) != MCP2515::ERROR_OK) {
        ESP_LOGE("MCP2515", "MCP2515 set bitrate failed");
        return;
    }

    // Set normal mode
    if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
        ESP_LOGE("MCP2515", "MCP2515 set normal mode failed");
        return;
    }
    ESP_LOGI("MCP2515", "MCP2515 initialized successfully");

    can_frame send_frame;
    send_frame.can_id = 0x002;
    send_frame.can_dlc = 4;
    send_frame.data[0] = 5;
    send_frame.data[1] = 6;
    send_frame.data[2] = 7;
    send_frame.data[3] = 8;

    can_frame receive_frame;

    while (true) {
        // Check if a transmit buffer is available
        if (mcp2515.sendMessage(&send_frame) == MCP2515::ERROR_OK) {
            ESP_LOGI("MCP2515", "Message sent successfully");
        } else {
            ESP_LOGE("MCP2515", "Error sending message");
        }

        ESP_LOGI("MCP2515", "MCP2515 status: %d", mcp2515.getStatus());

        if (flag) {
            flag = false; // Reset flag
            if (mcp2515.readMessage(&receive_frame) == MCP2515::ERROR_OK) {
                ESP_LOGI("MCP2515", "Message received successfully");

                if (receive_frame.can_dlc <= CAN_MAX_DLEN) {
                    ESP_LOGI("MCP2515", "Received data:");
                    for (int i = 0; i < receive_frame.can_dlc; i++) {
                        ESP_LOGI("MCP2515", "%d ", receive_frame.data[i]);
                    }
                } else {
                    ESP_LOGW("MCP2515", "Received invalid DLC: %d", receive_frame.can_dlc);
                }
            } else {
                ESP_LOGE("MCP2515", "Error receiving message");
            }

            ESP_LOGI("MCP2515", "MCP2515 status: %d", mcp2515.getStatus());
        }
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}
