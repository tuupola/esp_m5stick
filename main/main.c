/*

SPDX-License-Identifier: MIT-0

MIT No Attribution

Copyright (c) 2020 Mika Tuupola

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include "sdkconfig.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_sntp.h>
#include <esp_log.h>
#include <mipi_dcs.h>
#include <mipi_display.h>
#include <hagl.h>
#include <hagl_hal.h>
#include <fps.h>
#include <font6x9.h>
#include <rgb565.h>
#include <i2c_helper.h>
#include <axp192.h>
#include <bm8563.h>

#include "helpers/wifi.h"
#include "helpers/nvs.h"

static const char *TAG = "main";
static float fb_fps;

bm8563_t bm;
axp192_t axp;
struct tm rtc = {0};
struct tm rtc_alarm = {0};
spi_device_handle_t spi;

/*

Cap refresh rate to 45fps.
T = 1000 / 45 / (1000 / CONFIG_FREERTOS_HZ)

*/
void backbuffer_task(void *params)
{
    TickType_t last;
    const TickType_t period = 1000 / 45 / portTICK_RATE_MS;

    last = xTaskGetTickCount();

    while (1) {
        hagl_flush();
        fb_fps = fps();
        vTaskDelayUntil(&last, period);
    }

    vTaskDelete(NULL);
}

void rtc_task(void *params)
{
    uint16_t color = rgb565(0, 255, 0);
    wchar_t message[128];

    /* Calculate tm_yday for the first run. */
    mktime(&rtc);

    while (1) {
        bm8563_read(&bm, &rtc);
        swprintf(
            message,
            sizeof(message),
            L"%04d-%02d-%02d",
            rtc.tm_year + 1900, rtc.tm_mon + 1, rtc.tm_mday
        );
        hagl_put_text(message, 40, 28, color, font6x9);

        swprintf(
            message,
            sizeof(message),
            L"%02d:%02d:%02d",
            rtc.tm_hour, rtc.tm_min, rtc.tm_sec
        );
        hagl_put_text(message, 48, 38, color, font6x9);

        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

void alarm_timer_task(void *params)
{
    uint8_t tmp;

    while (1) {
        bm8563_ioctl(&bm, BM8563_CONTROL_STATUS2_READ, &tmp);
        if (tmp & BM8563_AF) {
            ESP_LOGI(TAG, "Got alarm flag. %d", tmp);
            tmp &= ~BM8563_AF;
            bm8563_ioctl(&bm, BM8563_CONTROL_STATUS2_WRITE, &tmp);
        }

        if (tmp & BM8563_TF) {
            ESP_LOGI(TAG, "Got timer flag. %d", tmp);
            tmp &= ~BM8563_TF;
            bm8563_ioctl(&bm, BM8563_CONTROL_STATUS2_WRITE, &tmp);
        }

        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

void log_task(void *params)
{
    float vacin, iacin, vvbus, ivbus, vts, temp, pbat, vbat, icharge, idischarge, vaps, cbat;
    uint8_t power, charge;
    char buffer[128];

    while (1) {
        axp192_read(&axp, AXP192_ACIN_VOLTAGE, &vacin);
        axp192_read(&axp, AXP192_ACIN_CURRENT, &iacin);
        axp192_read(&axp, AXP192_VBUS_VOLTAGE, &vvbus);
        axp192_read(&axp, AXP192_VBUS_CURRENT, &ivbus);
        axp192_read(&axp, AXP192_TEMP, &temp);
        axp192_read(&axp, AXP192_TS_INPUT, &vts);
        axp192_read(&axp, AXP192_BATTERY_POWER, &pbat);
        axp192_read(&axp, AXP192_BATTERY_VOLTAGE, &vbat);
        axp192_read(&axp, AXP192_CHARGE_CURRENT, &icharge);
        axp192_read(&axp, AXP192_DISCHARGE_CURRENT, &idischarge);
        axp192_read(&axp, AXP192_APS_VOLTAGE, &vaps);
        axp192_read(&axp, AXP192_COULOMB_COUNTER, &cbat);

        ESP_LOGI(TAG,
            "vacin: %.2fV iacin: %.2fA vvbus: %.2fV ivbus: %.2fA vts: %.2fV temp: %.0fC "
            "pbat: %.2fmW vbat: %.2fV icharge: %.2fA idischarge: %.2fA, vaps: %.2fV "
            "cbat: %.2fmAh",
            vacin, iacin, vvbus, ivbus, vts, temp, pbat, vbat, icharge, idischarge, vaps, cbat
        );

        axp192_ioctl(&axp, AXP192_READ_POWER_STATUS, &power);
        axp192_ioctl(&axp, AXP192_READ_CHARGE_STATUS, &charge);
        ESP_LOGI(TAG,
            "power: 0x%02x charge: 0x%02x",
            power, charge
        );

        strftime(buffer, 128 ,"%c (day %j)" , &rtc);
        ESP_LOGI(TAG, "RTC: %s", buffer);

        vTaskDelay(5000 / portTICK_RATE_MS);

        ESP_LOGI(TAG, "fps: %.1f", fb_fps);
    }
    vTaskDelete(NULL);
}

static void sntp_set_rtc(struct timeval *tv)
{
    struct tm *time;

    ESP_LOGI(TAG, "Got SNTP response, setting RTC.");

    time = localtime(&tv->tv_sec);
    bm8563_write(&bm, time);
}

void app_main()
{
    ESP_LOGI(TAG, "SDK version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Heap when starting: %d", esp_get_free_heap_size());

    static i2c_port_t i2c_port = I2C_NUM_0;

    /* Set your timezone here. */
    setenv("TZ", "<+07>-7", 1);
    tzset();

    ESP_LOGI(TAG, "Initializing I2C");
    i2c_init(i2c_port);

    ESP_LOGI(TAG, "Initializing AXP192");
    axp.read = &i2c_read;
    axp.write = &i2c_write;
    axp.handle = &i2c_port;

    axp192_init(&axp);
    axp192_ioctl(&axp, AXP192_COULOMB_COUNTER_ENABLE, NULL);
    axp192_ioctl(&axp, AXP192_COULOMB_COUNTER_CLEAR, NULL);

    ESP_LOGI(TAG, "Initializing BM8563");
    bm.read = &i2c_read;
    bm.write = &i2c_write;
    bm.handle = &i2c_port;

    rtc.tm_year = 2020 - 1900;
    rtc.tm_mon = 12 - 1;
    rtc.tm_mday = 31;
    rtc.tm_hour = 23;
    rtc.tm_min = 59;
    rtc.tm_sec = 45;

    bm8563_init(&bm);
    bm8563_write(&bm, &rtc);

    ESP_LOGI(TAG, "Setting BM8563 alarm");
    rtc_alarm.tm_min = 30;
    rtc_alarm.tm_hour = 19;
    rtc_alarm.tm_mday = BM8563_ALARM_NONE;
    rtc_alarm.tm_wday = BM8563_ALARM_NONE;

    bm8563_ioctl(&bm, BM8563_ALARM_SET, &rtc_alarm);

    ESP_LOGI(TAG, "Setting BM8563 timer");
    uint8_t count = 10;
    uint8_t reg = BM8563_TIMER_ENABLE | BM8563_TIMER_1HZ;

    bm8563_ioctl(&bm, BM8563_TIMER_WRITE, &count);
    bm8563_ioctl(&bm, BM8563_TIMER_CONTROL_WRITE, &reg);

    ESP_LOGI(TAG, "Initializing display");
    hagl_init();

    ESP_LOGI(TAG, "Initializing non volatile storage");
    nvs_init();

    ESP_LOGI(TAG, "Initializing wifi");
    wifi_init();

    ESP_LOGI(TAG, "Start SNTP sync");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    sntp_set_time_sync_notification_cb(sntp_set_rtc);
    sntp_init();

    ESP_LOGI(TAG, "Heap after init: %d", esp_get_free_heap_size());

    xTaskCreatePinnedToCore(rtc_task, "RTC", 8192, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(log_task, "Log", 8192, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(alarm_timer_task, "Alarm", 8192, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(backbuffer_task, "Backbuffer", 8192, NULL, 1, NULL, 0);
}
