// SPDX-License-Identifier: MIT
// NEO-M9N over the M5-Bus UART, plus the 1PPS edge.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "gnss_example.h"

static const char *TAG = "tab5_gnss";

#define GNSS_UART      UART_NUM_1
#define GNSS_RX_PIN    GPIO_NUM_7    // M5-Bus pin 15, M135 NEO_TXD
#define GNSS_TX_PIN    GPIO_NUM_6    // M5-Bus pin 16, M135 NEO_RXD
#define GNSS_BAUD      38400         // NEO-M9N factory default, 8N1
#define GNSS_BUF_LEN   2048

// PPS lands on M5-Bus pin 26 -> G51 with the DIP as set here. The switch has
// three positions, reaching bus pins 2, 24 and 26 (G16, G35, G51 on a Tab5);
// if yours is set differently, one of the other two is live instead. There is
// no way to read the switch from software and the Tab5 is not on the module's
// silkscreen legend, so this is a per-board constant you confirm once.
//
// Confirmed on hardware: 1 Hz, within ~25 us of the nominal second.
#define GNSS_PPS_PIN   GPIO_NUM_51

static volatile uint32_t s_pps_count;
static volatile int64_t  s_pps_last_us;
static volatile int64_t  s_pps_interval_us;

static void IRAM_ATTR pps_isr(void *arg)
{
    int64_t now = esp_timer_get_time();
    if (s_pps_last_us) s_pps_interval_us = now - s_pps_last_us;
    s_pps_last_us = now;
    s_pps_count++;
}

// --- NMEA ---------------------------------------------------------------

// Splits `line` in place on commas. Returns the field count. Empty fields are
// kept as empty strings, which matters: NMEA signals "no fix" by leaving
// fields blank, not by omitting them.
static int nmea_split(char *line, char *fields[], int max)
{
    int n = 0;
    fields[n++] = line;
    for (char *p = line; *p && n < max; p++) {
        if (*p == ',') { *p = '\0'; fields[n++] = p + 1; }
    }
    return n;
}

static bool nmea_checksum_ok(const char *line)
{
    const char *star = strrchr(line, '*');
    if (!star || strlen(star) < 3) return false;
    uint8_t sum = 0;
    for (const char *p = line + 1; p < star; p++) sum ^= (uint8_t)*p;
    return (uint8_t)strtoul(star + 1, NULL, 16) == sum;
}

// ddmm.mmmm -> decimal degrees. The degree field is 2 digits for latitude and
// 3 for longitude, which is why the width is passed in rather than guessed.
static double nmea_coord(const char *s, int deg_digits, char hemi)
{
    if (!s || !*s) return 0.0;
    char degbuf[4] = {0};
    memcpy(degbuf, s, deg_digits);
    double deg = atof(degbuf) + atof(s + deg_digits) / 60.0;
    if (hemi == 'S' || hemi == 'W') deg = -deg;
    return deg;
}

static const char *fix_name(int q)
{
    switch (q) {
    case 0: return "no fix";
    case 1: return "GPS";
    case 2: return "DGPS";
    case 4: return "RTK fixed";
    case 5: return "RTK float";
    case 6: return "dead reckoning";
    default: return "?";
    }
}

static const char *rmc_speed(void);
static const char *rmc_course(void);
static const char *rmc_date(void);

static void handle_gga(char *fields[], int n)
{
    if (n < 10) return;
    int quality = atoi(fields[6]);
    int sats    = atoi(fields[7]);

    if (quality == 0) {
        ESP_LOGI(TAG, "no fix, %d satellite%s used", sats, sats == 1 ? "" : "s");
        return;
    }

    double lat = nmea_coord(fields[2], 2, fields[3][0]);
    double lon = nmea_coord(fields[4], 3, fields[5][0]);

    // GGA field 7 is satellites *used in the solution*, not in view. GSV is
    // what reports view, and the two differ a lot with a poor sky: a receiver
    // tracking 38 and solving with 5 reads as "3 satellites" here, which looks
    // like a dead antenna rather than a marginal fix.
    ESP_LOGI(TAG, "fix          %s, %d satellites used, HDOP %s",
             fix_name(quality), sats, fields[8][0] ? fields[8] : "-");
    ESP_LOGI(TAG, "  position   %.6f, %.6f", lat, lon);
    ESP_LOGI(TAG, "  altitude   %s m MSL, geoid sep %s m",
             fields[9][0] ? fields[9] : "-", n > 11 && fields[11][0] ? fields[11] : "-");
    ESP_LOGI(TAG, "  UTC        %.2s:%.2s:%.2s on %s",
             fields[1], fields[1] + 2, fields[1] + 4, rmc_date());
    ESP_LOGI(TAG, "  speed      %s kn, course %s deg", rmc_speed(), rmc_course());
}

// The M9N emits RMC before GGA within each epoch, so printing RMC as it
// arrives puts speed and date above the fix they belong to. Latch them here
// and let the GGA handler print the whole epoch in one block.
static char s_speed[16] = "-";
static char s_course[16] = "-";
static char s_date[12]  = "-";

static void handle_rmc(char *fields[], int n)
{
    if (n < 10 || fields[2][0] != 'A') return;

    snprintf(s_speed, sizeof(s_speed), "%s", fields[7][0] ? fields[7] : "0");
    snprintf(s_course, sizeof(s_course), "%s", fields[8][0] ? fields[8] : "-");

    // ddmmyy. Day is at offset 0 and year at offset 4; reading them in ISO
    // order gives a plausible-looking date that is wrong in a way nobody
    // notices until the year is not 20xx-ambiguous.
    if (strlen(fields[9]) >= 6) {
        snprintf(s_date, sizeof(s_date), "20%.2s-%.2s-%.2s",
                 fields[9] + 4, fields[9] + 2, fields[9]);
    }
}

static const char *rmc_speed(void)  { return s_speed; }
static const char *rmc_course(void) { return s_course; }
static const char *rmc_date(void)   { return s_date; }

static void handle_line(char *line)
{
    if (line[0] != '$') return;
    if (!nmea_checksum_ok(line)) {
        ESP_LOGW(TAG, "bad NMEA checksum, dropping: %.20s", line);
        return;
    }

#ifdef GNSS_LOG_RAW
    ESP_LOGI(TAG, "raw: %s", line);
#endif

    char *fields[24];
    int n = nmea_split(line, fields, 24);
    // Talker ID varies with constellation mix: GP, GL, GA, GB, GN. Match on
    // the sentence type only, or you will silently ignore multi-GNSS output.
    const char *type = fields[0] + 3;

    if (!strncmp(type, "GGA", 3)) handle_gga(fields, n);
    else if (!strncmp(type, "RMC", 3)) handle_rmc(fields, n);
}

// --- task ---------------------------------------------------------------

static void gnss_task(void *arg)
{
    uint8_t *buf = malloc(GNSS_BUF_LEN);
    char line[128];
    size_t len = 0;

    for (;;) {
        int got = uart_read_bytes(GNSS_UART, buf, GNSS_BUF_LEN, pdMS_TO_TICKS(200));
        for (int i = 0; i < got; i++) {
            char c = (char)buf[i];
            if (c == '\r') continue;
            if (c == '\n') {
                line[len] = '\0';
                if (len) handle_line(line);
                len = 0;
            } else if (len < sizeof(line) - 1) {
                line[len++] = c;
            } else {
                len = 0;   // overlong; resync on the next newline
            }
        }
    }
}

static void pps_report_task(void *arg)
{
    uint32_t last = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        uint32_t now = s_pps_count;
        if (now == last) {
            ESP_LOGI(TAG, "PPS silent on G%d (no time fix yet, or DIP on another pin)",
                     GNSS_PPS_PIN);
        } else {
            ESP_LOGI(TAG, "PPS %lu pulses, last interval %lld us (%+lld us from 1 s)",
                     (unsigned long)now, s_pps_interval_us, s_pps_interval_us - 1000000);
        }
        last = now;
    }
}

esp_err_t gnss_example_start(void)
{
    uart_config_t cfg = {
        .baud_rate = GNSS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(GNSS_UART, GNSS_BUF_LEN * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GNSS_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(GNSS_UART, GNSS_TX_PIN, GNSS_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "NEO-M9N on UART%d, RX=G%d TX=G%d @ %d baud",
             GNSS_UART, GNSS_RX_PIN, GNSS_TX_PIN, GNSS_BAUD);

    gpio_config_t pps = {
        .pin_bit_mask = 1ULL << GNSS_PPS_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&pps));

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    ESP_ERROR_CHECK(gpio_isr_handler_add(GNSS_PPS_PIN, pps_isr, NULL));
    ESP_LOGI(TAG, "watching 1PPS on G%d (M5-Bus pin 26)", GNSS_PPS_PIN);

    xTaskCreate(gnss_task, "gnss", 4096, NULL, 5, NULL);
    xTaskCreate(pps_report_task, "gnss_pps", 3072, NULL, 4, NULL);
    return ESP_OK;
}
