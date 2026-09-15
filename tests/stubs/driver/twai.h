#pragma once
#include <Arduino.h>
#include <deque>
#include <vector>
using gpio_num_t = int;
using esp_err_t = int;
constexpr int ESP_OK = 0, ESP_FAIL = -1, TWAI_MODE_NORMAL = 0;
constexpr int TWAI_STATE_RUNNING = 1, TWAI_STATE_BUS_OFF = 2, TWAI_STATE_STOPPED = 3;
inline uint32_t pdMS_TO_TICKS(uint32_t ms) { return ms; }
struct twai_general_config_t { int rx_queue_len = 0, tx_queue_len = 0; };
struct twai_timing_config_t {};
struct twai_filter_config_t {};
#define TWAI_GENERAL_CONFIG_DEFAULT(tx,rx,mode) twai_general_config_t{}
#define TWAI_TIMING_CONFIG_1MBITS() twai_timing_config_t{}
#define TWAI_FILTER_CONFIG_ACCEPT_ALL() twai_filter_config_t{}
struct twai_message_t { bool extd=false, rtr=false; uint32_t identifier=0; uint8_t data_length_code=0; uint8_t data[8]={}; };
struct twai_status_info_t {
    int state=TWAI_STATE_RUNNING;
    uint32_t tx_error_counter=0, rx_error_counter=0, tx_failed_count=0, rx_missed_count=0,
        arb_lost_count=0, bus_error_count=0, msgs_to_tx=0, msgs_to_rx=0;
};
inline std::deque<twai_message_t> fake_can_rx;
inline std::vector<twai_message_t> fake_can_tx;
inline uint32_t fake_can_read_cost_us=10;
inline bool fake_can_tx_fail=false;
inline int fake_can_rx_capacity=0;
inline esp_err_t twai_driver_install(const twai_general_config_t* g, const twai_timing_config_t*, const twai_filter_config_t*) { fake_can_rx_capacity=g->rx_queue_len; return ESP_OK; }
inline esp_err_t twai_start() { return ESP_OK; }
inline esp_err_t twai_stop() { return ESP_OK; }
inline esp_err_t twai_driver_uninstall() { return ESP_OK; }
inline esp_err_t twai_initiate_recovery() { return ESP_OK; }
inline esp_err_t twai_get_status_info(twai_status_info_t* s) { *s={};s->msgs_to_rx=fake_can_rx.size();return ESP_OK; }
inline esp_err_t twai_transmit(const twai_message_t* m, uint32_t) { if(fake_can_tx_fail)return ESP_FAIL;fake_can_tx.push_back(*m);return ESP_OK; }
inline esp_err_t twai_receive(twai_message_t* m, uint32_t wait) {
    fake_us+=fake_can_read_cost_us;
    if(fake_can_rx.empty()) {fake_ms+=wait;return ESP_FAIL;}
    *m=fake_can_rx.front();fake_can_rx.pop_front();return ESP_OK;
}
