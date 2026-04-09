/*
 * ESP32Encoder.cpp  –  migrated to driver/pulse_cnt.h (IDF v5 / Arduino-ESP32 v3+)
 *
 * Replaces the deprecated driver/pcnt.h (legacy) API that produced:
 *   warning: "legacy pcnt driver is deprecated, please migrate to use driver/pulse_cnt.h"
 */

#include "ESP32Encoder.h"
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <rom/gpio.h>
#define delay(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#endif

#include <soc/soc_caps.h>
#if SOC_PCNT_SUPPORTED

#include "esp_log.h"
static const char* TAG_ENCODER __attribute__((unused)) = "ESP32Encoder";

// ── Static member definitions ─────────────────────────────────────────────────
puType   ESP32Encoder::useInternalWeakPullResistors = puType::down;
uint32_t ESP32Encoder::isrServiceCpuCore            = ISR_CORE_USE_DEFAULT;
ESP32Encoder* ESP32Encoder::encoders[MAX_ESP32_ENCODERS] = {};
bool ESP32Encoder::attachedInterrupt = false;

// ── Watch-point callback – fires when accumulator overflows the HW limit ──────
static bool IRAM_ATTR pcnt_on_reach(pcnt_unit_handle_t unit,
                                    const pcnt_watch_event_data_t* edata,
                                    void* user_ctx)
{
    ESP32Encoder* enc = static_cast<ESP32Encoder*>(user_ctx);
    // edata->watch_point_val is the limit that was hit (+_INT16_MAX or _INT16_MIN)
    enc->count += edata->watch_point_value;
    pcnt_unit_clear_count(unit);

    if (enc->always_interrupt && enc->_enc_isr_cb) {
        enc->_enc_isr_cb(enc->_enc_isr_cb_data);
    }
    return false; // no task wakeup needed
}

// ── Constructor / Destructor ──────────────────────────────────────────────────
ESP32Encoder::ESP32Encoder(bool always_interrupt_, enc_isr_cb_t cb, void* cb_data)
    : always_interrupt(always_interrupt_),
      aPinNumber((gpio_num_t)0),
      bPinNumber((gpio_num_t)0),
      _enc_isr_cb(cb),
      _enc_isr_cb_data(cb_data ? cb_data : this),
      count(0),
      attached(false),
      unitIndex(-1)
{}

ESP32Encoder::~ESP32Encoder() {
    if (attached) detach();
}

// ── detach ────────────────────────────────────────────────────────────────────
void ESP32Encoder::detach() {
    if (!attached) return;
    pcnt_unit_stop(pcnt_unit);
    pcnt_unit_disable(pcnt_unit);
    pcnt_del_channel(pcnt_chanA);
    if (pcnt_chanB) pcnt_del_channel(pcnt_chanB);
    pcnt_del_unit(pcnt_unit);
    pcnt_unit  = nullptr;
    pcnt_chanA = nullptr;
    pcnt_chanB = nullptr;
    if (unitIndex >= 0) {
        encoders[unitIndex] = nullptr;
        unitIndex = -1;
    }
    attached = false;
}

void ESP32Encoder::detatch() { detach(); }

// ── Internal attach ───────────────────────────────────────────────────────────
void ESP32Encoder::attach(int a, int b, encType et) {
    if (attached) {
        ESP_LOGE(TAG_ENCODER, "attach: already attached");
        return;
    }

    // Find a free slot
    int index = -1;
    for (int i = 0; i < MAX_ESP32_ENCODERS; i++) {
        if (encoders[i] == nullptr) { index = i; break; }
    }
    if (index < 0) {
        ESP_LOGE(TAG_ENCODER, "Too many encoders!");
        return;
    }
    unitIndex       = index;
    encoders[index] = this;
    aPinNumber = (gpio_num_t)a;
    bPinNumber = (gpio_num_t)b;

    // ── Configure GPIO pull resistors ─────────────────────────────────────────
    gpio_set_direction(aPinNumber, GPIO_MODE_INPUT);
    gpio_set_direction(bPinNumber, GPIO_MODE_INPUT);
    if (useInternalWeakPullResistors == puType::down) {
        gpio_pulldown_en(aPinNumber);
        gpio_pulldown_en(bPinNumber);
    } else if (useInternalWeakPullResistors == puType::up) {
        gpio_pullup_en(aPinNumber);
        gpio_pullup_en(bPinNumber);
    }

    // ── Create PCNT unit ──────────────────────────────────────────────────────
    pcnt_unit_config_t unit_cfg = {};
    unit_cfg.high_limit = _INT16_MAX;
    unit_cfg.low_limit  = _INT16_MIN;
    unit_cfg.flags.accum_count = 1;   // enable HW accumulation on overflow
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &pcnt_unit));

    // ── Glitch filter ─────────────────────────────────────────────────────────
    if (filterNs > 0) {
        pcnt_glitch_filter_config_t filter_cfg = {};
        filter_cfg.max_glitch_ns = filterNs;
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_cfg));
    }

    // ── Channel A: signal=A, control=B ───────────────────────────────────────
    pcnt_chan_config_t chanA_cfg = {};
    chanA_cfg.edge_gpio_num = a;
    chanA_cfg.level_gpio_num = b;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chanA_cfg, &pcnt_chanA));

    // Rising edge on A:  B low  → count up,  B high → count down
    // Falling edge on A: B low  → count down (half/full), B high → count up
    pcnt_channel_set_edge_action(pcnt_chanA,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,  // rising  edge
        et != encType::single ? PCNT_CHANNEL_EDGE_ACTION_DECREASE
                              : PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_channel_set_level_action(pcnt_chanA,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,     // level low  → keep direction
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE); // level high → invert

    // ── Channel B: signal=B, control=A (full-quad only) ──────────────────────
    if (et == encType::full) {
        pcnt_chan_config_t chanB_cfg = {};
        chanB_cfg.edge_gpio_num  = b;
        chanB_cfg.level_gpio_num = a;
        ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chanB_cfg, &pcnt_chanB));

        pcnt_channel_set_edge_action(pcnt_chanB,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        pcnt_channel_set_level_action(pcnt_chanB,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP);
    }

    // ── Watch points (overflow/underflow) + callback ─────────────────────────
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, _INT16_MAX));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, _INT16_MIN));

    pcnt_event_callbacks_t cbs = {};
    cbs.on_reach = pcnt_on_reach;
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, this));

    // ── Enable and start ──────────────────────────────────────────────────────
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    attached = true;
}

// ── Public attach helpers ─────────────────────────────────────────────────────
void ESP32Encoder::attachHalfQuad   (int a, int b) { attach(a, b, encType::half);   }
void ESP32Encoder::attachSingleEdge (int a, int b) { attach(a, b, encType::single); }
void ESP32Encoder::attachFullQuad   (int a, int b) { attach(a, b, encType::full);   }

// ── Count operations ──────────────────────────────────────────────────────────
int64_t ESP32Encoder::getCount() {
    int raw = 0;
    pcnt_unit_get_count(pcnt_unit, &raw);
    return count + raw;
}

void ESP32Encoder::setCount(int64_t value) {
    int raw = 0;
    pcnt_unit_get_count(pcnt_unit, &raw);
    count = value - raw;
}

int64_t ESP32Encoder::clearCount() {
    count = 0;
    pcnt_unit_clear_count(pcnt_unit);
    return 0;
}

int64_t ESP32Encoder::pauseCount() {
    pcnt_unit_stop(pcnt_unit);
    return getCount();
}

int64_t ESP32Encoder::resumeCount() {
    pcnt_unit_start(pcnt_unit);
    return getCount();
}

// ── Glitch filter (ns) ────────────────────────────────────────────────────────
// Must be called before attach(); if called after, it re-applies the filter.
void ESP32Encoder::setFilter(uint16_t value_ns) {
    filterNs = value_ns;
    if (!attached || !pcnt_unit) return;
    if (value_ns == 0) {
        pcnt_unit_set_glitch_filter(pcnt_unit, nullptr);
    } else {
        pcnt_glitch_filter_config_t f = {};
        f.max_glitch_ns = value_ns;
        pcnt_unit_set_glitch_filter(pcnt_unit, &f);
    }
}

#else
#warning PCNT not supported on this SoC
#endif // SOC_PCNT_SUPPORTED
