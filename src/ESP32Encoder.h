#pragma once
#include <driver/gpio.h>
#ifndef ARDUINO
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portable.h>
#include <freertos/semphr.h>
#endif

// Migrate from deprecated driver/pcnt.h to driver/pulse_cnt.h (IDF v5+)
#include <driver/pulse_cnt.h>

#define MAX_ESP32_ENCODERS 8   // pcnt_new_unit() supports up to 8 units on ESP32
#define _INT16_MAX  32766
#define _INT16_MIN -32766
#define ISR_CORE_USE_DEFAULT (0xffffffff)

enum class encType  { single, half, full };
enum class puType   { up, down, none };

class ESP32Encoder;
typedef void (*enc_isr_cb_t)(void*);

class ESP32Encoder {
public:
    ESP32Encoder(bool always_interrupt = false,
                 enc_isr_cb_t enc_isr_cb = nullptr,
                 void* enc_isr_cb_data   = nullptr);
    ~ESP32Encoder();

    void    attachHalfQuad   (int aPinNumber, int bPinNumber);
    void    attachFullQuad   (int aPinNumber, int bPinNumber);
    void    attachSingleEdge (int aPinNumber, int bPinNumber);
    int64_t getCount();
    int64_t clearCount();
    int64_t pauseCount();
    int64_t resumeCount();
    void    detach();
    [[deprecated("Replaced by detach")]] void detatch();
    bool    isAttached() { return attached; }
    void    setCount(int64_t value);
    void    setFilter(uint16_t value);   // glitch filter in ns (0 = disable)

    static ESP32Encoder* encoders[MAX_ESP32_ENCODERS];
    static puType   useInternalWeakPullResistors;
    static uint32_t isrServiceCpuCore;

    bool          always_interrupt;
    gpio_num_t    aPinNumber;
    gpio_num_t    bPinNumber;
    enc_isr_cb_t  _enc_isr_cb;
    void*         _enc_isr_cb_data;

    // New IDF v5 PCNT handles
    pcnt_unit_handle_t  pcnt_unit   = nullptr;
    pcnt_channel_handle_t  pcnt_chanA  = nullptr;
    pcnt_channel_handle_t  pcnt_chanB  = nullptr;

    volatile int64_t count = 0;   // software accumulator

private:
    static bool attachedInterrupt;
    void   attach(int aPinNumber, int bPinNumber, encType et);
    bool   attached  = false;
    int    unitIndex = -1;
    uint32_t filterNs = 0;
};

//Added by Sloeber
#pragma once
