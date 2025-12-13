#ifndef _MY_LED_H_
#define _MY_LED_H_

#include <mutex>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#ifdef CONFIG_LED_USE_ESP_TIMER
#include "esp_timer.h"
#elif CONFIG_LED_USE_FREERTOS_TIMER
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#elif CONFIG_LED_USE_GPTIMER
#include "driver/gptimer.h"
#endif

#define CONFIG_FREQUENCY        1000                 // 1kHz频率
#define CONFIG_DUTY_RESOLUTION  LEDC_TIMER_7_BIT


#define MAX_BRIGHTNESS         ((1 << CONFIG_DUTY_RESOLUTION) - 1)


#ifdef CONFIG_LED_USE_ESP_TIMER
    using led_timer_t   = esp_timer_handle_t;
#elif CONFIG_LED_USE_GPTIMER
    using led_timer_t   = gptimer_handle_t;
#else 
    using led_timer_t   = TimerHandle_t;
#endif
using interval_t = uint32_t;

class MyLed {
public:
    MyLed(gpio_num_t gpio, bool common_anode = true);
    ~MyLed();

    virtual void BlinkOnce(interval_t interval_ms = 1000);
    virtual void Blink(int times, interval_t interval_ms = 1000);
    virtual void StartContinuousBlink(interval_t interval_ms = 1000);
    void EnableBrightnessAdjust(bool enable, ledc_channel_t channel = LEDC_CHANNEL_0, ledc_timer_t timer = LEDC_TIMER_0);
    virtual void StartBreathing(interval_t adjust_ms = 30); 
    virtual void StopBreathing();
    virtual void TurnOn();
    virtual void TurnOff();
    virtual void SetBrightness(uint32_t brightness);
    inline  void DisableBrightnessAdjust() { EnableBrightnessAdjust(false); }

private:
    std::mutex  m_mutex;                    // 互斥锁，防止线程对资源同时进行操作
    gpio_num_t  m_led;                      // LED 引脚
    int         m_blink_times;              // 闪烁次数
    interval_t  m_blink_interval_ms;        // 闪烁间隔
    bool        m_led_is_turnon;            // 是否打开
    bool        m_common_anode;             // 是否为共阳极
    bool        m_continuous_blink;         // 是否连续闪烁
    bool        m_blink_timer_is_start;     // 是否开始定时
    led_timer_t m_blink_timer_handle;       // 闪烁定时器句柄


    bool            m_brightness_adjust_enable;     // 是否支持亮度调节
    uint32_t        m_brightness;                   // 亮度
    ledc_channel_t  m_channel;                      // LEDC通道
    ledc_timer_t    m_timer;
    uint16_t        m_freq_hz;                      // PWM调节频率
    uint32_t        m_resolution;                   // 分辨率
    interval_t      m_adjust_interval_ms;           // 亮度调节间隔
    bool            m_breathing_timer_is_start;     // 呼吸灯定时器是否开始
    led_timer_t     m_breathing_timer_handle;       // 呼吸灯调节定时器句柄

    void LedInit();
    void TimerInit(const char* name);
    void StartTimer(led_timer_t handle);
    void StopTimer(led_timer_t handle);
    void DeleteTimer(led_timer_t& handle);
    void LEDCInit(ledc_channel_t channel, ledc_timer_t timer);
    void BlinkOff();
    void BlinkOn();

    void DisableLEDC();
    void HandleBlinkTimerEvent();
    void HandleBreathingAdjustTimerEvent();
};

#endif