#include "my_led.h"
#include <esp_log.h>
#include <cstring>

#define TAG "MyLed"

MyLed::MyLed(gpio_num_t gpio, bool common_anode)
    : m_led(gpio)
    , m_blink_times(0)
    , m_blink_interval_ms(1000)
    , m_led_is_turnon(false)
    , m_common_anode(common_anode)
    , m_continuous_blink(false)
    , m_blink_timer_is_start(false)
    , m_blink_timer_handle(nullptr)
    , m_brightness_adjust_enable(false)
    , m_brightness(0)
    , m_channel(LEDC_CHANNEL_0)
    , m_timer(LEDC_TIMER_0)
    , m_freq_hz(CONFIG_FREQUENCY)
    , m_resolution(CONFIG_DUTY_RESOLUTION)
    , m_adjust_interval_ms(0)
    , m_breathing_timer_is_start(false)
    , m_breathing_timer_handle(nullptr)
{
    // 配置引脚、关闭输出
    LedInit();

    // 配置定时器
    TimerInit("blink_timer");
}

MyLed::~MyLed()
{
    if (m_brightness_adjust_enable) {
        DisableLEDC();
    } else {
        DeleteTimer(m_blink_timer_handle);
        DeleteTimer(m_breathing_timer_handle);
    }
}

    

void MyLed::LedInit()
{
    if (m_led < GPIO_NUM_0 || m_led >= GPIO_NUM_MAX) {
        ESP_LOGE(TAG, "LED引脚号错误");
        return;
    }
    ESP_LOGI(TAG, "初始化LED，引脚: %d,共阳极: %s", m_led, m_common_anode ? "yes" : "no");

    gpio_config_t led_config = {
        .pin_bit_mask = 1ULL << m_led,
                             .mode = GPIO_MODE_OUTPUT,
                             .pull_up_en = GPIO_PULLUP_DISABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type = GPIO_INTR_DISABLE
    };
    std::lock_guard<std::mutex> lock(m_mutex);
    ESP_ERROR_CHECK(gpio_config(&led_config));
    gpio_set_level(m_led, m_common_anode ? 0 : 1);
}


/// @brief 初始化定时器
/// @param name 定时器名称,blink_timer：闪烁定时器(默认)，其他adjust_timer：调节定时器
void MyLed::TimerInit(const char* name)
{
    led_timer_t* timer_handle_ptr = nullptr;

    #ifdef CONFIG_LED_USE_ESP_TIMER
        esp_timer_create_args_t timer_args = {
            .arg = this,
            #ifdef CONFIG_ESP_TIMER_DISPATCH_TASK
                .dispatch_method = ESP_TIMER_TASK,
            #elif CONFIG_ESP_TIMER_DISPATCH_ISR
                .dispatch_method = ESP_TIMER_ISR,
            #endif
            .name = name,
            .skip_unhandled_events = true
        };
        if (strcmp(name, "blink_timer") == 0) {
            timer_handle_ptr = &m_blink_timer_handle;
            timer_args.callback = [](void *arg)
            {
                auto* led = static_cast<MyLed *>(arg);
                led->HandleBlinkerTimerEvent();
            };
        } else if (strcmp(name, "adjust_timer") == 0) {
            timer_handle_ptr = &m_breathing_timer_handle;
            timer_args.callback = [](void *arg)
            {
                auto* led = static_cast<MyLed *>(arg);
                led->HandleBreathingAdjustTimerEvent();
            };
        } else {
            ESP_LOGE(TAG, "定时器名称错误");
            return;
        }
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, timer_handle_ptr));
        ESP_LOGI(TAG, "Using ESP Timer for %s control", name);        
    #elif CONFIG_LED_USE_FREERTOS_TIMER
        if (strcmp(name, "blink_timer") == 0) {
            timer_handle_ptr = &m_blink_timer_handle;
            *timer_handle_ptr = xTimerCreate(
                name,
                pdMS_TO_TICKS(1000),
                pdTRUE,
                (void *)this,
                [](TimerHandle_t xTimer) {
                    auto* led = (MyLed*)pvTimerGetTimerID(xTimer);
                    led->HandleBlinkTimerEvent();
                }
            );
        } else if (strcmp(name, "adjust_timer") == 0) {
            timer_handle_ptr = &m_breathing_timer_handle;
            *timer_handle_ptr = xTimerCreate(
                name,
                pdMS_TO_TICKS(1000),
                pdTRUE,
                (void *)this,
                [](TimerHandle_t xTimer) {
                    auto* led = (MyLed*)pvTimerGetTimerID(xTimer);
                    led->HandleBreathingAdjustTimerEvent();
                }
            );
        } else {
            ESP_LOGE(TAG, "定时器名称错误");
            return;
        }
        ESP_LOGI(TAG, "Using FreeRTOS Timer for %s control", name); 
    #elif CONFIG_LED_USE_GPTIMER
        gptimer_config_t timer_config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1000 * 1000,                 // 1us
        };
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = 1000 * 1000,
            .reload_count = 0,
            .flags = {
                .auto_reload_on_alarm = true,
            },
        };
        gptimer_event_callbacks_t gptimer_callback = {};
        if (strcmp(name, "blink_timer") == 0) {
            gptimer_callback.on_alarm = [] (gptimer_handle_t timer 
                ,const gptimer_alarm_event_data_t *edata
                , void *user_ctx) -> bool {
                    auto* led = static_cast<MyLed *>(user_ctx);
                    led->HandleBlinkerTimerEvent();
                    return true;
            };
        } else if (strcmp(name, "adjust_timer") == 0) {
            gptimer_callback.on_alarm = [] (gptimer_handle_t timer 
                ,const gptimer_alarm_event_data_t *edata
                , void *user_ctx) -> bool {
                    auto* led = static_cast<MyLed *>(user_ctx);
                    led->HandleBreathingAdjustTimerEvent();
                    return true;
            };
        } else {
            ESP_LOGE(TAG, "定时器名称错误");
            return;
        }
        ESP_LOGI(TAG, "Using GPTimer for %s control", name);        
        ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, timer_handle_ptr));
        ESP_ERROR_CHECK(gptimer_set_alarm_action(*timer_handle_ptr, &alarm_config));
        ESP_ERROR_CHECK(gptimer_register_event_callbacks(*timer_handle_ptr, &gptimer_callback, this));
        ESP_ERROR_CHECK(gptimer_enable(*timer_handle_ptr));  
    #endif   
}

void MyLed::StartTimer(led_timer_t handle)
{
    auto interval = m_blink_interval_ms;
    auto* start_flag = &m_blink_timer_is_start;
    if (handle) { 
        if (handle != m_blink_timer_handle) {
            interval = m_adjust_interval_ms;
            start_flag = &m_breathing_timer_is_start;
        } 
    } else {
        ESP_LOGW(TAG, "定时器未初始化");
        return;
    }
    if (*start_flag) {
        return;
    }

    #ifdef CONFIG_LED_USE_ESP_TIMER
        esp_timer_stop(handle);
        esp_timer_start_periodic(handle, interval * 1000);
    #elif CONFIG_LED_USE_FREERTOS_TIMER
        xTimerStop(handle, 0);
        xTimerChangePeriod(handle, pdMS_TO_TICKS(interval), 0);
    #elif CONFIG_LED_USE_GPTIMER
        gptimer_stop(handle);
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = interval * 1000,
            .reload_count = 0,
            .flags = {
                .auto_reload_on_alarm = true,
            },
        };
        ESP_ERROR_CHECK(gptimer_set_alarm_action(handle, &alarm_config));
        gptimer_start(handle);
    #endif
    *start_flag = true;
}

void MyLed::StopTimer(led_timer_t handle)
{
    auto* start_flag = &m_blink_timer_is_start;
    if (handle) { 
        if (handle != m_blink_timer_handle) {
            start_flag = &m_breathing_timer_is_start;
        } 
    } else {
        ESP_LOGW(TAG, "定时器未初始化");
        return;
    }
    if (!*start_flag) {
        return;
    }


    #ifdef CONFIG_LED_USE_ESP_TIMER
        esp_timer_stop(handle);
    #elif CONFIG_LED_USE_FREERTOS_TIMER
        xTimerStop(handle, 0);
    #elif CONFIG_LED_USE_GPTIMER
        gptimer_stop(handle);
    #endif
    *start_flag = false;
}

void MyLed::DeleteTimer(led_timer_t& handle)
{
    bool* start_flag = nullptr;
    if (handle) { 
        if (handle == m_blink_timer_handle) {
            start_flag = &m_blink_timer_is_start;
        } else if (handle == m_breathing_timer_handle) {
            start_flag = &m_breathing_timer_is_start;
        } else {
            ESP_LOGW(TAG, "定时器句柄错误");
            return;
        }
    } else {
        ESP_LOGW(TAG, "定时器未初始化");
        return;
    }

    if (start_flag && *start_flag) {
        StopTimer(handle);
    }

    #ifdef CONFIG_LED_USE_ESP_TIMER
        esp_timer_delete(handle);
    #elif CONFIG_LED_USE_FREERTOS_TIMER
        xTimerDelete(handle, 0);
    #elif CONFIG_LED_USE_GPTIMER
        gptimer_del_timer(handle);
    #endif
    handle = nullptr;
}

void MyLed::LEDCInit(ledc_channel_t channel, ledc_timer_t timer)
{
    if (timer >= LEDC_TIMER_MAX) {
        ESP_LOGW(TAG, "定时器参数超出范围");
        return;
    }
    if (channel >= LEDC_CHANNEL_MAX) {
        ESP_LOGW(TAG, "通道号参数超出范围");
        return;
    }
    // 禁用定时器
    gpio_reset_pin(m_led);
    ledc_timer_config_t ledc_timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_7_BIT,
        .timer_num = timer,
        .freq_hz = CONFIG_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_cfg));

    ledc_channel_config_t ledc_channel_cfg;
    ledc_channel_cfg.gpio_num = m_led;
    ledc_channel_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel_cfg.channel = channel;
    ledc_channel_cfg.intr_type = LEDC_INTR_DISABLE;
    ledc_channel_cfg.timer_sel = timer;
    ledc_channel_cfg.duty = 1 << (LEDC_TIMER_7_BIT - 1);
    ledc_channel_cfg.hpoint = 0;
    ledc_channel_cfg.sleep_mode =LEDC_SLEEP_MODE_NO_ALIVE_NO_PD;
    ledc_channel_cfg.flags.output_invert = m_common_anode ? (unsigned int) 0 : (unsigned int)1;
    m_channel = channel;
    m_timer = timer;
    m_brightness = ledc_channel_cfg.duty;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_cfg));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, m_channel, m_brightness);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, m_channel);
}

void MyLed::DisableLEDC()
{
    ledc_timer_config_t ledc_timer_cfg;
    ledc_timer_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer_cfg.timer_num = m_timer;
    ledc_timer_cfg.deconfigure = true;
    ESP_ERROR_CHECK(ledc_timer_pause(LEDC_LOW_SPEED_MODE, m_timer));
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_cfg));
}

void MyLed::TurnOn()
{
    if (m_blink_timer_is_start) {
        StopTimer(m_blink_timer_handle);
    }
    if (m_breathing_timer_is_start) {
        StopBreathing();
        EnableBrightnessAdjust(false);
    }
    gpio_set_level(m_led, m_common_anode ? 1 : 0);  
}

void MyLed::TurnOff()
{
    if (m_blink_timer_is_start) {
        StopTimer(m_blink_timer_handle);
    }
    if (m_breathing_timer_is_start) {
        StopBreathing();
        EnableBrightnessAdjust(false);
    }
    gpio_set_level(m_led, m_common_anode ? 0 : 1);  
}

void MyLed::BlinkOn()
{
    if (m_led_is_turnon) {
        return;
    } else {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_brightness_adjust_enable) {
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, m_channel, m_brightness));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, m_channel));
        } else {
            gpio_set_level(m_led, m_common_anode ? 1 : 0);  
        }
        m_led_is_turnon = true;
    }
}

void MyLed::BlinkOff()
{
    if (!m_led_is_turnon) {
        return;
    } else {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_brightness_adjust_enable) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, m_channel, 0);
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, m_channel));
        } else { 
            gpio_set_level(m_led, m_common_anode ? 0 : 1);  
        }
        m_led_is_turnon = false;
    }
}

void MyLed::BlinkOnce(interval_t interval_ms)
{
    m_blink_times = 2;
    m_continuous_blink = false;
    m_blink_interval_ms = interval_ms;
    StartTimer(m_blink_timer_handle);
    BlinkOn();
}

void MyLed::Blink(int times, interval_t interval_ms)
{
    m_blink_times = times << 1;
    m_blink_interval_ms = interval_ms;
    m_continuous_blink = false;
    StartTimer(m_blink_timer_handle);
    BlinkOn();
}

void MyLed::StartContinuousBlink(interval_t interval_ms)
{
    m_continuous_blink = true;
    m_blink_interval_ms = interval_ms;
    StartTimer(m_blink_timer_handle);
    BlinkOn();
}

void MyLed::EnableBrightnessAdjust(bool enable, ledc_channel_t channel, ledc_timer_t timer)
{
    if ( enable == m_brightness_adjust_enable) {
        return;
    }

    m_brightness_adjust_enable = enable;
    if (enable) {
        // 启用LEDC
        LEDCInit(channel, timer);
    } else {
        // 禁用LEDC
        DisableLEDC();
        // 恢复引脚控制
        gpio_reset_pin(m_led);
        LedInit();
    }
}

void MyLed::SetBrightness(uint32_t brightness)
{
    if (!m_brightness_adjust_enable) {
        ESP_LOGW(TAG, "未开启亮度调节，无法启动呼吸灯");
        return;
    }

    if (m_brightness == brightness || (m_brightness == MAX_BRIGHTNESS && brightness > MAX_BRIGHTNESS)) {
        return;
    }

    m_brightness = brightness > MAX_BRIGHTNESS ? MAX_BRIGHTNESS : brightness;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, m_channel, m_brightness);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, m_channel);
}

void MyLed::StartBreathing(interval_t adjust_ms)
{
    if (!m_brightness_adjust_enable) {
        ESP_LOGW(TAG, "未开启亮度调节，无法启动呼吸灯");
        return;
    }
    StopTimer(m_blink_timer_handle);
    m_adjust_interval_ms = adjust_ms;
    TimerInit("adjust_timer");
    StartTimer(m_breathing_timer_handle);
    EnableBrightnessAdjust(true);
}

void MyLed::StopBreathing()
{
    if (!m_brightness_adjust_enable) {
        ESP_LOGW(TAG, "未开启亮度调节，无法停止呼吸灯");
        return;
    }
    StopTimer(m_breathing_timer_handle);
    DeleteTimer(m_breathing_timer_handle);
}

void MyLed::HandleBlinkTimerEvent()
{
    if (--m_blink_times & 1) {
        BlinkOn();
    } else {
        BlinkOff();
        if (m_blink_times == 0 && !m_continuous_blink) {
            #ifdef CONFIG_LED_USE_ESP_TIMER
                esp_timer_stop(m_blink_timer_handle);
            #elif CONFIG_LED_USE_FREERTOS_TIMER
                xTimerStop(m_blink_timer_handle, 0);
            #elif CONFIG_LED_USE_GPTIMER
                gptimer_stop(m_blink_timer_handle);
            #endif
            m_blink_timer_is_start = false;
        }
    }
}

void MyLed::HandleBreathingAdjustTimerEvent()
{
    static bool increasing = true; // 亮度增加标志
    static auto brightness = m_brightness;
    uint8_t step = brightness > (MAX_BRIGHTNESS >> 1) ? 2 : 1;
    if (increasing) {
        if (brightness < MAX_BRIGHTNESS) {
            brightness += step;
        } else {
            increasing = false; // 达到最大亮度，开始减小
            brightness -= step;
        }
    } else {
        if (brightness > 1) {
            brightness -= step;
        } else {
            increasing = true; // 达到最小亮度，开始增大
            brightness += step;
        }
    }
    SetBrightness(brightness);
}
