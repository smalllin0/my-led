# MyLed组件

- 用于控制LED的组件

## 功能
1. LED开关控制；
2. LED闪烁（闪烁间隔、次数可调）
3. 呼吸灯效果（使用LEDC通道）
4. 亮度调节（使用LEDC通道）

```
#include "esp_log.h"
#include "my_led.h"

#define TAG "MAIN"

extern "C" void app_main(void)
{
    MyLed led12(GPIO_NUM_12);
    MyLed led13(GPIO_NUM_13);

    led12.StartContinuousBlink();


    led13.EnableBrightnessAdjust(true, LEDC_CHANNEL_1, LEDC_TIMER_1);
    // led13.StartContinuousBlink(500);
    led13.SetBrightness(7);

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(10*1000));
        led13.SetBrightness(0);
        led12.TurnOn();
    }
}

```
