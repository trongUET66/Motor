#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define MOTOR_EN    14   // Chân GPIO để kích hoạt động cơ
#define MOTOR_IN1   12   // Chân GPIO để điều khiển hướng động cơ
#define MOTOR_IN2   13   // Chân GPIO để điều khiển hướng động cơ

#define LEDC_HS_TIMER          LEDC_TIMER_0          // Chọn timer cho LEDC
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE  // Chế độ tốc độ cao cho LEDC
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0        // Kênh 0 của LEDC

#define LEDC_TEST_DUTY         (4000)                // Độ duty cycle thử nghiệm cho PWM
#define MAX_SPEED              (8191)                // Giá trị tối đa của duty cycle (2^13 - 1)

void motor_control(int speed, int direction) {
    gpio_set_level(MOTOR_IN1, direction);            // Đặt hướng quay của động cơ
    gpio_set_level(MOTOR_IN2, !direction);           // Đặt hướng quay của động cơ (nghịch đảo)
    ledc_set_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL, speed);  // Đặt duty cycle cho động cơ
    ledc_update_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL);      // Cập nhật duty cycle
}

void init(void) {
    // Cấu hình GPIO
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << MOTOR_EN) | (1ULL << MOTOR_IN1) | (1ULL << MOTOR_IN2),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE};
    gpio_config(&io_conf);

    // Khởi tạo LEDC
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_HS_TIMER,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t pwm_conf = {
        .gpio_num = MOTOR_EN,
        .speed_mode = LEDC_HS_MODE,
        .channel = LEDC_HS_CH0_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_HS_TIMER,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&pwm_conf);
}


void app_main(void) {
    // Khởi tạo các thiết bị và cấu hình
    init();

    // Dừng động cơ ban đầu
    gpio_set_level(MOTOR_EN, 0);  // Tắt nguồn cho động cơ

    int direction = 1;  // Ban đầu, hướng điều khiển là tiến

    while (1) {
        // Đảo hướng điều khiển của động cơ sau mỗi chu kỳ hoạt động
        direction = !direction;

        for (int duty_cycle = 0; duty_cycle <= MAX_SPEED; duty_cycle += 500)
        {
            motor_control(duty_cycle, 1); // Điều khiển động cơ với tốc độ tăng dần
        }


        //TEST
        // // Chạy động cơ với hướng mới
        // motor_control(2000, direction);

        // Chờ một khoảng thời gian trước khi thực hiện chu kỳ tiếp theo
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Chờ 2 giây
    }
}
