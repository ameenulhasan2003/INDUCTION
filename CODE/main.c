/* STM32CubeIDE Industrial Induction Control Template */
#include "main.h"
#include "stm32f1xx_hal.h"

/* ----- Hardware Pins ----- */
#define TEMP_ADC_CHANNEL   ADC_CHANNEL_0   // NTC/LM35 connected to PA0
#define PWM_TIMER          &htim2
#define PWM_CHANNEL        TIM_CHANNEL_1
#define MODE_BTN_LOW       GPIO_PIN_0
#define MODE_BTN_MED       GPIO_PIN_1
#define MODE_BTN_HIGH      GPIO_PIN_2
#define ESTOP_PIN          GPIO_PIN_3

/* ----- Control Parameters ----- */
float temp_setpoint = 150.0;   // °C default for Medium
float temp_input = 0;
float pwm_duty = 0;
float frequency_hz = 50000;    // starting frequency (Hz)
float f_min = 20000;
float f_max = 100000;

/* PID Controller Variables */
float error = 0;
float integral = 0;
float Kp = 1.0, Ki = 0.01;     // Tune for your coil/system

/* Current Heat Mode */
typedef enum {HEAT_LOW, HEAT_MED, HEAT_HIGH} HeatMode;
HeatMode mode = HEAT_MED;

/* ----- Function Prototypes ----- */
void read_temperature(void);
void update_mode(void);
void pid_control(void);
void set_pwm(float duty, float freq);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();

    HAL_TIM_PWM_Start(PWM_TIMER, PWM_CHANNEL);

    while(1)
    {
        // 1) Emergency Stop Check
        if(HAL_GPIO_ReadPin(GPIOC, ESTOP_PIN) == GPIO_PIN_RESET)
        {
            __HAL_TIM_SET_COMPARE(PWM_TIMER, PWM_CHANNEL, 0); // Stop PWM
            while(1); // halt
        }

        // 2) Read temperature
        read_temperature();

        // 3) Check heat mode buttons
        update_mode();

        // 4) Run PID + Frequency adjustment
        pid_control();

        // 5) Update PWM
        set_pwm(pwm_duty, frequency_hz);

        HAL_Delay(10); // Control loop 100Hz
    }
}

/* --- Functions --- */

void read_temperature(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
    temp_input = ((float)adc_val) * 330.0 / 4095.0; // Example conversion for LM35
}

void update_mode(void)
{
    if(HAL_GPIO_ReadPin(GPIOC, MODE_BTN_LOW) == GPIO_PIN_SET)
    {
        mode = HEAT_LOW; temp_setpoint = 100; frequency_hz = 30000;
    }
    else if(HAL_GPIO_ReadPin(GPIOC, MODE_BTN_MED) == GPIO_PIN_SET)
    {
        mode = HEAT_MED; temp_setpoint = 150; frequency_hz = 50000;
    }
    else if(HAL_GPIO_ReadPin(GPIOC, MODE_BTN_HIGH) == GPIO_PIN_SET)
    {
        mode = HEAT_HIGH; temp_setpoint = 200; frequency_hz = 80000;
    }
}

void pid_control(void)
{
    error = temp_setpoint - temp_input;
    integral += error;
    pwm_duty = Kp * error + Ki * integral;

    if(pwm_duty > 100) pwm_duty = 100;
    if(pwm_duty < 0) pwm_duty = 0;

    // Optional: adjust frequency slightly based on power/load
    frequency_hz += 0.01f * error;
    if(frequency_hz > f_max) frequency_hz = f_max;
    if(frequency_hz < f_min) frequency_hz = f_min;
}

void set_pwm(float duty, float freq)
{
    // Map duty 0-100% to timer CCR value
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(PWM_TIMER);
    __HAL_TIM_SET_COMPARE(PWM_TIMER, PWM_CHANNEL, (uint32_t)(period * duty / 100.0));

    // Optionally adjust ARR for frequency change
    uint32_t new_arr = (uint32_t)(SystemCoreClock / freq) - 1;
    __HAL_TIM_SET_AUTORELOAD(PWM_TIMER, new_arr);
}
