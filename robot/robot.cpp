#include <cstdint>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include <atomic>
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

constexpr char kSsid[] = "palantir";
constexpr char kPass[] = "LIVElife";
constexpr uint16_t kPort = 5005;
std::atomic<uint16_t> receivedByte{0};
constexpr float kServoPeriodUs = 20000.0f;
constexpr float kServoMinUs = 600.0f;  // pulse for 0 degrees
constexpr float kServoMaxUs = 2400.0f; // pulse for 180 degrees
constexpr uint servo1Pin;         // SERVO1 -> GP0
constexpr uint servo2Pin;        // SERVO2 -> GP28
constexpr int sleep = 1;
constexpr int minAngle=5;
constexpr int maxAngle=175;

//-- motors
constexpr int u1;
constexpr int u2;

constexpr int topRightIN1;
constexpr int topRightIN2;
constexpr int topRightPWM;

constexpr int topLeftIN1;
constexpr int topLeftIN2;
constexpr int topLeftPWM;

constexpr int bottomRightIN1;
constexpr int bottomRightIN2;
constexpr int bottomRightPWM;

constexpr int bottomLeftIN1;
constexpr int bottomLeftIN2;
constexpr int bottomLeftPWM;

constexpr int maxSpeed = 65000;
constexpr int baseSpeed = 43800;


void setup_gpio(uint pin, bool initial = false)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, initial);
}

void setup_pwm(uint pin, float freq_hz)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, 65535);
    float clkdiv = (float)clock_get_hz(clk_sys) / (freq_hz * 65536.0f);
    pwm_config_set_clkdiv(&cfg, clkdiv);

    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(pin, 0);
}

// ---- Motor helpers (in1/in2 = direction pins, en = PWM pin) ----
void motor_forward(uint in1, uint in2, uint en, uint16_t duty /* 0..65535 */)
{
    gpio_put(in1, 1);
    gpio_put(in2, 0);
    pwm_set_gpio_level(en, duty);
}

void motor_backward(uint in1, uint in2, uint en, uint16_t duty /* 0..65535 */)
{
    gpio_put(in1, 0);
    gpio_put(in2, 1);
    pwm_set_gpio_level(en, duty);
}

void motor_stop(uint in1, uint in2, uint en)
{ // coast
    gpio_put(in1, 0);
    gpio_put(in2, 0);
    pwm_set_gpio_level(en, 0);
}

static int servo_set_angle(uint pin, int degrees)
{
    if (degrees < minAngle)
        degrees = minAngle;
    if (degrees > maxAngle)
        degrees = maxAngle;

    float pulseUs = kServoMinUs + (degrees / 180.0f) * (kServoMaxUs - kServoMinUs);
    uint16_t level = (uint16_t)((pulseUs / kServoPeriodUs) * 65536.0f);
    pwm_set_gpio_level(pin, level);
    return degrees;
}

static void onUdpReceive(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                         const ip_addr_t *addr, u16_t port)
{
    (void)arg;
    (void)pcb;
    (void)addr;
    (void)port;

    if (p == nullptr)
        return;

    if (p->tot_len >= 2)
    {
        uint16_t v = pbuf_get_at(p, 0) | (pbuf_get_at(p, 1) << 8);
        receivedByte.store(v, std::memory_order_release);
    }
    pbuf_free(p);
}

int main()
{

    stdio_init_all();

    if (cyw43_arch_init())
    {
        return 1;
    }
    cyw43_arch_enable_sta_mode();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    setup_gpio(u1, true);
    setup_gpio(u2, true);

    setup_gpio(topRightIN1);
    setup_gpio(topRightIN2);
    setup_pwm(topRightPWM, 1000.0f);

    setup_gpio(topLeftIN1);
    setup_gpio(topLeftIN2);
    setup_pwm(topLeftPWM, 1000.0f);

    setup_gpio(bottomRightIN1);
    setup_gpio(bottomRightIN2);
    setup_pwm(bottomRightPWM, 1000.0f);

    setup_gpio(bottomLeftIN1);
    setup_gpio(bottomLeftIN2);
    setup_pwm(bottomLeftPWM, 1000.0f);

    // ---- Servos: both start at 0 degrees ----
    setup_pwm(servo1Pin, 50.0f);
    setup_pwm(servo2Pin, 50.0f);

    if (cyw43_arch_wifi_connect_timeout_ms(kSsid, kPass, CYW43_AUTH_WPA3_WPA2_AES_PSK, 30000))
    {
        cyw43_arch_deinit();
        return 1;
    }

    // Keep the radio awake for lower receive latency
    cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);

    // lwIP calls from the main thread must be wrapped in begin/end
    cyw43_arch_lwip_begin();
    udp_pcb *pcb = udp_new();
    if (pcb == nullptr || udp_bind(pcb, IP_ANY_TYPE, kPort) != ERR_OK)
    {
        cyw43_arch_lwip_end();
        return 1;
    }
    udp_recv(pcb, onUdpReceive, nullptr);
    cyw43_arch_lwip_end();

    uint16_t current = 0;
    constexpr int differenceDegree = 15;
    int servoOneDegree = 0;
    servo_set_angle(servo1Pin, servoOneDegree);
    int servoTwoDegree = 0;
    servo_set_angle(servo2Pin, servoTwoDegree);

    while (true)
    {
        uint16_t load = receivedByte.load(std::memory_order_acquire);
        if (load != current)
        {
            current = load;
            bool w = current & (1 << 0);
            bool s = current & (1 << 1);
            bool a = current & (1 << 2);
            bool d = current & (1 << 3);
            bool c = current & (1 << 8);
            bool shift = current & (1 << 4);
            bool quite = current & (1 << 7);
            bool left = current & (1 << 5);
            bool right = current & (1 << 6);
            bool control = current & (1 << 9);

            if (quite)
            {
                motor_stop(topRightIN1, topRightIN2, topRightPWM);
                motor_stop(topLeftIN1, topLeftIN2, topLeftPWM);
                motor_stop(bottomRightIN1, bottomRightIN2, bottomRightPWM);
                motor_stop(bottomLeftIN1, bottomLeftIN2, bottomLeftPWM);
                servoOneDegree = servo_set_angle(servo1Pin, 0);
                servoTwoDegree = servo_set_angle(servo2Pin, 0);
                continue;
            }

            if (left)
            {
                if (control)
                {
                    servoOneDegree -= differenceDegree;
                }
                else
                {
                    servoOneDegree += differenceDegree;
                }
                servoOneDegree = servo_set_angle(servo1Pin, servoOneDegree);
            }
            if (right)
            {
                if (control)
                {
                    servoTwoDegree -= differenceDegree;
                }
                else
                {
                    servoTwoDegree += differenceDegree;
                }
                servoTwoDegree = servo_set_angle(servo2Pin, servoTwoDegree);
            }

            uint16_t speed = baseSpeed;

            speed += shift ? (maxSpeed - speed) : 0;

            if (c && d)
            {
                motor_forward(topLeftIN1, topLeftIN2, topLeftPWM, speed);
                motor_backward(topRightIN1, topRightIN2, topRightPWM, speed);
                motor_forward(bottomLeftIN1, bottomLeftIN2, bottomLeftPWM, speed);
                motor_backward(bottomRightIN1, bottomRightIN2, bottomRightPWM, speed);
            }
            else if (c && a)
            {
                motor_backward(topLeftIN1, topLeftIN2, topLeftPWM, speed);
                motor_forward(topRightIN1, topRightIN2, topRightPWM, speed);
                motor_backward(bottomLeftIN1, bottomLeftIN2, bottomLeftPWM, speed);
                motor_forward(bottomRightIN1, bottomRightIN2, bottomRightPWM, speed);
            }
            else if (w)
            {
                if (d)
                {
                    motor_stop(topRightIN1, topRightIN2, topRightPWM);
                    motor_forward(topLeftIN1, topLeftIN2, topLeftPWM, speed);
                    motor_stop(bottomLeftIN1, bottomLeftIN2, bottomLeftPWM);
                    motor_forward(bottomRightIN1, bottomRightIN2, bottomRightPWM, speed);
                }
                else if (a)
                {
                    motor_stop(topLeftIN1, topLeftIN2, topLeftPWM);
                    motor_forward(topRightIN1, topRightIN2, topRightPWM, speed);
                    motor_stop(bottomRightIN1, bottomRightIN2, bottomRightPWM);
                    motor_forward(topLeftIN1, topLeftIN2, topLeftPWM, speed);
                }
                else
                {
                    motor_forward(topRightIN1, topRightIN2, topRightPWM, speed);
                    motor_forward(topLeftIN1, topLeftIN2, topLeftPWM, speed);
                    motor_forward(bottomRightIN1, bottomRightIN2, bottomRightPWM, speed);
                    motor_forward(bottomLeftIN1, bottomLeftIN2, bottomLeftPWM, speed);
                }
            }
            else if (s)
            {
                if (d)
                {
                    motor_stop(topRightIN1, topRightIN2, topRightPWM);
                    motor_backward(topLeftIN1, topLeftIN2, topLeftPWM, speed);
                    motor_stop(bottomLeftIN1, bottomLeftIN2, bottomLeftPWM);
                    motor_backward(bottomRightIN1, bottomRightIN2, bottomRightPWM, speed);
                }
                else if (a)
                {
                    motor_stop(topLeftIN1, topLeftIN2, topLeftPWM);
                    motor_backward(topRightIN1, topRightIN2, topRightPWM, speed);
                    motor_stop(bottomRightIN1, bottomRightIN2, bottomRightPWM);
                    motor_backward(bottomLeftIN1, bottomLeftIN2, bottomLeftPWM, speed);
                }
                else
                {
                    motor_backward(topRightIN1, topRightIN2, topRightPWM, speed);
                    motor_backward(topLeftIN1, topLeftIN2, topLeftPWM, speed);
                    motor_backward(bottomRightIN1, bottomRightIN2, bottomRightPWM, speed);
                    motor_backward(bottomLeftIN1, bottomLeftIN2, bottomLeftPWM, speed);
                }
            }
            else if (d)
            {
                
                motor_forward(topLeftIN1, topLeftIN2, topLeftPWM, speed);
                motor_backward(topRightIN1, topRightIN2, topRightPWM, speed);
                motor_forward(bottomRightIN1, bottomRightIN2, bottomRightPWM, speed);
                motor_backward(bottomLeftIN1, bottomLeftIN2, bottomLeftPWM, speed);
            }
            else if (a)
            {
             
                motor_backward(topLeftIN1, topLeftIN2, topLeftPWM, speed);
                motor_forward(topRightIN1, topRightIN2, topRightPWM, speed);
                motor_forward(bottomLeftIN1, bottomLeftIN2, bottomLeftPWM, speed);
                motor_backward(bottomRightIN1, bottomRightIN2, bottomRightPWM, speed);   
            }
            else
            {
                motor_stop(topRightIN1, topRightIN2, topRightPWM);
                motor_stop(topLeftIN1, topLeftIN2, topLeftPWM);
                motor_stop(bottomRightIN1, bottomRightIN2, bottomRightPWM);
                motor_stop(bottomLeftIN1, bottomLeftIN2, bottomLeftPWM);
            }
        }

        sleep_ms(sleep);
    }
}