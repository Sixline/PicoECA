#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "RP2040.h"
#include "can2040/can2040.h"

#define ECS_GPIO_IN 27 // Ethanol Content Sensor GPIO Input Pin
#define UPDATE_INTERVAL 250 // In milliseconds
#define CANBUS_RX_PIN 11 //*NOT CAN HIGH OR LOW!* Goes to CANBUS transceiver!
#define CANBUS_TX_PIN 10 //*NOT CAN HIGH OR LOW!* Goes to CANBUS transceiver!
#define CANBUS_BITRATE 500000

volatile uint32_t rising_edge_time = 0;
volatile uint32_t falling_edge_time = 0;
volatile uint32_t low_time = 0;
volatile uint32_t high_time = 0;
volatile uint32_t period = 0;
volatile uint32_t last_rising_edge_time = 0;
volatile bool interrupt_occurred = false;

static struct can2040 cbus;

void gpio_callback(uint gpio, uint32_t events);
void canbus_setup(void);
static void PIOx_IRQHandler(void);
static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg);
void transmit_can_message(uint8_t ethanol_percent, uint8_t temp_c, uint8_t sensor_status);

int main() {
    // Initialize stdio and GPIO
    stdio_init_all();
    gpio_init(ECS_GPIO_IN);
    gpio_set_dir(ECS_GPIO_IN, GPIO_IN);
    gpio_pull_down(ECS_GPIO_IN);

    // Initialize Status LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    uint ledon=0;

    // Set up interrupt on ECS_GPIO_IN GPIO pin
    gpio_set_irq_enabled_with_callback(ECS_GPIO_IN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // Set up CAN bus
    canbus_setup();

    while (true) {
        if (interrupt_occurred) {
            gpio_put(PICO_DEFAULT_LED_PIN, (ledon = !ledon));
            // Output the measured low time and frequency
            if (period != 0) {
                float temp_c = (41.5 * (low_time / 1000.0f)) - 81.25;
                float temp_f = (temp_c * 1.8) + 32;
                float frequency = 1.0f / (period / 1000000.0f);
                printf("Temp C: %.2f C, Temp F: %.2f, Frequency: %.2f Hz\n", temp_c, temp_f, frequency);
                uint8_t ethanol_percent = (round(frequency)) - 50; // Frequency will be between 50 and 150 hz, subtract 50 to get Ethanol percentage.
                uint8_t temp_c_int = round(temp_c) + 40; // 8-bit integer is only 0 to 255 and can't be negative. Add 40 to support down to -40 C and offset on other end.
                uint8_t sensor_status = 1;

                if (frequency > 152 || frequency < 48) { // If the frequency is greater than 152 Hz or lower than 48 Hz, we transmit 50% Ethanol, max temp, and 1(failure) for sensor state.
                    sensor_status = 1;
                    ethanol_percent = 50;
                    temp_c_int = 165;
                    gpio_put(PICO_DEFAULT_LED_PIN, 1);
                }
                else {
                    sensor_status = 0;
                }

                transmit_can_message(ethanol_percent, temp_c_int, sensor_status);
            }
        }
        else {
            printf("ERROR! NO INTERRUPT HAS OCCURED! SENSOR FAILED OR DISCONNECTED!\n");
            transmit_can_message(50, 165, 1); // If we don't get an interrupt then the sensor has either failed or is disconnected. We transmit 50% Ethanol, max temp, and 1(failure) for sensor state.
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
        }
        interrupt_occurred = false;
        sleep_ms(UPDATE_INTERVAL);
    }
    return 0;
}

void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == ECS_GPIO_IN) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            // Capture time at rising edge
            rising_edge_time = time_us_32();
            if (last_rising_edge_time != 0) {
                period = rising_edge_time - last_rising_edge_time;
            }
            if (falling_edge_time != 0) {
                low_time = rising_edge_time - falling_edge_time;
            }
            last_rising_edge_time = rising_edge_time;
        } else if (events & GPIO_IRQ_EDGE_FALL) {
            // Capture time at falling edge
            falling_edge_time = time_us_32();
            high_time = falling_edge_time - rising_edge_time;
        }
        interrupt_occurred = true;
    }
}

void canbus_setup(void)
{
    uint32_t pio_num = 0;
    uint32_t sys_clock = 125000000, bitrate = CANBUS_BITRATE;
    uint32_t gpio_rx = CANBUS_RX_PIN, gpio_tx = CANBUS_TX_PIN;

    // Setup canbus
    can2040_setup(&cbus, pio_num);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable irqs
    irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, PIOx_IRQHandler);
    NVIC_SetPriority(PIO0_IRQ_0_IRQn, 1);
    NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);

    // Start canbus
    can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
}

static void PIOx_IRQHandler(void)
{
    can2040_pio_irq_handler(&cbus);
}

static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
}

void transmit_can_message(uint8_t ethanol_percent, uint8_t temp_c, uint8_t sensor_status)
{
    struct can2040_msg msg;
    msg.id = 0x00EC; // Zeitronix CAN Bus ID
    msg.dlc = 8;    // Data length code (number of data bytes)
    msg.data[0] = ethanol_percent;
    msg.data[1] = temp_c;
    msg.data[2] = 0x00;
    msg.data[3] = 0x00;
    msg.data[4] = 0x00;
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = sensor_status;

    can2040_transmit(&cbus, &msg);
}