#include "sam3xa/include/sam3xa.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "sam3xa/include/component/uart.h"     // General header for SAM microcontrollers
#include "sam3xa/include/component/pio.h"      // Header for PIO definitions
#include "sam3xa/include/component/pmc.h"      // Header for Power Management Controller definitions
#include "sam3xa/include/component/adc.h"      // Header for ADC definitions
#include "sam3xa/include/component/wdt.h" 
#include "sam.h"     // Header for Watchdog Timer definitions
// Header for UART definitions


#define TEMP_PIN 0 // ADC channel 0 (A10)
#define LIGHT_PIN 4 // ADC channel 4 (A4)
#define MOISTURE_PIN 5 // ADC channel 5 (A5)
#define LED_PIN PIO_PC23 // Pin 23
#define FAN_PIN PIO_PC12 // Pin 51
#define MOTOR_PIN PIO_PC15 // Pin 37

#define LIGHT_THRESHOLD 500
#define TEMP_THRESHOLD 20
#define MOISTURE_THRESHOLD 300

#define RS_PIN PIO_PA7
#define EN_PIN PIO_PA8
#define D4_PIN PIO_PA9
#define D5_PIN PIO_PA10
#define D6_PIN PIO_PA11
#define D7_PIN PIO_PA12

// Function prototypes
void adc_init(void);
uint32_t adc_read(uint8_t channel);
void gpio_init(void);
void gpio_set_output(Pio *pio, uint32_t mask);
void gpio_write(Pio *pio, uint32_t mask, uint8_t value);
void uart_init(void);
void uart_send_string(const char *str);
void uart_send_float(float value);
void delay_ms(uint32_t ms);
void lcd_init(void);
void lcd_clear(void);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_print(const char *str);
void lcd_print_float(float value);
void lcd_write_nibble(uint8_t nibble);
void lcd_send_command(uint8_t command);
void lcd_send_data(uint8_t data);

int main(void) {
    // Disable watchdog timer
    WDT->WDT_MR = WDT_MR_WDDIS;

    // Initialize peripherals
    adc_init();
    gpio_init();
    uart_init();
    gpio_set_output(PIOC, LED_PIN | FAN_PIN | MOTOR_PIN);

    // Initialize LCD
    lcd_init();
    lcd_clear();
    lcd_print("Temp: ");

    while (1) {
        // Read sensor values
        uint32_t tempReading = adc_read(TEMP_PIN);
        uint32_t lightReading = adc_read(LIGHT_PIN);
        uint32_t moistureReading = adc_read(MOISTURE_PIN);

        // Convert temperature reading
        float voltage = tempReading * 3.3 / 4095.0;
        float temperatureC = (voltage - 0.5) * 100;

        // Serial output
        uart_send_float(temperatureC);
        uart_send_string(",");
        uart_send_float(lightReading);
        uart_send_string(",");
        uart_send_float(moistureReading);
        uart_send_string(",");

        // Control actuators
        bool ledStatus = lightReading < LIGHT_THRESHOLD;
        bool fanStatus = temperatureC > TEMP_THRESHOLD;
        bool motorStatus = moistureReading < MOISTURE_THRESHOLD;

        gpio_write(PIOC, LED_PIN, ledStatus ? 1 : 0);
        gpio_write(PIOC, FAN_PIN, fanStatus ? 1 : 0);
        gpio_write(PIOC, MOTOR_PIN, motorStatus ? 1 : 0);

        // Send actuator status to NodeMCU
        uart_send_string(ledStatus ? "ON" : "OFF");
        uart_send_string(",");
        uart_send_string(fanStatus ? "ON" : "OFF");
        uart_send_string(",");
        uart_send_string(motorStatus ? "ON" : "OFF");
        uart_send_string("\n");

        // Display on LCD
        lcd_set_cursor(0, 0);
        lcd_print("Temp: ");
        lcd_print_float(temperatureC);
        lcd_print(" C");
        lcd_set_cursor(0, 1);
        lcd_print("MOISTURE: ");
        lcd_print_float(moistureReading);

        delay_ms(1000);
    }

    return 0;
}

void adc_init(void) {
    // Enable ADC peripheral clock
    PMC->PMC_PCER1 |= PMC_PCER1_PID37;

    // Reset and configure the ADC
    ADC->ADC_CR = ADC_CR_SWRST;
    ADC->ADC_MR = ADC_MR_TRGEN_DIS | ADC_MR_TRGSEL_ADC_TRIG1 | ADC_MR_PRESCAL(15) |
                  ADC_MR_STARTUP_SUT0 | ADC_MR_TRACKTIM(2) | ADC_MR_TRANSFER(1);
    ADC->ADC_CHER = ADC_CHER_CH0 | ADC_CHER_CH4 | ADC_CHER_CH5; // Enable channels 0, 4, 5
}

uint32_t adc_read(uint8_t channel) {
    // Start conversion
    ADC->ADC_CR = ADC_CR_START;

    // Wait for conversion to complete
    while (!(ADC->ADC_ISR & (1 << channel)));

    // Read and return the value
    return ADC->ADC_CDR[channel] & ADC_CDR_DATA_Msk;
}

void gpio_init(void) {
    // Enable PIO peripheral clocks
    PMC->PMC_PCER0 |= PMC_PCER0_PID11; // Enable PIOA clock
    PMC->PMC_PCER0 |= PMC_PCER0_PID12; // Enable PIOB clock
    PMC->PMC_PCER0 |= PMC_PCER0_PID13; // Enable PIOC clock
    PMC->PMC_PCER0 |= PMC_PCER0_PID14; // Enable PIOD clock
}

void gpio_set_output(Pio *pio, uint32_t mask) {
    pio->PIO_PER = mask; // Enable PIO control
    pio->PIO_OER = mask; // Enable output
    pio->PIO_OWER = mask; // Enable write
}

void gpio_write(Pio *pio, uint32_t mask, uint8_t value) {
    if (value) {
        pio->PIO_SODR = mask; // Set the pin
    } else {
        pio->PIO_CODR = mask; // Clear the pin
    }
}

void uart_init(void) {
    // Enable UART0 peripheral clock
    PMC->PMC_PCER0 |= PMC_PCER0_PID8;

    // Configure UART0
    UART0->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS;
    UART0->UART_MR = UART_MR_PAR_NO | UART_MR_CHMODE_NORMAL;
    UART0->UART_BRGR = 45; // Baud rate 9600 assuming 84 MHz clock
    UART0->UART_CR = UART_CR_TXEN | UART_CR_RXEN;
}

void uart_send_string(const char *str) {
    while (*str) {
        while (!(UART0->UART_SR & UART_SR_TXRDY));
        UART0->UART_THR = *str++;
    }
}

void uart_send_float(float value) {
    char buffer[10];
    snprintf(buffer, sizeof(buffer), "%.2f", value);
    uart_send_string(buffer);
}

void delay_ms(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms * 2100; i++);
}

void lcd_init(void) {
    gpio_set_output(PIOA, RS_PIN | EN_PIN | D4_PIN | D5_PIN | D6_PIN | D7_PIN);
    delay_ms(20);

    lcd_write_nibble(0x03);
    delay_ms(5);
    lcd_write_nibble(0x03);
    delay_ms(5);
    lcd_write_nibble(0x03);
    delay_ms(1);
    lcd_write_nibble(0x02); // 4-bit mode
    lcd_send_command(0x28); // Function set
    lcd_send_command(0x0C); // Display on, cursor off
    lcd_send_command(0x06); // Entry mode set
    lcd_send_command(0x01); // Clear display
    delay_ms(2);
}

void lcd_clear(void) {
    lcd_send_command(0x01); // Clear display
    delay_ms(2);
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    lcd_send_command(0x80 | (col + row_offsets[row]));
}

void lcd_print(const char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

void lcd_print_float(float value) {
    char buffer[10];
    snprintf(buffer, sizeof(buffer), "%.2f", value);
    lcd_print(buffer);
}

void lcd_write_nibble(uint8_t nibble) {
    gpio_write(PIOA, D4_PIN, (nibble >> 0) & 0x01);
    gpio_write(PIOA, D5_PIN, (nibble >> 1) & 0x01);
    gpio_write(PIOA, D6_PIN, (nibble >> 2) & 0x01);
    gpio_write(PIOA, D7_PIN, (nibble >> 3) & 0x01);

    gpio_write(PIOA, EN_PIN, 1);
    delay_ms(1);
    gpio_write(PIOA, EN_PIN, 0);
    delay_ms(1);
}

void lcd_send_command(uint8_t command) {
    gpio_write(PIOA, RS_PIN, 0);
    lcd_write_nibble(command >> 4);
    lcd_write_nibble(command & 0x0F);
}

void lcd_send_data(uint8_t data) {
    gpio_write(PIOA, RS_PIN, 1);
    lcd_write_nibble(data >> 4);
    lcd_write_nibble(data & 0x0F);
}
