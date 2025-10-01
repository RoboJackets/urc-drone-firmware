#include <Wire.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>

// ---------------- Motor Control Vars ----------------
volatile int16_t mech_angle = 0;   // desired mechanical angle (deg)
uint8_t pole_pairs = 5;

// ---------------- PWM Init ----------------
void pwm_init() {
    DDRB |= (1 << PB1) | (1 << PB2) | (1 << PB3);  // OC1A, OC1B, OC2A

    // Timer1 setup
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS10);

    // Timer2 setup
    TCCR2A = (1 << COM2A1) | (1 << WGM20) | (1 << WGM21);
    TCCR2B = (1 << CS20);
}

// ---------------- PWM Write ----------------
void set_pwm(uint8_t a, uint8_t b, uint8_t c) {
    OCR1A = a;  // phase A
    OCR1B = b;  // phase B
    OCR2A = c;  // phase C
}

// ---------------- Motor Drive ----------------
void drive_motor(int16_t mech_angle) {
    float elec_angle = mech_angle * pole_pairs;

    float a = elec_angle * M_PI / 180.0;
    float b = (elec_angle + 120) * M_PI / 180.0;
    float c = (elec_angle + 240) * M_PI / 180.0;

    uint8_t pwmA = (uint8_t)(sin(a) * 127.5 + 127.5);
    uint8_t pwmB = (uint8_t)(sin(b) * 127.5 + 127.5);
    uint8_t pwmC = (uint8_t)(sin(c) * 127.5 + 127.5);

    set_pwm(pwmA * 0.7, pwmB * 0.7, pwmC * 0.7);
}

// ---------------- I²C Receive Handler ----------------
void receiveEvent(int howMany) {
    if (howMany >= 2) {
        int16_t angle_raw = Wire.read();         // low byte
        angle_raw |= (Wire.read() << 8);         // high byte
        mech_angle = angle_raw % 360;            // wrap into [0,360)
    }
}

// ---------------- Main ----------------
int main(void) {
    pwm_init();

    // I²C init (ATMega as slave at address 0x08)
    Wire.begin(0x08);
    Wire.onReceive(receiveEvent);

    while (1) {
        drive_motor(mech_angle);
        _delay_ms(2);  // slow down loop slightly
    }
}
