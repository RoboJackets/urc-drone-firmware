#include <avr/io.h>
#include <util/delay.h>

// Pin mapping on ATmega (adjust if needed)
// Let's assume:
//   enable = PB0
//   phaseA = PB1
//   phaseB = PB2
//   phaseC = PB3
#define ENABLE_PIN PB0
#define PHASEA_PIN PB1
#define PHASEB_PIN PB2
#define PHASEC_PIN PB3

// Commutation table: each row = {A,B,C}
const uint8_t steps[6][3] = {
    {1, 0, 0}, // A high
    {1, 1, 0}, // A+B
    {0, 1, 0}, // B high
    {0, 1, 1}, // B+C
    {0, 0, 1}, // C high
    {1, 0, 1}  // A+C
};

int main(void) {
    // Set pins as outputs
    DDRB |= (1 << ENABLE_PIN) | (1 << PHASEA_PIN) | (1 << PHASEB_PIN) | (1 << PHASEC_PIN);

    // Enable driver
    PORTB |= (1 << ENABLE_PIN);

    uint8_t stepIndex = 0;

    while (1) {
        // Clear phase pins first
        PORTB &= ~((1 << PHASEA_PIN) | (1 << PHASEB_PIN) | (1 << PHASEC_PIN));

        // Apply step pattern
        if (steps[stepIndex][0]) PORTB |= (1 << PHASEA_PIN);
        if (steps[stepIndex][1]) PORTB |= (1 << PHASEB_PIN);
        if (steps[stepIndex][2]) PORTB |= (1 << PHASEC_PIN);

        // Next step
        stepIndex++;
        if (stepIndex >= 6) stepIndex = 0;

        _delay_ms(5); // adjust speed
    }
}
