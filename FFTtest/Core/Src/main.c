#include "arm_math.h"
#include <stdio.h>

#define FFT_SIZE 256

q15_t input[FFT_SIZE * 2]; // Interleaved real and imaginary parts
q15_t output[FFT_SIZE * 2];
arm_rfft_instance_q15 S;

int main() {
    // Initialize the RFFT instance
    arm_rfft_init_q15(&S, FFT_SIZE, 0, 1); // Forward FFT, bit-reversal enabled

    // Fill input with sample data (e.g., a sine wave)
    for (int i = 0; i < FFT_SIZE; i++) {
        input[2 * i] = (q15_t)(32767.0 * sin(2 * 3.14159265 * i / FFT_SIZE));
        input[2 * i + 1] = 0; // Imaginary part is zero
    }

    // Perform FFT
    arm_rfft_q15(&S, input, output);


    return 0;
}
