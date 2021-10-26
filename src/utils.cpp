#include "utils.h"

#ifdef GD32

extern "C" {

#include "config.h"
#include "gd32vf103.h"
#include "systick.h"

/* retarget the C library printf function to the USART 
 * TODO build up a buffer and send via DMA
*/
int _put_char(int ch)
{
    usart_data_transmit(USART0, (uint8_t) ch );
    while ( usart_flag_get(USART0, USART_FLAG_TBE)== RESET){
    }

    return ch;
}

} // extern C

// ==========================================================
// C++ functions

// get_timer_value() runs at main clock / 4, so with 108MHz clock,
// it gives 27MHz ticks
#define TIMER_TO_MILLIS (27000)
#define TIMER_TO_MICROS (27)

unsigned long millis(void){
    return (unsigned long)(get_timer_value() / TIMER_TO_MILLIS);
}

unsigned long micros(void){
    return (unsigned long)(get_timer_value() / TIMER_TO_MICROS);
}

/** Send 8 bit data over SPI and read 8 bit return value
 *  TODO Not currently used - remove?
 */
// uint8_t spi1_transfer(uint8_t x)
// {
//     gpio_bit_reset(RADIO_NSS_PORT, RADIO_NSS_PIN); // set NSS low

//     while (0 == spi_i2s_flag_get(SPI1, SPI_FLAG_TBE))
//         ; // wait for any previous tx to complete

//     spi_i2s_data_transmit(SPI1, x);

//     while (0 == spi_i2s_flag_get(SPI1, SPI_FLAG_RBNE))
//         ; // wait for rx data to be ready

//     uint16_t r = spi_i2s_data_receive(SPI1);

//     gpio_bit_set(RADIO_NSS_PORT, RADIO_NSS_PIN); // set NSS high

//     return (uint8_t)r;
// }

/**
 * Transfer multiple bytes over SPI
 * nBytes of data from buffer will be sent, with return values overwriting
 * the original data.
 * 
 *  TODO replace with DMA
 * 
 *  XXX TODO Move to it's own SPI abstraction layer
 * 
 * This is the critical path for communicating with the radio
 * 
 */
// void spi1_transferBytes(uint8_t *buffer, const int nBytes)
// {
//     gpio_bit_reset(RADIO_NSS_PORT, RADIO_NSS_PIN); // set NSS low

//     for(int i=0; i<nBytes; i++) {
//         while (0 == spi_i2s_flag_get(SPI1, SPI_FLAG_TBE))
//             ; // wait for any previous tx to complete

//         spi_i2s_data_transmit(SPI1, buffer[i]);

//         while (0 == spi_i2s_flag_get(SPI1, SPI_FLAG_RBNE))
//             ; // wait for rx data to be ready

//         buffer[i] = spi_i2s_data_receive(SPI1);
//     }

//     gpio_bit_set(RADIO_NSS_PORT, RADIO_NSS_PIN); // set NSS high
// }

#endif // GD32

#ifdef ESPC3

#include <sys/time.h>


unsigned long micros()
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;

    return time_us;
}

unsigned long millis()
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t time_ms = (int64_t)tv_now.tv_sec * 1000L + ((int64_t)tv_now.tv_usec / 1000);

    return time_ms;
}


#endif // ESPC3

// common code for all platforms

uint8_t CalcCRC(volatile uint8_t *data, int length)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        crc = crc8tab[crc ^ *data++];
    }
    return crc;
}

unsigned long seed = 0;

// returns values between 0 and 0x7FFF
// NB rngN depends on this output range, so if we change the
// behaviour rngN will need updating
long rng(void)
{
    long m = 2147483648;
    long a = 214013;
    long c = 2531011;
    seed = (a * seed + c) % m;
    return seed >> 16;
}

void rngSeed(long newSeed)
{
    seed = newSeed;
}

// returns 0 <= x < max where max <= 256
// (actual upper limit is higher, but there is one and I haven't
//  thought carefully about what it is)
unsigned int rngN(unsigned int max)
{
    unsigned long x = rng();
    unsigned int result = (x * max) / RNG_MAX; // XXX this is wrong and needs fixing, but all rxes and txes will need to be done at the same time
    return result;
}

