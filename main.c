#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "gpio_driver.pio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "fatfs/ff.h"

const uint n_phones = 7;
const uint8_t counter[] = {0, 1, 2, 3, 4, 5, 6};
const uint counter_address = (uint)&counter;
uint feed_dma_chan;
uint ctrl_dma_chan;
PIO pio;
uint sm;

void setup_pio() {
    // Choose which PIO instance to use (there are two instances)
    pio = pio0;
    uint offset = pio_add_program(pio, &gpio_driver_program);
    sm = pio_claim_unused_sm(pio, true);
    gpio_driver_program_init(pio, sm, offset, 16);

    feed_dma_chan = dma_claim_unused_channel(true);
    ctrl_dma_chan = dma_claim_unused_channel(true);

    dma_channel_config ctrl_cfg = dma_channel_get_default_config(ctrl_dma_chan);
    channel_config_set_transfer_data_size(&ctrl_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&ctrl_cfg, false);
    channel_config_set_write_increment(&ctrl_cfg, false);
    dma_channel_configure(ctrl_dma_chan, &ctrl_cfg, &dma_hw->ch[feed_dma_chan].al3_read_addr_trig, &counter_address, 1, false);

    dma_channel_config feed_cfg = dma_channel_get_default_config(feed_dma_chan);
    channel_config_set_transfer_data_size(&feed_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&feed_cfg, true);
    channel_config_set_write_increment(&feed_cfg, false);
    channel_config_set_dreq(&feed_cfg, DREQ_ADC);
    channel_config_set_chain_to(&feed_cfg, ctrl_dma_chan);
    dma_channel_configure(feed_dma_chan, &feed_cfg, &pio0_hw->txf[sm], counter, n_phones, false);
}

const uint adc_channel = 0;
const uint adc_pin = 26 + adc_channel;
const uint capture_size = 128 * n_phones;
ushort *sample_buffer;
uint adc_dma_chan;
const uint capture_bytes = capture_size * sizeof(ushort);

void setup_adc() {
    sample_buffer = malloc(capture_bytes);

    adc_gpio_init(adc_pin);
    adc_init();
    adc_select_input(adc_channel);
    adc_fifo_setup(true, true, 1, false, false);
    adc_set_clkdiv(0);

    adc_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config dma_cfg = dma_channel_get_default_config(adc_dma_chan);
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_cfg, false);
    channel_config_set_write_increment(&dma_cfg, true);
    channel_config_set_dreq(&dma_cfg, DREQ_ADC);
    dma_channel_configure(adc_dma_chan, &dma_cfg, sample_buffer, &adc_hw->fifo, capture_size, false);
}

FATFS fs;
FIL fil;
FRESULT fr;     /* FatFs return code */
UINT bw;
const uint led_pin = 25;
const uint debug_pin = 15;

void setup() {
    // set up gpio
    gpio_init(led_pin);
    gpio_set_dir(led_pin, true);
    gpio_init(debug_pin);
    gpio_set_dir(debug_pin, true);

    // set up sd card
    fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) {
        return;
    }

    fr = f_open(&fil, "data", FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        return;
    }

    // more involved setup
    setup_adc();
    setup_pio();
}

void capture_samples() {
    dma_start_channel_mask((1u << adc_dma_chan) | (1u << feed_dma_chan));
    adc_run(true);
    dma_channel_wait_for_finish_blocking(adc_dma_chan);
    adc_run(false);
    adc_fifo_drain();
    dma_channel_set_read_addr(feed_dma_chan, counter, false);
    dma_channel_set_write_addr(adc_dma_chan, sample_buffer, false);
    pio_sm_put(pio, sm, 0);
}

const uint n_captures = 1000;

int main() {
    // stdio_init_all();
    // printf("Setting Up\n");
    setup();

    for(int i = 0; i < n_captures; i++){
        // printf("Capture %d\n", i);
        capture_samples();
        gpio_xor_mask((1u << led_pin) | (1u << debug_pin));
        fr = f_write(&fil, sample_buffer, capture_bytes, &bw);
        gpio_xor_mask((1u << led_pin) | (1u << debug_pin));
    }
    f_close(&fil);
    // printf("finished\n");

    return 0;
}
