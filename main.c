#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "gpio_driver.pio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "fatfs/ff.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"

#define N_BUFFERS 5
#define N_PHONES 7
#define N_CAPTURES 200
#define CAPTURE_SIZE_PER_PHONE (2048 + 1024)
#define CAPTURE_SIZE (CAPTURE_SIZE_PER_PHONE * N_PHONES)

const uint8_t counter[] = {0, 1, 2, 3, 4, 5, 6};
const uint counter_address = (uint)&counter;
uint feed_dma_chan;
uint ctrl_dma_chan;
PIO pio;
uint sm;

const uint adc_channel = 0;
const uint adc_pin = 26 + adc_channel;
uint adc_dma_chan;
struct SampleBuffer {
    uint64_t timestamp;
    ushort buffer[CAPTURE_SIZE];
};
struct SampleBuffer* sample_buffers[N_BUFFERS];

FATFS fs;
FIL fil;
FRESULT fr;     /* FatFs return code */
UINT bw;
const uint debug_pin = 15;

queue_t done_sampling_q;
queue_t done_writing_q;

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
    dma_channel_configure(feed_dma_chan, &feed_cfg, &pio0_hw->txf[sm], counter, N_PHONES, false);
}

void setup_adc() {
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
    dma_channel_configure(adc_dma_chan, &dma_cfg, &sample_buffers[0]->buffer, &adc_hw->fifo, CAPTURE_SIZE, false);
}

void setup() {
    // more involved setup
    setup_adc();
    setup_pio();
}

void capture_samples(int buffer_id) {
    dma_channel_set_write_addr(adc_dma_chan, &sample_buffers[buffer_id]->buffer, false);
    dma_start_channel_mask((1u << adc_dma_chan) | (1u << feed_dma_chan));
    sample_buffers[buffer_id]->timestamp = time_us_64();
    adc_run(true);
    dma_channel_wait_for_finish_blocking(adc_dma_chan);
    adc_run(false);
    adc_fifo_drain();
    dma_channel_set_read_addr(feed_dma_chan, counter, false);
    pio_sm_put(pio, sm, 0);
}

void core1_entry() {
    // printf("setting up core 1\n");
    // debug led
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
    // printf("core1 set up sd\n");

    for(int i = 0; i < N_BUFFERS; i++){
        queue_add_blocking(&done_writing_q, &i);
        // printf("core1 added %d\n", i);
    }

    int buffer_id;
    while(true){
        queue_remove_blocking(&done_sampling_q, &buffer_id);
        // printf("core1 received %d\n", buffer_id);
        if(buffer_id == -1){
            break;
        }
        gpio_xor_mask(1u << debug_pin);
        fr = f_write(&fil, sample_buffers[buffer_id], sizeof(struct SampleBuffer), &bw);
        gpio_xor_mask(1u << debug_pin);
        queue_add_blocking(&done_writing_q, &buffer_id);
        // printf("core1 added %d\n", buffer_id);
    }
    f_close(&fil);
    // printf("core1 finished\n");
    buffer_id = -2;
    queue_add_blocking(&done_writing_q, &buffer_id);
    return;
}

int main() {
    queue_init(&done_sampling_q, sizeof(int), N_BUFFERS + 1);
    queue_init(&done_writing_q, sizeof(int), N_BUFFERS + 1);
    stdio_init_all();
    multicore_launch_core1(core1_entry);

    for(int i = 0; i < N_BUFFERS; i++){
        sample_buffers[i] = (struct SampleBuffer*)malloc(sizeof(struct SampleBuffer));
    }
    setup();
    sleep_ms(100);
    printf("ready\n");

    char start_char = getchar();
    while(start_char != 'g'){
        sleep_ms(100);
        start_char = getchar();
    }

    int buffer_id;
    for(int i = 0; i < N_CAPTURES; i++){
        queue_remove_blocking(&done_writing_q, &buffer_id);
        // printf("core0 received %d\n", buffer_id);
        capture_samples(buffer_id);
        queue_add_blocking(&done_sampling_q, &buffer_id);
        // printf("core0 added %d\n", buffer_id);
    }
    buffer_id = -1;
    queue_add_blocking(&done_sampling_q, &buffer_id);
    while(true){
        queue_remove_blocking(&done_writing_q, &buffer_id);
        if(buffer_id == -2){
            break;
        }
    }
    multicore_reset_core1();
    for(int i = 0; i < N_BUFFERS; i++){
        free(sample_buffers[i]);
    }

    printf("transmitting\n");
    fr = f_open(&fil, "data", FA_READ);
    struct SampleBuffer buffer;
    uint br;
    for(int i = 0; i < N_CAPTURES; i++){
        f_read(&fil, &buffer, sizeof(struct SampleBuffer), &br);
        printf("sample buffer: %d\n", i);
        printf("timestamp: %lld\n", buffer.timestamp);
        for(int j = 0; j < CAPTURE_SIZE; j++){
            printf("%d\n", buffer.buffer[j]);
        }
        printf("\n\n");
    }
    printf("done");

    return 0;
}
