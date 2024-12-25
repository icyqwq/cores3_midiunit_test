// idf4.4.5
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
#include <Arduino.h>
#include <M5Unified.h>
#include <SD.h>

#include "si5351.h"
#include "midi_defines.h"
#include "es8388.h"

#define TAG "MAIN"

#define PAUSE while (1) {vTaskDelay(1000);}

HardwareSerial &MIDISerial = Serial1;
Si5351 si5351;
extern const uint8_t music_pcm_start[] asm("_binary_canon_pcm_start");
extern const uint8_t music_pcm_end[]   asm("_binary_canon_pcm_end");
LGFX_Sprite canvas(&M5.Display);

static void app_midi_write(uint8_t channel, uint8_t command, uint8_t arg1, uint8_t arg2 = 0) {

  if(command < 128) {
    // shift over command
    command <<= 4;
  }
  command |= channel;
  char buf[3] = {command, arg1, arg2};

  // send MIDI data
  MIDISerial.write(buf, 3);
}

static void app_midi_change_program(uint8_t channel, uint8_t prog) 
{
	MIDISerial.write(MIDI_CMD_PROGRAM_CHANGE + channel);
	MIDISerial.write(prog);
}

static void app_midi_change_pressure(uint8_t channel, uint8_t val)
{
	MIDISerial.write(MIDI_CMD_CONTROL_CHANGE + channel);
	MIDISerial.write(0x40);
	MIDISerial.write(val);
}

static void app_midi_test()
{
	uint8_t notes[] = {60, 62, 64, 65, 67, 69, 71, 72}; // C4, D4, E4, F4, G4, A4, B4, C5

	for (int i = 0; i < 8; i++) {
		app_midi_write(0, 0x90, notes[i], 0x7F);
		delay(100);
		app_midi_write(0, 0x80, notes[i], 0x45);
	}
}

static void wire_write_byte(uint8_t addr, uint8_t reg, uint8_t data)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

static uint8_t wire_read_byte(uint8_t addr, uint8_t reg)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(addr, 1U);
    return Wire.read();
}

static void wire_read_bytes(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(addr, len);
    Wire.readBytes(data, len);
}

static void wire_write_bytes(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(data, len);
    Wire.endTransmission();
}

void es8388_dev_init() {
    //MIDI
    // audio_hal_codec_config_t es8388_cfg = {
    //     .adc_input  = AUDIO_HAL_ADC_INPUT_LINE1,
    //     .dac_output = AUDIO_HAL_DAC_OUTPUT_LINE1,
    //     .codec_mode = AUDIO_HAL_CODEC_MODE_LINE_IN,
    //     .i2s_iface =
    //         {
    //             .mode    = AUDIO_HAL_MODE_SLAVE,
    //             .fmt     = AUDIO_HAL_I2S_NORMAL,
    //             .samples = AUDIO_HAL_44K_SAMPLES,
    //             .bits    = AUDIO_HAL_BIT_LENGTH_16BITS,
    //         },
    // };
    // es8388_init(&es8388_cfg);
    // es8388_config_i2s(es8388_cfg.codec_mode, &es8388_cfg.i2s_iface);
    // es8388_set_bits_per_sample(ES_MODULE_ADC_DAC, BIT_LENGTH_16BITS);
    // es8388_ctrl_state(AUDIO_HAL_CODEC_MODE_LINE_IN, AUDIO_HAL_CTRL_START);
    // I2S
    audio_hal_codec_config_t es8388_cfg = {
        .adc_input  = AUDIO_HAL_ADC_INPUT_LINE1,
        .dac_output = AUDIO_HAL_DAC_OUTPUT_LINE1,
        .codec_mode = AUDIO_HAL_CODEC_MODE_DECODE,
        .i2s_iface =
            {
                .mode    = AUDIO_HAL_MODE_SLAVE,
                .fmt     = AUDIO_HAL_I2S_NORMAL,
                .samples = AUDIO_HAL_44K_SAMPLES,
                .bits    = AUDIO_HAL_BIT_LENGTH_16BITS,
            },
    };
    es8388_init(&es8388_cfg);
    es8388_config_i2s(es8388_cfg.codec_mode, &es8388_cfg.i2s_iface);
    es8388_set_bits_per_sample(ES_MODULE_DAC, BIT_LENGTH_16BITS);
    es8388_ctrl_state(AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);
    es_i2s_clock_t es_i2s_clk = {
        .sclk_div = MCLK_DIV_4,
        .lclk_div = LCLK_DIV_256,
    };
    es8388_i2s_config_clock(es_i2s_clk);
    // 0xC0: -96 dB, 0x64: -50 dB, 0x00: 0 dB
    es8388_set_voice_volume(0x40);
    // es8388_write_reg(0x42, 0b01111000);
    // es8388_write_reg(0x39, 0b01111000);

    es8388_read_all();
}

uint32_t last_update_time = 0;
uint8_t last_color = 0;
void st_test() {
    uint8_t buffer[16];
    canvas.fillSprite(TFT_WHITE);
    canvas.setCursor(0, 20);

    wire_read_bytes(0x56, 0x00, buffer, 3);
    canvas.printf("  Keys: %02X, %02X, %02X\n", buffer[0], buffer[1], buffer[2]);

    wire_read_bytes(0x56, 0x10, buffer, 8);
    int32_t *counter_val = (int32_t *)buffer;
    canvas.printf("  Encoder: %d, %d\n", *counter_val, *(counter_val + 1));

    wire_read_bytes(0x56, 0x62, buffer, 1);
    canvas.printf("  HPD: %d\n", buffer[0]);

    if (millis() - last_update_time > 1000) {
        last_update_time = millis();

        memset(buffer, 0, 16);
        buffer[last_color] = 0x30;
        last_color = (last_color + 1) % 3;

        for (int i = 1; i < 5; i++ ){
            memcpy(buffer + 4 * i, buffer, 4);
        }

        for (int i = 0x70; i <= 0xe0; i += 0x10) {
            wire_write_bytes(0x56, i, buffer, 16);
        }
    }

    canvas.pushSprite(0, 0);
}

void es8388_paly_test() {
    size_t bytes_written = 0;
    ESP_LOGI(TAG, "Start to play music");
    esp_err_t ret = i2s_write(I2S_NUM_0, music_pcm_start, music_pcm_end - music_pcm_start, &bytes_written, portMAX_DELAY);
    printf("ret %d, bytes_written %d\n", ret, bytes_written);
    vTaskDelay(1);
}


static esp_err_t i2s_driver_init(void)
{
    // idf 4.x
    static const i2s_config_t i2s_cfg = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX),
        .sample_rate = 44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = true,
        .tx_desc_auto_clear = false,
    };

    if (i2s_driver_install(I2S_NUM_0, &i2s_cfg, 0, NULL) != ESP_OK) {
        printf("i2s_driver_install failed\r\n");
    }
    i2s_pin_config_t i2s_pin_cfg = {
        .mck_io_num = 7,
        .bck_io_num = 6,
        .ws_io_num = 0,
        .data_out_num = 13,
        .data_in_num = 14
    };
    if (i2s_set_pin(I2S_NUM_0, &i2s_pin_cfg) != ESP_OK) {
        printf("i2s_set_pin failed\r\n");
    }

    i2s_zero_dma_buffer(I2S_NUM_0);
    // idf 4.x
    i2s_start(I2S_NUM_0);

    printf("i2s_driver_init success\r\n");
    return ESP_OK;
}


extern "C" void app_main(void)
{
    // vTaskDelay(3000);
    esp_log_level_set("*", ESP_LOG_INFO); 
    esp_log_level_set("gpio", ESP_LOG_WARN);

    auto config = M5.config();
    config.internal_mic = false;
    config.internal_spk = false;
    M5.begin(config);
    M5.Display.fillScreen(TFT_WHITE);

    MIDISerial.begin(31250, SERIAL_8N1, -1, 5);
    if (!Wire.begin(12, 11, 100000)) {
        ESP_LOGE(TAG, "Wire begin failed");
    }

    printf("I2C Scan started\n");
    for (int addr = 0; addr < 128; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            ESP_LOGI(TAG, "Found I2C device at address 0x%02x", addr);
        }
    }

    wire_write_byte(0x56, 0x40, 0x01); // enable AP
    printf("AP: %d\n", wire_read_byte(0x56, 0x40));

    printf("music_pcm %p, %p, size %d\n", music_pcm_start, music_pcm_end, music_pcm_end - music_pcm_start);

    es8388_dev_init();

    if (!si5351.init(SI5351_CRYSTAL_LOAD_10PF, 27000000, 0)) {
        ESP_LOGE(TAG, "Si5351 not found");
    }
    si5351.set_freq(1028960000ULL, SI5351_CLK1);
    vTaskDelay(1000);
    si5351.update_status();
    printf("SYS_INIT %d, LOL_A %d, LOL_B %d, LOS %d, REVID %d\n", si5351.dev_status.SYS_INIT, si5351.dev_status.LOL_A, si5351.dev_status.LOL_B, si5351.dev_status.LOS, si5351.dev_status.REVID);

    i2s_driver_init();

    canvas.setPsram(true);
    canvas.setColorDepth(lgfx::rgb565_2Byte);
    canvas.createSprite(320, 240);
    canvas.fillSprite(0xFFFFFF);
    canvas.pushSprite(0, 0);

    canvas.setTextSize(2);
    canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    M5.Display.setRotation(0);

    xTaskCreatePinnedToCore([](void *arg) {
        while (1)
        {
            es8388_paly_test();
            // vTaskDelay(5000);
        }
    }, "i2s", 1024 * 4, NULL, 5, NULL, 1);

    // xTaskCreatePinnedToCore([](void *arg) {
    //     while (1)
    //     {
    //         app_midi_test();
    //         vTaskDelay(5000);
    //     }
    // }, "midi", 1024 * 4, NULL, 5, NULL, 1);

    while (1)
    {
        st_test();
        vTaskDelay(50);
    }
    
    



    PAUSE
}
