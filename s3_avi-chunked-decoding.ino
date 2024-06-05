#pragma GCC push_options
#pragma GCC optimize("O3")

/***
 * Required libraries:
 * avilib: https://github.com/lanyou1900/avilib.git
 * libhelix: https://github.com/pschatzmann/arduino-libhelix.git
 * ESP32_JPEG: https://github.com/esp-arduino-libs/ESP32_JPEG/tree/feat/add_s3_block_decode
 */

const char *root = "/root";
const char *avi_file = "/root/AviMp3Mjpeg272p24fps.avi";

#include <WiFi.h>

#include <FFat.h>
#include <LittleFS.h>
#include <SD_MMC.h>

extern "C" {
#include <avilib.h>
}
#include "pins_config.h"
#include "src/lcd/nv3401a_lcd.h"

nv3401a_lcd lcd = nv3401a_lcd(TFT_QSPI_CS, TFT_QSPI_SCK, TFT_QSPI_D0, TFT_QSPI_D1, TFT_QSPI_D2, TFT_QSPI_D3, TFT_QSPI_RST);

/* variables */
static avi_t *a;
static long frames, estimateBufferSize, aRate, aBytes, aChunks, actual_video_size;
static long w, h, aChans, aBits, aFormat;
static double fr;
static char *compressor;
static char *vidbuf;
static char *audbuf;
static size_t audbuf_read;
static size_t audbuf_remain = 0;
static bool isStopped = true;
static long curr_frame = 0;
static long skipped_frames = 0;
static unsigned long start_ms, curr_ms, next_frame_ms;
static unsigned long total_read_video_ms = 0;
static unsigned long total_decode_video_ms = 0;
static unsigned long total_show_video_ms = 0;

#include "esp32_audio.h"

#include "jpeg_dec.h"
//jpeg绘制回调
static int jpegDrawCallback(jpeg_dec_io_t *jpeg_io, jpeg_dec_header_info_t *out_info) {
  lcd.draw16bitbergbbitmap(0, jpeg_io->output_line - jpeg_io->cur_line, out_info->width, jpeg_io->cur_line, (uint16_t *)jpeg_io->outbuf);
  return 1;
}

void setup() {
  WiFi.mode(WIFI_OFF);

  Serial.begin(115200);
  Serial.println("s3_avi-chunked-decoding");

  // Init Display
  lcd.begin();

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  pinMode(SDMMC_CS /* CS */, OUTPUT);
  digitalWrite(SDMMC_CS /* CS */, HIGH);
  SD_MMC.setPins(SDMMC_CLK /* CLK */, SDMMC_CMD /* CMD/MOSI */, SDMMC_D0 /* D0/MISO */);
  if (!SD_MMC.begin("/root", true)) /* 1-bit SD bus mode */
  {
    Serial.println("ERROR: File system mount failed!");
  } else {
    a = AVI_open_input_file(avi_file, 1);

    if (a) {
      frames = AVI_video_frames(a);
      w = AVI_video_width(a);
      h = AVI_video_height(a);
      fr = AVI_frame_rate(a);
      compressor = AVI_video_compressor(a);
      estimateBufferSize = w * h * 2 / 7;
      Serial.printf("AVI frames: %ld, %ld x %ld @ %.2f fps, format: %s, estimateBufferSize: %ld, ESP.getFreeHeap(): %ld\n", frames, w, h, fr, compressor, estimateBufferSize, (long)ESP.getFreeHeap());

      aChans = AVI_audio_channels(a);
      aBits = AVI_audio_bits(a);
      aFormat = AVI_audio_format(a);
      aRate = AVI_audio_rate(a);
      aBytes = AVI_audio_bytes(a);
      aChunks = AVI_audio_chunks(a);
      Serial.printf("Audio channels: %ld, bits: %ld, format: %ld, rate: %ld, bytes: %ld, chunks: %ld\n", aChans, aBits, aFormat, aRate, aBytes, aChunks);

      vidbuf = (char *)heap_caps_malloc(estimateBufferSize, MALLOC_CAP_8BIT);
      audbuf = (char *)heap_caps_malloc(MP3_MAX_FRAME_SIZE, MALLOC_CAP_8BIT);

      i2s_init(I2S_NUM_0,
               aRate /* sample_rate */,
               -1 /* mck_io_num */, /*!< MCK in out pin. Note that ESP32 supports setting MCK on GPIO0/GPIO1/GPIO3 only*/
               I2S_BCLK,            /*!< BCK in out pin*/
               I2S_LRC,             /*!< WS in out pin*/
               I2S_DOUT,            /*!< DATA out pin*/
               -1 /* data_in_num */ /*!< DATA in pin*/
      );
      i2s_zero_dma_buffer(I2S_NUM_0);

      isStopped = false;
      start_ms = millis();
      next_frame_ms = start_ms + ((curr_frame + 1) * 1000 / fr);

      Serial.println("Play AVI start");
      curr_ms = millis();
      start_ms = curr_ms;

      audbuf_read = AVI_read_audio(a, audbuf, MP3_MAX_FRAME_SIZE);
      audbuf_remain = audbuf_read;
      total_read_audio_ms += millis() - curr_ms;
      curr_ms = millis();

      Serial.println("Start play audio task");
      BaseType_t ret_val = mp3_player_task_start(a);
      if (ret_val != pdPASS) {
        Serial.printf("mp3_player_task_start failed: %d\n", ret_val);
      }
    }
  }
}

void loop() {
  if (!isStopped) {
    if (curr_frame < frames) {
      if (audbuf_remain == 0) {
        audbuf_read = AVI_read_audio(a, audbuf, MP3_MAX_FRAME_SIZE);
        audbuf_remain = audbuf_read;
        total_read_audio_ms += millis() - curr_ms;
        curr_ms = millis();
      }

      if (millis() < next_frame_ms)  // check show frame or skip frame
      {
        AVI_set_video_position(a, curr_frame);

        int iskeyframe;
        long video_bytes = AVI_frame_size(a, curr_frame);
        if (video_bytes > estimateBufferSize) {
          Serial.printf("video_bytes(%ld) > estimateBufferSize(%ld)\n", video_bytes, estimateBufferSize);
        } else {
          actual_video_size = AVI_read_frame(a, vidbuf, &iskeyframe);
          total_read_video_ms += millis() - curr_ms;
          curr_ms = millis();
          // Serial.printf("frame: %ld, iskeyframe: %ld, video_bytes: %ld, actual_video_size: %ld, audio_bytes: %ld, ESP.getFreeHeap(): %ld\n", curr_frame, iskeyframe, video_bytes, actual_video_size, audio_bytes, (long)ESP.getFreeHeap());

          // Set input buffer and buffer len to io_callback
          total_decode_video_ms += millis() - curr_ms;
          curr_ms = millis();

          esp_jpeg_decoder_one_picture_block_out((unsigned char *)vidbuf, actual_video_size, jpegDrawCallback);

          total_show_video_ms += millis() - curr_ms;
          curr_ms = millis();
        }
        while (millis() < next_frame_ms) {
          vTaskDelay(pdMS_TO_TICKS(1));
        }
      } else {
        ++skipped_frames;
        // Serial.printf("Skip frame %ld > %ld\n", millis(), next_frame_ms);
      }

      ++curr_frame;
      curr_ms = millis();
      next_frame_ms = start_ms + ((curr_frame + 1) * 1000 / fr);
    } else {
      int time_used = millis() - start_ms;
      int total_frames = curr_frame;
      audbuf_read = 0;
      AVI_close(a);
      isStopped = true;
      Serial.println("Play AVI end");

      long played_frames = total_frames - skipped_frames;
      float fps = 1000.0 * played_frames / time_used;
      total_decode_audio_ms -= total_play_audio_ms;
      //total_decode_video_ms -= total_show_video_ms;
      Serial.printf("Played frames: %ld\n", played_frames);
      Serial.printf("Skipped frames: %ld (%0.1f %%)\n", skipped_frames, 100.0 * skipped_frames / total_frames);
      Serial.printf("Time used: %lu ms\n", time_used);
      Serial.printf("Expected FPS: %0.1f\n", fr);
      Serial.printf("Actual FPS: %0.1f\n", fps);
      Serial.printf("Read audio: %lu ms (%0.1f %%)\n", total_read_audio_ms, 100.0 * total_read_audio_ms / time_used);
      Serial.printf("Decode audio: %lu ms (%0.1f %%)\n", total_decode_audio_ms, 100.0 * total_decode_audio_ms / time_used);
      Serial.printf("Play audio: %lu ms (%0.1f %%)\n", total_play_audio_ms, 100.0 * total_play_audio_ms / time_used);
      Serial.printf("Read video: %lu ms (%0.1f %%)\n", total_read_video_ms, 100.0 * total_read_video_ms / time_used);
      Serial.printf("Decode video: %lu ms (%0.1f %%)\n", total_decode_video_ms, 100.0 * total_decode_video_ms / time_used);
      Serial.printf("Show video: %lu ms (%0.1f %%)\n", total_show_video_ms, 100.0 * total_show_video_ms / time_used);
    }
  } else {
    delay(100);
  }
}