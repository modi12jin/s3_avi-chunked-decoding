#ifndef _JPEG_DEC_H_
#define _JPEG_DEC_H_

#include <ESP32_JPEG_Library.h>
#include "FS.h"

size_t getFileSize(fs::FS &fs, const char *path) {
  File file = fs.open(path);
  if (!file) {
    Serial.println("- failed to open file for getting size");
    return 0;
  }

  size_t len = file.size();
  file.close();
  return len;
}

void readFile(fs::FS &fs, const char *path, uint8_t *buf, size_t buf_len) {
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    return;
  }

  size_t len = file.size();
  len = len > buf_len ? buf_len : len;
  while (len) {
    size_t read_len = len > 512 ? 512 : len;
    read_len = file.read(buf, read_len);
    buf += read_len;
    len -= read_len;
  }

  file.close();

  Serial.println("Read file done");
}

bool esp_jpeg_decoder_one_picture_block_out(unsigned char *in_buf, int in_len, int (*jpegDrawCallback)(jpeg_dec_io_t *jpeg_io, jpeg_dec_header_info_t *out_info)) {
  unsigned char *output_block = NULL;
  int output_len = 0;
  jpeg_dec_io_t *jpeg_io = NULL;
  jpeg_dec_header_info_t *out_info = NULL;
  jpeg_dec_handle_t *jpeg_dec = NULL;

  // Generate configuration
  jpeg_dec_config_t config = DEFAULT_JPEG_DEC_CONFIG();
  config.block_enable = 1;

  // Create jpeg_dec
  jpeg_dec = jpeg_dec_open(&config);

  // Create io_callback handle
  jpeg_io = (jpeg_dec_io_t *)calloc(1, sizeof(jpeg_dec_io_t));

  // Create out_info handle
  out_info = (jpeg_dec_header_info_t *)calloc(1, sizeof(jpeg_dec_header_info_t));

  // Set input buffer and buffer len to io_callback
  jpeg_io->inbuf = in_buf;
  jpeg_io->inbuf_len = in_len;

  // Parse jpeg picture header and get picture for user and decoder
  jpeg_dec_parse_header(jpeg_dec, jpeg_io, out_info);

  output_len = out_info->width * (out_info->y_factory[0] << 3) * 2;
  // Malloc output block buffer
  output_block = (unsigned char *)jpeg_malloc_align(output_len, 16);

  jpeg_io->outbuf = output_block;

  while (jpeg_io->output_line < jpeg_io->output_height) {
    jpeg_dec_process(jpeg_dec, jpeg_io);
    jpegDrawCallback(jpeg_io, out_info);
  }

  jpeg_dec_close(jpeg_dec);
  free(jpeg_io);
  free(out_info);
  jpeg_free_align(output_block);
  return true;
}

#endif  // _JPEG_DEC_H_