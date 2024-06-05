# 本项目安装库

https://github.com/esp-arduino-libs/ESP32_JPEG/tree/feat/add_s3_block_decode

# 请使用arduino_esp32_v3版本

# ffmpeg转换代码

```
ffmpeg -i input.mp4 -c:a mp3 -c:v mjpeg -q:v 7 -vf "fps=24,scale=-1:272:flags=lanczos,crop=480:in_h:(in_w-480)/2:0" AviMp3Mjpeg272p24fps.avi
```

# 注意事项

>+ 启动PSRAM
