# 图传固件（ESP32-S3 + esp32-camera）

采集并推送 MJPEG 视频流的独立固件。相机传感器型号待实机确认（配置同时开启了
OV2640 与 OV3660 支持，见 `sdkconfig`）。

## 构建与烧录

需要 ESP-IDF（`dependencies.lock` 声明使用 IDF 5.5.3）：

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## 工具

- `set_wifi.py`：通过相机自带的 HTTP 接口写入 Wi-Fi 配置，默认 `DEFAULT_HOST`
  指向相机在热点下的地址，按实际网段修改后使用。
- `scripts/test_discovery.py`：局域网内发现相机并做连通性自检。

## 注意事项

- 本固件与底盘固件是两套独立工程，烧录目标不同，勿互相覆盖。
- 图传与底盘之间的地址、端口以主机端 `ldc_camera` / `ldc_vision` 配置为准。
