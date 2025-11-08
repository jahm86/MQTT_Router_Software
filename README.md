# MQTT Router Server Software

A software tool used for communication between diverse protocols (Modbus, I/O, etc.) and MQTT clients. These protocols can be configured dynamically, without recompiling, as long as their drivers are included in the firmware.

If you get trouble compiling, plaese refer to [this document](HowModifiedESPAsyncWebServer.md).

### Loading web server files

The first time you load the firmware, and every time you make changes to web server contents into path **./data**, you must load the **SPIFFS** with these contents. There are 3 ways to do this:

1. With Platformio in VSCode: Click the PIO icon at the left side bar. Go to the project task you are working. Then expand "Platform" and press "Build Filesystem Image", then "Upload Filesystem Image". You can also find "Build Filesystem Image" and "Upload Filesystem Image" commands with the command pallete (Ctrl+Shift+P). More details in [this link](https://randomnerdtutorials.com/esp32-vs-code-platformio-spiffs/).

2. By command line, by using "mkspiffs" and "esptool.py":

```bash
# Buld image from files in "./data" directory
mkspiffs -c data -b 4096 -p 256 -s 0x300000 spiffs_image.bin
# Delete flash content in ESP32
esptool.py --chip esp32 --port <serial_port> erase_flash
# Upload image to ESP32
esptool.py --chip esp32 --port <serial_port> --baud 921600 write_flash 0x100000 spiffs_image.bin
```

3. With Arduino IDE: watch this video tutorial:

[![Watch the video](https://img.youtube.com/vi/9i1nDUoDRcI/default.jpg)](https://youtu.be/9i1nDUoDRcI)
