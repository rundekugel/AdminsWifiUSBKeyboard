# AdminsWifiUSBKeyboard
Virtual BIOS compatible USB Keyboard over wifi for admins 

# Build
Needs to install ESP-IDF <br>
then: <br>
<br>
 source <path-to-esp-idf>/export.sh<br>
 idf.py build<br>
 idf.py flash
 
## Debug Test only with smaller CPU, without USB
idf.py set-target  esp32c3

## Build with USB 
idf.py add-dependency "espressif/esp_tinyusb"<br>
idf.py set-target  esp32s3

# Details
[Help-file](doc/admin_keyboard_help.md)
