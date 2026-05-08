# AdminsWifiUSBKeyboard
Virtual BIOS compatible USB Keyboard over wifi for admins 

# Build
install esp-idf

## Debug Test only with smaller CPU, without USB
idf.py set-target  esp32c3

## Build with USB 
idf.py add-dependency "espressif/esp_tinyusb"
idf.py set-target  esp32s3

