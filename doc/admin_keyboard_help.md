# Admin-Keyboard – Help V0.5.1

Admin-Keyboard is an ESP32-S3 device that acts as a **USB HID keyboard** and is configured over a **Wi‑Fi web interface**. Connect to it via its Wi‑Fi access point (or your local network), open the web UI, and send keystrokes or run macros to the host PC.

---

## Contents

1. [Getting started](#getting-started)
2. [Console page](#console-page)
   - [Status bar](#status-bar)
   - [Send keys](#send-keys)
   - [Lock keys](#lock-keys)
   - [Media keys](#media-keys)
   - [Macros](#macros)
3. [Macro script language](#macro-script-language)
   - [Key names](#key-names)
   - [Modifier names](#modifier-names)
   - [Media key names](#media-key-names)
4. [Wi‑Fi settings](#wi-fi-settings)
5. [USB Device config](#usb-device-config)
6. [Hardware settings](#hardware-settings)

---

## Getting started

1. Power the device. It broadcasts a Wi‑Fi access point.
2. Connect your phone or laptop to that AP.
3. Open `http://192.168.4.1` in a browser.
4. Plug the device into the target PC via the USB‑OTG port.
5. Type in the text area or run a macro.

> The device can also join your existing Wi‑Fi network.

---

# Console page

## Status bar

| Indicator | Meaning |
|---|---|
| USB HID | Whether the target PC has enumerated the keyboard |
| Wi‑Fi Clients | Number of connected AP clients |
| Wi‑Fi STA | External Wi‑Fi connection status |

## Send keys

Click in the text area and type. With **Send keys immediately** enabled every keystroke is forwarded in real time.

Use the modifier checkboxes (`LCtrl`, `RCtrl`, `LShift`, `RShift`, `LAlt`, `RAlt`, `Super`) to hold modifiers while typing.

## Lock keys

The **NUM / CAPS / SCRL** indicators reflect the lock-key state reported by the host.

## Media keys

The media buttons send HID consumer-control reports:

- Play/Pause
- Previous
- Next
- Mute
- Volume Down
- Volume Up

## Macros

A macro is a named script stored in flash.

- Save up to 20 macros
- Execute macros in the background
- Continue using the UI while macros run

---

# Macro script language

Each line of a macro body is one command.

| Command | Description | Example |
|---|---|---|
| `STRING <text>` | Type text character by character | `STRING hello world` |
| `KEY <name>` | Press and release one key | `KEY ENTER` |
| `COMBO <mod>+<key>` | Hold modifiers and press a key simultaneously | `COMBO LCTRL+C` |
| `MEDIA <name>` | Send a media key | `MEDIA PLAY_PAUSE` |
| `DELAY <ms>` | Wait in milliseconds | `DELAY 500` |

## Example macro

```text
STRING Hello from macro!
KEY ENTER
DELAY 200
COMBO LCTRL+S
DELAY 1000
MEDIA VOL_UP
```

## Key names

| Name(s) | Key |
|---|---|
| `A`–`Z` | Letter keys |
| `0`–`9` | Digit keys |
| `F1`–`F12` | Function keys |
| `ENTER` | Enter / Return |
| `ESC` | Escape |
| `TAB` | Tab |
| `SPACE` | Space bar |
| `DELETE` | Delete |
| `HOME` | Home |
| `END` | End |
| `UP` | Arrow Up |
| `DOWN` | Arrow Down |
| `LEFT` | Arrow Left |
| `RIGHT` | Arrow Right |

## Modifier names

| Name(s) | Modifier |
|---|---|
| `LCTRL`, `CTRL` | Left Ctrl |
| `RCTRL` | Right Ctrl |
| `LSHIFT`, `SHIFT` | Left Shift |
| `RSHIFT` | Right Shift |
| `LALT`, `ALT` | Left Alt |
| `RALT` | Right Alt |
| `SUPER`, `GUI`, `WIN`, `META` | Left GUI / Super |

## Media key names

| Name(s) | Action |
|---|---|
| `PLAY_PAUSE` | Play / Pause |
| `NEXT` | Next track |
| `PREV` | Previous track |
| `VOL_UP` | Volume up |
| `VOL_DOWN` | Volume down |
| `MUTE` | Mute |

---

# Wi‑Fi settings

## Access Point (AP)

The device always runs its own AP so you can reach it without an external network.

Leave the password field empty for an open network.

Changes take effect after restart.

## Station (STA)

Use **Scan for Networks** to find nearby APs, enter credentials, and click **Save & Connect**.

Multiple networks can be saved.

---

# USB Device config

Accessible from `/usb`.

| Field | Description |
|---|---|
| VID | USB Vendor ID |
| PID | USB Product ID |
| Manufacturer | Manufacturer string |
| Product | Product string |
| Serial Number | Device serial number |

---

# Hardware settings

Accessible from `/hw`.

## Status LED

| Field | Description |
|---|---|
| GPIO Pin | GPIO number for status LED |
| NeoPixel (WS2812) | Use WS2812 protocol |
| Invert (active-low) | Invert GPIO logic |

### LED states

| State | Plain LED | NeoPixel |
|---|---|---|
| Booting / WPS active | 2 Hz blink | Blue blink |
| Idle | Off | Off |
| Wi‑Fi connected | Steady on | Green |
| Connecting | Pulse | Yellow pulse |
| Key being sent | Brief invert | Red flash |

## WPS Button

| Field | Description |
|---|---|
| GPIO Pin | GPIO for WPS button |

### Triggering WPS

- Hold the hardware button for 3 seconds
- Or click **WPS** in the web interface

The LED fast-blinks while WPS is active.
