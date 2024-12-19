ESP32 (New) = 34:5f:45:a9:c1:08 (ESP-Touch/Sender)
ESP32 (Old) = b0:b2:1c:a6:45:a0 (ESP-Read/Receiver)

Tools being used are:
1. GUIslicer, GUIslicer-Builder, and GUIslicer Image2C - For building GUI for 2.8' TFT Displays
	Note: 
	> Follow the wirings from this link (https://www.youtube.com/watch?v=NvBblQnWhsQ&list=LL&index=4)
	Wiring:
	TD_VCC -> EXTERNAL 5V
	TD_GND -> EXTERNAL GND
	TD_CS -> ESP32 D15
	TD_RST -> ESP32 D4
	TD_DC -> ESP32 D2
	TD_MOSI -> ESP32 D23
	TD_SCK -> ESP32 D18
	TD_LED -> ESP32 3V3
	TD_MISO	-> ESP32 D19
	T_CLK -> ESP32 D18
	T_CS -> ESP32 D21
	T_DIN -> ESP32 D23
	T_DO -> ESP32 D19
	T_IRQ -> UNCONNECTED

	> Copy the content of User_Setup_Select.h from TFT_eSPI library in Arduino libraries, as well as the Setup42_ILI9341_ESP32.h (configurable for ST7789 or ILI9341 driver).
	> Uncomment "#include "../configs/esp-tftespi-default-xpt2046_int.h"" for touchscreen, and "#include "../configs/esp-tftespi-default-notouch.h"" for display only. Select only one or just copy the content.
	> Copy the "esp-tftespi_default_xpt2046_int.h" content (Touchscreen) or "esp-tftespi-default-notouch.h" (Display only) depending on the driver.
	> Use Image Extern for Flash with C-arrays inside C-files. Better if use SD Card than Flash Memory.
	
2. fastdup


Troubleshooting:
- If it goes white, and doesn't read touchscreen but displays check wires from touchscreen.