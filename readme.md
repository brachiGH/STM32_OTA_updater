# STM32 Over The Air Update System

A bootloader for stm32 with OTA update using a esp32 connect via uart.

<img src="diagram.png" alt="diagram" width="400"/>

```
/* Memories definition */
MEMORY
{
  RAM        (rwx)   : ORIGIN = 0x20000000,  LENGTH = 96K
  FLASH      (rx)    : ORIGIN = 0x08000000,  LENGTH = 32K       // Bootloader
  APPFLASH   (rwx)   : ORIGIN = 0x08008000,  LENGTH = 480K - 8  // Application / Firmware
  FIRMWAREID (rwx)   : ORIGIN = 0x0807FFF8,  LENGTH = 8         // Firmware ID
}
```
<img src="table.png" alt="Sectors Table" width="700"/>

Your Firmware memories definition should be samething like this:
```
MEMORY
{
  RAM      (xrw)  : ORIGIN = 0x20000000,   LENGTH = 96K
  FLASH    (rx)   : ORIGIN = 0x8008000,    LENGTH = 480K - 8
}
```

## License

This project is licensed under the MIT License.