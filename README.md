# USB_Drive_bootloader
USB Drive bootloader for STM32

follow below steps

1) download the code 
2) burn the bootloader file into the STM32F4 discovery board.
3) then copy the BLINK.BIN in to a USB drive and plug it on the USB port of the board using an OTG cable.
4) Then press and hold the "USER" button and Reset the board using a RESET switch.
5) if all goes right it will burn the "BLINK.BIN" in to flash and Green LED will start to blink showing success.
6) to run the application .. reset the board again. It will run the "BLINK.BIN" in which Blue and Green LEDs will be blinking at every 500mS.


