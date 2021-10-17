# USB_Drive_bootloader

USB Drive bootloader for STM32

follow below steps

download the code
burn the bootloader file into the STM32F4 discovery board.
then copy the BLINK.BIN in to a USB drive and plug it on the USB port of the board using an OTG cable.
Then press and hold the "USER" button and Reset the board using a RESET switch.
if all goes right it will burn the "BLINK.BIN" in to flash and Green LED will start to blink showing success.
to run the application .. reset the board again. It will run the "BLINK.BIN" in which Blue and Green LEDs will be blinking at every 500mS.
