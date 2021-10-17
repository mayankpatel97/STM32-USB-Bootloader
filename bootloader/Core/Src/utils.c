#include "main.h"
#include "fatfs.h"
#include "usb_host.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "menu.h"
/*
#define PACKET_HEADER_SIZE      ((uint32_t)3)
#define PACKET_DATA_INDEX       ((uint32_t)4)
#define PACKET_START_INDEX      ((uint32_t)1)
#define PACKET_NUMBER_INDEX     ((uint32_t)2)
#define PACKET_CNUMBER_INDEX    ((uint32_t)3)
#define PACKET_TRAILER_SIZE     ((uint32_t)2)
#define PACKET_OVERHEAD_SIZE    (PACKET_HEADER_SIZE + PACKET_TRAILER_SIZE - 1)
#define PACKET_SIZE             ((uint32_t)128)
#define PACKET_1K_SIZE          ((uint32_t)1024)

extern char USBHPath[4];
FATFS myUsbFatFS;

FIL myFile;
FRESULT res;
UINT byteswritten, bytesread;
char rwtext[100];
char msg[100];
unsigned int x;
unsigned char pointer,hexBuffer[0xFF];
bool startPacket,newPacket;
unsigned int atoh(char *encHex);




bool USB_DriveRead(void)
{
	bool read_data;
	uint8_t raw_byte,PrevValue;
	unsigned int byteCount=0;
	uint32_t flashdestination, ramsource, filesize;

	uint8_t *file_ptr;
	uint32_t *p_size;
	uint32_t packet_length1;

	read_data = true;

	uint8_t aPacketData[PACKET_1K_SIZE + PACKET_DATA_INDEX + PACKET_TRAILER_SIZE];
	//Open file for Reading
	if(f_open(&myFile, "BLY5.BIN", FA_READ) != FR_OK)
	//if(f_open(&myFile, "BLINKY.BIN", FA_READ) != FR_OK) 
	{
		HAL_GPIO_WritePin(RED_LED_GPIO_Port,RED_LED_Pin,GPIO_PIN_SET);
		return 0; //error
	}
	else
	{
		
		HAL_GPIO_WritePin(RED_LED_GPIO_Port,RED_LED_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port,ORANGE_LED_Pin,GPIO_PIN_SET);
		flashdestination = USER_START_ADDRESS;
		filesize = f_size(&myFile);

		PrevValue = 0x00;
		FLASH_If_Erase(USER_START_ADDRESS);
		*p_size = filesize;
		while(read_data == true)
		{
			ramsource = (uint32_t) & aPacketData[PACKET_DATA_INDEX];
			if(FLASH_If_Write(flashdestination,(uint32_t*)ramsource, packet_length1/4) == FLASHIF_OK)
			{
				flashdestination += packet_length1;
				//Serial_PutByte(ACK);
			}
		}
		
		//Erase_App_memory();
		//FLASH_WaitForLastOperation(100);
		
		
		//HAL_Delay(100);
		
		//HAL_FLASH_Unlock();

		//HAL_FLASH_Lock();

		//Reading error handling
		if(byteCount==0) 
			return 0;
	}
	
	//Close file
	f_close(&myFile);
	return 1; //success
}
*/

/*
unsigned int atoh(char *encHex)
{
	//"F1A6"
	unsigned int result = 0, result1 = 0;
	unsigned char i,h_array[8];
	//strcpy(msg,"Converting  Bytes ....\r\n");
	//HAL_UART_Transmit(&huart4,(unsigned char*)msg,strlen(msg),100);
	
	// first make an array of actual vaues 
	for(i=0;i<8;i++)
	{
		if( (encHex[i] >= '0' ) && (encHex[i] <= '9' ) )
		{
			h_array[i] = encHex[i] - '0';
		}
		
		else if( (encHex[i] >= 'A' ) && (encHex[i] <= 'F' ) ) 
		{
			h_array[i] = (encHex[i] - 'A') + 0x0A;
		}
	}
	result = 0;
	result1 = 0;

	for(i=0;i<8;i++)
	{
		result |= h_array[i] & 0x0F;
		if(i != 7)
			result <<=4;
	}
	
	
	result1 |= (result & 0xFF000000) >> 24;
	result1 |= (result & 0x00FF0000) >> 8;
	result1 |= (result & 0x0000FF00) << 8;
	result1 |= (result & 0x000000FF)<< 24;
	
	
	return result1;
}
*/

/*
 * if(raw_byte == 0x00)
			{
				break;
			}


			// capture packet
			if(raw_byte == ':')
			{
				pointer = 0;
				hexBuffer[0] = 0;
				startPacket = true;
				//HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port,ORANGE_LED_Pin,GPIO_PIN_SET);
			}
			else if(raw_byte == 0x0D)
			{
				if(startPacket == true)
				{
					startPacket=false;
					hexBuffer[pointer] = 0;
					newPacket=true;
				}
			}
			else if(startPacket == true)
			{
				if(pointer < 0xff)
				{
					HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port,ORANGE_LED_Pin,GPIO_PIN_SET);
					hexBuffer[pointer] = raw_byte;
					pointer++;
				}
			}
			// Analyse packet
			if(newPacket == true)
			{
				newPacket = false;
				//HAL_UART_Transmit(&huart4,&hexBuffer[0],strlen((char *)hexBuffer),100);
				//HAL_UART_Transmit(&huart4,(unsigned char *)"\r\n",2,100);

				//strcpy(msg,"Testing Purpose ... \r\n");

				if(strncmp((char *)hexBuffer,"10",2) == NULL)
				{
					// if hux buffer starts with 10
					// program is functioning well till here
					static unsigned char tmpBuffer[10],z;
					static unsigned int MemoryAdress;
					static unsigned int FlashBuffer[5];
					strncpy((char *)tmpBuffer,(char *)&hexBuffer[2],4);

					MemoryAdress = atoi((char* )tmpBuffer);

					for(z=0;z<4;z++)
					{
						strncpy((char *)tmpBuffer,(char *)&hexBuffer[8+z*8],8);
						FlashBuffer[z] = atoh((char *)tmpBuffer);
						//FlashBuffer[z] = strtol((char *)tmpBuffer, 0, 32);
					}
					//HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
					for(z=0;z<4;z++)
					{
						FLASH_WaitForLastOperation(100);
						//HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_APP_START_ADDRESS + MemoryAdress + z*4,FlashBuffer[z]);

					}

					static unsigned char len;
					len = strlen((char*)hexBuffer);
					HAL_UART_Transmit(&huart4,(unsigned char*)hexBuffer,len,100);
					HAL_UART_Transmit(&huart4,(unsigned char*)"\r\n",2,100);

				}


			}

			//HAL_UART_Transmit(&huart4,&raw_byte,1,100);
			PrevValue = raw_byte;
 *
 *
 */
