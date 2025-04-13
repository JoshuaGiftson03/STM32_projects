#include "main.h"
//#include "spi.h"
//#include "usb_host.h"
//#include "gpio.h"
#include "mfrc522.h"
#include "my_delay.h"

void SystemClock_Config(void);
void MX_USB_HOST_Process(void);
uint8_t CompareID(uint8_t* CardID, uint8_t* CompareID);

uint8_t status; uint8_t g_ucTempbuf[20];  bool flag_loop=0;


/*HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USB_HOST_Init();
  MX_SPI1_Init();*/
int main(void)
{
	//4 bytes = 32 bits
	uint8_t MyID[5] = {0xE4, 0xA8, 0x8A, 0x3F};

		 //cho phep USART2 hoat dong
		MF522_init();
		delay_init(168);
    MFRC522_Reset();
//    MFRC522_AntennaOff();
    MFRC522_AntennaOn();
		//printf("CHUONG TRINH RFID\r\n");
	 //Read64Block();
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	while(1)
	{
		status = MFRC522_Request(PICC_REQALL, g_ucTempbuf);
    if (status != MI_OK)
    {
		  flag_loop=0;
      continue;
    }
		status = MFRC522_Anticoll(g_ucTempbuf);
    if (status != MI_OK)
         {
					 flag_loop=0;
           continue;
         }
	  if(flag_loop==1)
        {
				 MFRC522_Halt();
				 continue;
				}
		flag_loop=1;
		//printf("\n UID=%x:%x:%x:%x\r\n",g_ucTempbuf[0],g_ucTempbuf[1],g_ucTempbuf[2],g_ucTempbuf[3] );
		MFRC522_Halt();
	  if (CompareID(g_ucTempbuf,MyID) == MI_OK)
			{
				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); // Blue ON
				  delay_ms(500);
				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); // Blue OFF
			}
		else
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // Red ON
				delay_ms(500);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // Red OFF
			}


	}
}
uint8_t CompareID(uint8_t* CardID, uint8_t* CompareID)
{
	uint8_t i;
	for (i = 0; i < 5; i++) {
		if (CardID[i] != CompareID[i])
			{
			return MI_ERR;
		  }
	}
	return MI_OK;
}

