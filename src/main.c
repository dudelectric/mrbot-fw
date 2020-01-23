#include "stddef.h"
#include "stm32f10x.h"
#include "misc.h"
#include "delay.h"
#include "mp3_lib.h"
#include "n3310.h"
#include "picture.h"
#include "serial.h"
#include "serial2.h"

int vol = 0;

int main(void) {
	//=== REMAP ===
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	//=============
	delay_init();

	/******************** DFplayer setup ********************/
	MP3_init();

	//Wait for DFPlayer Initialization
	delay_ms(500);

	//Set DFPlayer Volume
	MP3_send_cmd(MP3_VOLUME, 0, 10); // Volume 0-30
	delay_ms(1000);
	//Choose a folder with mp3-files
	//You can use some folders with different languages or different voices
	MP3_set_folder(1);
	delay_ms(10);

	//Play single file from folder
	//This command start playing file 032.mp3 from folder 05
	//MP3_send_cmd(MP3_PLAY_FOLDER_FILE, 2, 32); //folder 01..99, file 001..255

	//Make Voice QUEUE
//	MP3_say(MP3_NO_VALUE, 3148, MP3_NO_VALUE);
//	MP3_say(MP3_NO_VALUE, -35, MP3_NO_VALUE);
//	MP3_say(100, 153, 103);
//	MP3_say(MP3_NO_VALUE, 715, MP3_NO_VALUE);

	/******************** LCD n3310 setup ********************/
	LcdInit();
	LcdClear();

	/******************** serial1 setup ********************/
	//uart1 for debuging purpose
	delay_ms(50);
	serial_begin(9600);
	delay_ms(50);

	/******************** serial2 setup ********************/
	//uart2 for jdy-08 bluetooth module
	delay_ms(50);
	serial2_begin(9600);
	delay_ms(50);

//	if(vol == 0){
//		MP3_send_cmd(MP3_VOLUME, 0, 10); // Volume 0-30
//		delay_ms(1000);
//		LcdImage(Picture);
//		LcdUpdate();
//		delay_ms(5000);
//		vol = 1;
//	}

	while (1) {
//		if (serial2_available() > 0) {
//			char c = serial2_read();
//			serial_putchar(c);
//		}
//		if (serial_available() > 0) {
//			char c = serial_read();
//			serial2_putchar(c);
//		}

		for (int x = 0; x < 19; x++) {
			MP3_queue_processing();
			MP3_send_cmd(MP3_PLAY_FOLDER_FILE, 1, x); //folder 01..99, file 001..255
			LcdImage(face2);
			LcdUpdate();
			delay_ms(200);
			LcdImage(face1);
			LcdUpdate();
			delay_ms(800);
		}
	}
}

