/* ParallelFlash. Library - for filesystem-like access to SPI Serial Flash memory
 * https://github.com/FrankBoesing/ParallelFlash
 * Copyright (C) 2015,2016, Paul Stoffregen, paul@pjrc.com, f.boesing
 *
 * Development of this library was funded by PJRC.COM, LLC by sales of Teensy.
 * Please support PJRC's efforts to develop open source software by purchasing
 * Teensy or other genuine PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "BiParFlash.h"
#include "GPIOhelper.h"
#include <arm_math.h>


const int flash_sck   =  17; //PTB1 FlashPin 6
const int flash_cs    =  15; //PTC0 FlashPin 1 Flash #0
const int flash1_cs    = 16; //PTB0 FlashPin 1 Flash #1

//Don't edit:
const int flash_sio0  =   2; //PTD0 FlashPin 5 Flash #0
const int flash_sio1  =  14; //PTD1 FlashPin 2 Flash #0
const int flash_sio2  =   7; //PTD2 FlashPin 3 Flash #0
const int flash_sio3  =   8; //PTD3 FlashPin 7 Flash #0
const int flash1_sio0 =   6; //PTD4 FlashPin 5 Flash #1
const int flash1_sio1 =  20; //PTD5 FlashPin 2 Flash #1
const int flash1_sio2 =  21; //PTD6 FlashPin 3 Flash #1
const int flash1_sio3 =   5; //PTD7 FlashPin 7 Flash #1


#define MASK_CS 		( pin_to_bitmask(flash_cs) )
#define MASK1_CS 		( pin_to_bitmask(flash1_cs) )
#define MASK_SCK		( pin_to_bitmask(flash_sck) )

#define MASK_SIO0		( 0x01 )	
#define MASK1_SIO0		( 0x10 )
 #define MASK_ALL		( MASK_CS | MASK_SCK | 0x0f )

#define DATA_INPUT 		{GPIO_D->PDDR &=~0xff;}					
#define DATA_OUTPUT 	{GPIO_D->PDDR |= 0xff;}
#define DATA_OUT		GPIO_D->PDOR
#define DATA_IN			GPIO_D->PDIR
#define CLK0			{GPIO_B->PCOR = MASK_SCK;} 				// CLK both chips = 0 
#define CLK1			{GPIO_B->PSOR = MASK_SCK;} 				// CLK both chips = 1 
#define CSASSERT() 		{GPIO_C->PCOR = MASK_CS;GPIO_B->PCOR = MASK1_CS; } //CS both chips = 0
#define CSRELEASE() 	{GPIO_C->PSOR = MASK_CS;GPIO_B->PSOR = MASK1_CS  | MASK_SCK;} //UN-CS Both Chips (=1),  SCK=1

#define CSASSERTSPI()  	{ GPIO_C->PCOR = MASK_CS; }				// Flash #0
#define CSRELEASESPI()	{ GPIO_C->PSOR = MASK_CS; }				// Flash #0

#define CSASSERTSPI1()  { GPIO_B->PCOR = MASK1_CS; }			// Flash #1
#define CSRELEASESPI1()	{ GPIO_B->PSOR = MASK1_CS; }			// Flash #1


#if (F_CPU == 144000000)
#define flash_Wait0 {  }
#define flash_Wait1 {  }
#define flash_Wait2 {  }
#define flash_Wait3 { asm volatile ("\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n"); }
#elif (F_CPU == 120000000)
#define flash_Wait0 {  }
#define flash_Wait1 {  }
#define flash_Wait2 {  }
#define flash_Wait3 { asm volatile ("\tNOP\n\tNOP\n\tNOP\n"); }
#elif (F_CPU == 96000000)
#define flash_Wait0 {  }
#define flash_Wait1 {  }
#define flash_Wait2 {  }
#define flash_Wait3 { asm volatile ("\tNOP\n\tNOP\n\tNOP\n"); }
#elif (F_CPU == 72000000)
#define flash_Wait0 {  }
#define flash_Wait1 {  }
#define flash_Wait2 {  }
#define flash_Wait3 { asm volatile ("\tNOP\n\tNOP\n"); }
#elif (F_CPU <= 48000000)
#define flash_Wait0 {  }
#define flash_Wait1 {  }
#define flash_Wait2 {  }
#define flash_Wait3 {  asm volatile ("\tNOP\n"); }
#endif


uint16_t BiParFlashChip::dirindex = 0;
uint8_t BiParFlashChip::flags = 0;
uint8_t BiParFlashChip::busy = 0;

#define ID0_WINBOND	0xEF

#define FLAG_32BIT_ADDR		0x01	// larger than 16 MByte address
//#define FLAG_STATUS_CMD70	0x02	// requires special busy flag check
//#define FLAG_DIFF_SUSPEND	0x04	// uses 2 different suspend commands
//#define FLAG_MULTI_DIE		0x08	// multiple die, don't read cross 32M barrier
#define FLAG_256K_BLOCKS	0x10	// has 256K erase blocks
//#define FLAG_DIE_MASK		0xC0	// top 2 bits count during multi-die erase


void BiParFlashChip::writeByteBoth(const uint8_t val) {
	//write the same byte to both chips
	
  DATA_OUTPUT;
  CSASSERT();
  CLK0;
  
  flash_Wait2;
  DATA_OUT = (val & 0xf0) | (val >> 4);
  CLK1;  
  flash_Wait2;  
  CLK0;
  flash_Wait2;
  DATA_OUT = (val & 0x0f) | (val << 4);
  CLK1;
  flash_Wait2;
}


void BiParFlashChip::write32Both(const uint32_t val) {
//	uint32_t buf = __REV(val);
//	writeBytes((uint8_t*) &buf, 4);
	writeByteBoth((val>>24) & 0xff);
	writeByteBoth((val>>16) & 0xff);
	writeByteBoth((val>> 8) & 0xff);
	writeByteBoth((val>> 0) & 0xff);
}

uint16_t BiParFlashChip::readByteBoth(void) {
 //returns one byte from both chips
  uint16_t val,val2;

  DATA_INPUT;
  CSASSERT();
  CLK0;
  flash_Wait2;
  CLK1;
  flash_Wait3;
  val = DATA_IN;
  CLK0;
  flash_Wait2;
  CLK1;
  flash_Wait3;
  val2 = DATA_IN ;
  return ((val<<8) & 0xf000) | ((val2<<4) & 0x0f00) | ((val<<4) & 0xf0) | (val2 & 0x0f);

}

void BiParFlashChip::readBytes( uint8_t * const buf, const int len) {

  if (len == 0) return;

//  uint32_t val, val2;
  uint8_t *src = buf;
  const uint8_t *target = src + len;

  DATA_INPUT;
  CSASSERT();
  
/* 
 TODO.....!!!!!!
  while (((uintptr_t) src & 0x03) != 0 && src < target)  {

    GPIO_D->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
	val = GPIO_D->PDIR;

    GPIO_D->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
    val = ((val & 0x0f) << 4) | ( GPIO_D->PDIR & 0x0f );
	*src++= val;
  }
  
 TODO.....!!!!!!
  while (src < target-4)  {
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
	val = GPIO_D->PDIR;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
    val = ((val & 0x0f) << 4 ) | (GPIO_D->PDIR & 0x0f);

    GPIO_D->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
    val |= (GPIO_D->PDIR & 0x0f) << 12;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
    val |= (GPIO_D->PDIR & 0x0f) << 8;

    GPIO_D->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
	val2 = GPIO_D->PDIR;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
    val |= ((val2 & 0x0f) << 20) | (GPIO_D->PDIR & 0x0f) << 16;

    GPIO_D->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
	val2 = GPIO_D->PDIR;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
    val |= ((val2 & 0x0f)<<28) | (GPIO_D->PDIR & 0x0f) << 24;

	*(uint32_t*) src = val;
	src += 4;
  }
*/
  while (src < target)  {
	CLK0;
	flash_Wait2;
	CLK1;
	flash_Wait3;
	*src++ = DATA_IN;	  
	  
/*	  
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
	val = GPIO_D->PDIR;

    GPIO_D->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_D->PTOR = MASK_SCK;
	flash_Wait3;
    val = ((val & 0x0f) << 4) | ( GPIO_D->PDIR & 0x0f );
	*src++= val;
*/	
  }

}

void BiParFlashChip::wait(void)
{
	uint16_t status;
	//Serial.print("wait-");
	writeByteBoth(0x05); //Read Status Byte #1
	while (1) {
	//	writeByte(0x05); //Read Status Byte #1
		status = readByteBoth();
		//Serial.printf("b=%02x.", status & 0xFF);
		if (!(status & 0x0101)) break;
	}
	CSRELEASE();
	busy = 0;

	//Serial.println();
}

void BiParFlashChip::read(uint32_t addr, void *buf, uint32_t len)
{
	uint8_t *p = (uint8_t *)buf;
	uint8_t b, f;
	uint16_t status;

	addr /=2;
	
	f = flags;
	b = busy;
	if (b) {
		// read status register ... chip may no longer be busy
		writeByteBoth(0x05); //Read Status Byte #1
		status = readByteBoth();
		if (!(status & 0x0101)) b = 0;		
		CSRELEASE();
		if (b == 0) {
			// chip is no longer busy :-)
			busy = 0;
		} else if (b < 3) {
			writeByteBoth(0x06); // write enable
			CSRELEASE();

			writeByteBoth(0x75);// Suspend command
			CSRELEASE();

			writeByteBoth(0x05); //Read Status Byte #1
			do {
				status = readByteBoth();
			} while ((status & 0x0101));
			CSRELEASE();

		} else {
			// chip is busy with an operation that can not suspend
			wait();			// should we wait without ending
			b = 0;			// the transaction??
		}
	}


	if (f & FLAG_32BIT_ADDR) {
		writeByteBoth(0x0b);
		write32Both(addr);
		readByteBoth();
	} else {
		write32Both((0x0b << 24) | addr);
		readByteBoth();
	}

	readBytes(p, len);
	CSRELEASE();

	if (b) {
		writeByteBoth(0x06); // write enable
		CSRELEASE();
		writeByteBoth(0x7A);// Suspend command
		CSRELEASE();
	}
}

void BiParFlashChip::write(uint32_t addr, const void *buf, uint32_t len)
{
	uint8_t *p = (uint8_t *)buf;
	uint32_t max, pagelen;

	 //Serial.printf("WR: addr %08X, len %d\n", addr, len);
	do {
		if (busy) wait();
		writeByteBoth(0x06); // write enable
		CSRELEASE();
		max = 256 - (addr & 0xFF);
		pagelen = (len <= max) ? len : max;
		 //Serial.printf("WR: addr %08X, pagelen %d\n", addr, pagelen);

		//writeBytes(p++, pagelen );

		{ //WriteBytes

			uint8_t byteOne;
			//Serial.printf("wr addr %d\n",addr);
			if (addr & 0x01) {			
				read(addr, &byteOne, 1);
				Serial.printf("rd addr %d=%d\n",addr, byteOne);
			}
/*
			if (flags & FLAG_32BIT_ADDR) {
				writeByteBoth(0x02); // program page command
				write32Both(addr / 2);
			} else {
				write32Both((0x02 << 24) | (addr / 2) );
			}
*/
			write32Both((0x02 << 24) | (addr / 2) );
			//DATA_OUTPUT;
			//CSASSERT();

			if (addr & 0x01) {
				CLK0;
				flash_Wait1;
				DATA_OUT = byteOne;
				CLK1;
				flash_Wait2;
			}

			addr += pagelen;
			len -= pagelen;

			uint8_t *target = p + pagelen;

			while (p < target) {
				CLK0;
				flash_Wait1;
				DATA_OUT = *p++;
				CLK1;  
				flash_Wait2;  	
			}	

			CSRELEASE();
		}

		busy = 1;
	} while (len > 0);
	DATA_INPUT;		
}

void BiParFlashChip::eraseAll()
{
	if (busy) wait();
	uint8_t id[3];
	readID(id);
	//Serial.printf("ID: %02X %02X %02X\n", id[0], id[1], id[2]);
	// bulk erase command
	writeByteBoth(0x06); // write enable
	CSRELEASE();
	writeByteBoth(0xC7);
	CSRELEASE();
	busy = 3;
}

void BiParFlashChip::eraseBlock(uint32_t addr)
{
	uint8_t f = flags;
		
	if (busy) wait();
	writeByteBoth(0x06); // write enable
	CSRELEASE();
	if (f & FLAG_32BIT_ADDR) {
		writeByteBoth(0xD8);
		write32Both(addr / 2);
	} else {
		write32Both((0xD8 << 24) | (addr / 2));
	}
	CSRELEASE();
	busy = 2;
}


bool BiParFlashChip::ready()
{
	uint16_t status;
	if (!busy) return true;

	// all others work by simply reading the status reg
	writeByteBoth(0x05);
	status = readByteBoth();
	CSRELEASE();
	//Serial.printf("ready=%04x\n", status);
	if ((status & 0x0101)) return false;

	busy = 0;
	if (flags & 0xC0) {
		// continue a multi-die erase
		eraseAll();
		return false;
	}
	return true;
}





void BiParFlashChip::enterQPI()
{
	if (busy) wait();
	
	//Flash 0
	CSASSERTSPI();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x38); //Enter QPI Mode
	CSRELEASESPI();

	//Flash 1
	CSASSERTSPI1();
	shiftOut( flash1_sio0 ,  flash_sck , MSBFIRST, 0x38); //Enter QPI Mode
	CSRELEASESPI1();

	GPIO_D->PDDR &= ~0xff;

	/*
	writeByte(0xC0);
	writeByte(0x20);
	CSRELEASE();
	*/
}

void BiParFlashChip::exitQPI()
{
	if (busy) wait();

	writeByteBoth(0xff); // Exit QPI Mode
	CSRELEASE();
	CLK0;
	DATA_INPUT;
	
	//GPIO_D->PDDR = (GPIO_D->PDDR & ~0x0f) | MASK_SIO0;
}

bool BiParFlashChip::begin()
{
	uint8_t id[3];
	uint8_t f;
	uint32_t size;

	pinMode(flash_cs, OUTPUT);
	pinMode(flash1_cs, OUTPUT);
	digitalWriteFast(flash_cs, 1);
	digitalWriteFast(flash1_cs, 1);	
	pinMode(flash_sck, OUTPUT);
	digitalWriteFast(flash_sck, 1);


	pinMode(flash_sio0, OUTPUT);
	pinMode(flash1_sio0, OUTPUT);
	pinMode(flash_sio1, INPUT);
	pinMode(flash1_sio1, INPUT);

	//Reset/Hold
	pinMode(flash_sio3, INPUT_PULLUP);
	pinMode(flash1_sio3, INPUT_PULLUP);

	//Software Reset
	//Flash 0
	CSASSERTSPI();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x66);
	CSRELEASESPI();
	//Flash 1
	CSASSERTSPI1();
	shiftOut( flash1_sio0 ,  flash_sck , MSBFIRST, 0x66);
	CSRELEASESPI();

	delayMicroseconds(1);
	//Flash 0
	CSASSERTSPI();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x99);
	CSRELEASESPI();
	//Flash 1
	CSASSERTSPI1();
	shiftOut( flash1_sio0 ,  flash_sck , MSBFIRST, 0x99);
	CSRELEASESPI1();
	
	delayMicroseconds(100);

	//Configure Pins for fast switching/reading
		
	*portConfigRegister(flash_sio0) = 0x100;
	*portConfigRegister(flash_sio1) = 0x100;
	*portConfigRegister(flash_sio2) = 0x100;
	*portConfigRegister(flash_sio3) = 0x100;
    *portConfigRegister(flash_cs)	= 0x100;

	*portConfigRegister(flash1_sio0) = 0x100;
	*portConfigRegister(flash1_sio1) = 0x100;
	*portConfigRegister(flash1_sio2) = 0x100;
	*portConfigRegister(flash1_sio3) = 0x100;
    *portConfigRegister(flash1_cs)	= 0x100;
	
	*portConfigRegister(flash_sck)	= 0x100;

	//PORTD_DFER = 0;
	//Don't use pinMode below this point! It would change the PCR settings

	CSASSERTSPI();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x06); //Write Enable
	CSRELEASESPI();
	
	CSASSERTSPI1();
	shiftOut( flash1_sio0 ,  flash_sck , MSBFIRST, 0x06); //Write Enable
	CSRELEASESPI1();
#define DEBUG
#ifdef DEBUG

	//Flash 0
	CSASSERTSPI();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x05); //Read Status Register 1
	uint8_t r = shiftIn( flash_sio1 ,  flash_sck , MSBFIRST);
	CSRELEASESPI();
	Serial.print("#0 Status Register 1:0x");
	Serial.println(r, HEX);
	//Flash 1
	CSASSERTSPI1();
	shiftOut( flash1_sio0 ,  flash_sck , MSBFIRST, 0x05); //Read Status Register 1
	r = shiftIn( flash1_sio1 ,  flash_sck , MSBFIRST);
	CSRELEASESPI1();
	Serial.print("#1 Status Register 1:0x");
	Serial.println(r, HEX);
	
	//Flash 0
	CSASSERTSPI();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x15); //Read Status Register 3
	r = shiftIn( flash_sio1 ,  flash_sck , MSBFIRST);
	Serial.print("#0 Status Register 3:0x");
	Serial.println(r, HEX);
	CSRELEASESPI();
	//Flash 1
	CSASSERTSPI1();
	shiftOut( flash1_sio0 ,  flash_sck , MSBFIRST, 0x15); //Read Status Register 3
	r = shiftIn( flash1_sio1 ,  flash_sck , MSBFIRST);
	Serial.print("#1 Status Register 3:0x");
	Serial.println(r, HEX);
	CSRELEASESPI1();

#endif
	//Enter QPI Mode

	//Serial.println("Enabling QPI in SR2");
	//Enable QPI in Statusregister 2:
	//digitalWriteFast(flash_cs, 0);

	//Flash 0
	CSASSERTSPI();
	shiftOut( flash_sio0,  flash_sck , MSBFIRST, 0x31); //Write Status Register 2
	shiftOut( flash_sio0,  flash_sck , MSBFIRST,  2);
	CSRELEASESPI();
	
	//Flash 1	
	CSASSERTSPI1();
	shiftOut( flash1_sio0,  flash_sck , MSBFIRST, 0x31); //Write Status Register 2
	shiftOut( flash1_sio0,  flash_sck , MSBFIRST,  2);
	CSRELEASESPI1();

	
	delay(16); //needed here
#ifdef DEBUG
	//Flash 0
	CSASSERTSPI();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x35); //Read Status Register 2
	r = shiftIn( flash_sio1 ,  flash_sck , MSBFIRST);
	CSRELEASESPI();
	Serial.print("#0 Status Register 2:0x");
	Serial.println(r, HEX);
	
	//Flash 1
	CSASSERTSPI1();
	shiftOut( flash1_sio0 ,  flash_sck , MSBFIRST, 0x35); //Read Status Register 2
	r = shiftIn( flash1_sio1 ,  flash_sck , MSBFIRST);
	CSRELEASESPI1();
	Serial.print("#1 Status Register 2:0x");
	Serial.println(r, HEX);
	
#endif

	enterQPI();

	readID(id);
	f = 0;
	size = capacity(id);

	if (size > 16777216*2) {
		// more than 16 Mbyte requires 32 bit addresses
		f |= FLAG_32BIT_ADDR;
		// micron & winbond & macronix use command
		writeByteBoth(0x06); // write enable
		CSRELEASE();
		writeByteBoth(0xB7); // enter 4 byte addr mode
		CSRELEASE();

	}


	if ((id[0]=ID0_WINBOND) && (id[1]==0x60) && (id[2]>=18)) {

		uint16_t r3;
		writeByteBoth(0x15);
		r3 = readByteBoth();
		CSRELEASE();

		Serial.print("Status Register 3:0x");
		Serial.println(r3, HEX);
/*
		//Remove block locks (Winbond)
		writeByteBoth(0x06); // write enable
		CSRELEASE();

		writeByteBoth(0x98); // global block unlock
		CSRELEASE();

		if (r3 != 0x00) {

			writeByteBoth(0x06); // write enable
			CSRELEASE();

			writeByteBoth(0x11); //write statusregister 3
			writeByteBoth(0x00);
			CSRELEASE();
		}
*/
		writeByteBoth(0x06); // write enable
		CSRELEASE();

	}

	flags = f;
	return true;
}

// chips tested: https://github.com/PaulStoffregen/ParallelFlash./pull/12#issuecomment-169596992
//
void BiParFlashChip::sleep()
{
	if (busy) wait();
	writeByteBoth(0xB9); // Deep power down command
	CSRELEASE();
}

void BiParFlashChip::wakeup()
{
	writeByteBoth(0xAB); // Wake up from deep power down command
	CSRELEASE();
}

void BiParFlashChip::readID(uint8_t *buf)
{
	if (busy) wait();

	writeByteBoth(0x9F);
	uint16_t b1, b2, b3;
	b1 = readByteBoth(); // manufacturer ID,
	b2 = readByteBoth(); // memory type
	b3 = readByteBoth(); // capacity
	CSRELEASE();
	
	if (((b1 >> 8) != (b1 & 0xff)) || ((b2 >> 8) != (b2 & 0xff)) || ((b3 >> 8) != (b3 & 0xff))) {
		buf[0]=buf[1]=buf[2]=0;//assume not working board if different IDs - return zero-id
	} else {
		buf[0]=b1; buf[1]=b2; buf[2]=b3;
	}
#ifdef DEBUG
 	Serial.printf("#0 ID: %02X %02X %02X\n#1 ID: %02X %02X %02X\n", b1>>8, b2>>8, b3>>8, b1 & 0xff, b2 & 0xff, b3 & 0xff);
#endif
}

/*
void BiParFlashChip::readSerialNumber(uint8_t *buf) //needs room for 8 bytes
{

	exitQPI();

	GPIO_D->PCOR = (1<<5);
	CSASSERT();

	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x4B);
	for (int i=0; i<4; i++) {
		shiftIn( flash_sio1 ,  flash_sck , MSBFIRST);
	}
	for (int i=0; i<8; i++) {
		buf[i] = shiftIn( flash_sio1 ,  flash_sck , MSBFIRST);
	}
	CSRELEASESPI();

	enterQPI();

//	uint8_t id[3];
//	readID(id);
    //Serial.printf("Serial Number: %02X %02X %02X %02X %02X %02X %02X %02X\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
}
*/

uint32_t BiParFlashChip::capacity(const uint8_t *id)
{
	if (id[2] >= 16 && id[2] <= 31) {
		return 2ul << id[2];
	} else
	if (id[2] >= 32 && id[2] <= 37) {
		return 2ul << (id[2] - 6);
	} else
	if (id[2]==0) return 0;
	else
	return 2*1048576; // unknown chips, default to 1 MByte
}

uint32_t BiParFlashChip::blockSize()
{
	// Spansion chips >= 512 mbit use 256K sectors
	if (flags & FLAG_256K_BLOCKS) return 262144;
	// everything else seems to have 64K sectors
	return 65536*2;
}




/*
Chip		Uniform Sector Erase
		20/21	52	D8/DC
		-----	--	-----
W25Q64CV	4	32	64
W25Q128FV	4	32	64

*/



//			size	sector			busy	pgm/erase	chip
// Part			Mbyte	kbyte	ID bytes	cmd	suspend		erase
// ----			----	-----	--------	---	-------		-----
// Winbond W25Q64CV	8	64	EF 60 17
// Winbond W25Q128FV	16	64	EF 60 18	05	single		60 & C7
// Winbond W25Q256FV	32	64	EF 60 19


BiParFlashChip BiParFlash;
