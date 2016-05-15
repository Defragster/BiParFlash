/* BiParFlash. Library - for filesystem-like access to SPI Serial Flash memory - paired chips 4 bits each
 *
 * ParallelFlash. Library - for filesystem-like access to SPI Serial Flash memory
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


// const int flash_sck   =  15; //PTC0 FlashPin 6
// const int flash_cs1   =  22; //PTC1 Flash1 Pin 1
// const int flash_cs2   =  23; //PTC2 Flash2 Pin 1

// Assemble "Parflash 8" board w/pins 16 and 17 cut, and with wires move those to 23 and 22:
#ifdef BEFORE_8	// Parallel Flash standard pins - HALF of PORT_D
/*
const int flash_sck   =  20; //PTD5 FlashPin 6
const int flash_cs    =   5; //PTD7 FlashPin 1

//Don't edit:
const int flash_sio0  =   2; //PTD0 FlashPin 5
const int flash_sio1  =  14; //PTD1 FlashPin 2
const int flash_sio2  =   7; //PTD2 FlashPin 3
const int flash_sio3  =   8; //PTD3 FlashPin 7

#define MASK_CS 		( pin_to_bitmask(flash_cs) ) //PTD7
#define MASK_SCK		( pin_to_bitmask(flash_sck) ) //PTD5
#define MASK_SIO0		( 1 )	//PTD0: SIO0..SIO3 not changeable!
#define MASK_ALL		( MASK_CS | MASK_SCK | 0x0f )

#define CSASSERT()  	{ GPIO_C->PCOR = MASK_CS; }
#define CSRELEASE() 	{ GPIO_C->PDOR = MASK_CS | MASK_SCK; }
#define CSRELEASESPI()	{ GPIO_C->PDOR = MASK_CS; }
*/
#endif

// CONTROL signals on PORT_C
// BUGBUG S==8 :: fla8h renamed to forced attention to all uses
const int fla8h_sck   =  23; //PTC2 FlashPin 6      // wire FROM teensy PIN 17
const int flash_cs1   =  15; //PTC0 Flash1 Pin 1
const int flash_cs2   =  22; //PTC1 Flash2 Pin 1    // wire FROM teensy PIN 16



//Don't edit: BiPar - DATA SIGNALS on PORT_D
// BUGBUG S==8 :: fla8h renamed to forced attention to all uses
const int fla8h_sio0  =   2; //PTD0 Flash_1 Pin 5 // First Par Flash Start
const int fla8h_sio1  =  14; //PTD1 Flash_1 Pin 2
const int fla8h_sio2  =   7; //PTD2 Flash_1 Pin 3
const int fla8h_sio3  =   8; //PTD3 Flash_1 Pin 7
const int flash_sio4  =   6; //PTD4 Flash_2 Pin 5 // Second Par Flash Start
const int flash_sio5  =  20; //PTD5 Flash_2 Pin 2
const int flash_sio6  =  21; //PTD6 Flash_2 Pin 3
const int flash_sio7  =   5; //PTD7 Flash_2 Pin 7
const int flash_sio0_1  =   flash_sio0; // First Par Flash :: flash_sio0 == PTD0 Flash_1 Pin 5
const int flash_sio0_2  =   flash_sio4; // Second Par Flash :: flash_sio4 == PTD4 Flash_2 Pin 5

// BUGBUG S==8 :: MA8K are renamed to forced attention to all uses
#define MA8K_CS1 		( pin_to_bitmask(flash_cs1) ) //PTC0
#define MA8K_CS2 		( pin_to_bitmask(flash_cs2) ) //PTC1
#define MA8K_CS			( MASK_CS1 | MASK_CS2 )
#define MA8K_SCK		( pin_to_bitmask(flash_sck) ) //PTC2
#define MA8K_SIO0		( 1 )	//PTD0: SIO0..SIO7 not changeable!
#define MASK_ALL		( MASK_CS | MASK_SCK )

// BUGBUG S==8 :: C8???() are renamed to forced attention to all uses
#define C8ASSERT()  	{ GPIO_C->PCOR = MASK_CS;  }
#define C8RELEASE() 	{ GPIO_C->PDOR = MASK_CS | MASK_SCK; }
#define C8RELEASESPI()	{ GPIO_C->PDOR = MASK_CS; }


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


void BiParFlashChip::writeByte(const uint8_t val) {
  uint32_t clk0 = GPIO_C->PDOR & ~MASK_ALL;
  uint32_t clk1 = clk0 | MASK_SCK; //CS=0; CLK=1

  GPIO_D->PDDR |= 0x0f;		// BUGBUG_DOUBLE_THIS :: Will use 8 bits PORT_D
  GPIO_C->PDOR = clk0;
  flash_Wait2;
  GPIO_D->PDOR = (val >> 4);
  GPIO_C->PDOR = clk1 ;
  flash_Wait2;
  GPIO_C->PDOR = clk0;
  flash_Wait2;
  GPIO_D->PDOR = (val & 0x0f);
  GPIO_C->PDOR = clk1;
}



void BiParFlashChip::writeBytes(const uint8_t * buf, const int len) {

  uint32_t clk0 = GPIO_C->PDOR & ~MASK_ALL;
  uint32_t clk1 = clk0 | MASK_SCK; //CS=0; CLK=1

  uint8_t *src = (uint8_t *) buf;
  uint8_t *target = src + len;

  uint8_t val;

  GPIO_D->PDDR |= 0x0f;		// BUGBUG_DOUBLE_THIS :: Will use 8 bits PORT_D

  while (src < target) {
	GPIO_C->PDOR = clk0;
	flash_Wait1;
	val = *src++;
	GPIO_D->PDOR = (val >> 4);
	GPIO_C->PDOR = clk1;
	flash_Wait2;
	GPIO_C->PDOR = clk0;
	flash_Wait2;
	GPIO_D->PDOR = (val & 0x0f);
	GPIO_C->PDOR = clk1;
  }
}

void BiParFlashChip::write16(const uint16_t val) {
	uint16_t buf = __REV16(val);
	writeBytes((uint8_t*) &buf, 2);
}

void BiParFlashChip::write32(const uint32_t val) {
	uint32_t buf = __REV(val);
	writeBytes((uint8_t*) &buf, 4);
}

uint8_t BiParFlashChip::readByte(void) {
  uint32_t val;

  GPIO_D->PDDR &= ~0x0f;		// BUGBUG_DOUBLE_THIS :: Will use 8 bits PORT_D
  GPIO_C->PCOR = MASK_CS;

  GPIO_C->PTOR = MASK_SCK;
  flash_Wait2;

  GPIO_C->PTOR = MASK_SCK;
  flash_Wait3;
  val = GPIO_D->PDIR;
  GPIO_C->PTOR = MASK_SCK;
  flash_Wait2;

  GPIO_C->PTOR = MASK_SCK;
  flash_Wait3;
  val = ((val & 0x0f) << 4) | ( GPIO_D->PDIR & 0x0f );
  return val;

}

void BiParFlashChip::readBytes( uint8_t * const buf, const int len) {
  if (len == 0) return;

  uint32_t val, val2;
  uint8_t *src = buf;
  const uint8_t *target = src + len;

  GPIO_D->PDDR &= ~0x0f;		// BUGBUG_DOUBLE_THIS :: Will use 8 bits PORT_D
  GPIO_C->PCOR = MASK_CS;

  while (((uintptr_t) src & 0x03) != 0 && src < target)  {

    GPIO_C->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_C->PTOR = MASK_SCK;
	flash_Wait3;
	val = GPIO_D->PDIR;

    GPIO_C->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_C->PTOR = MASK_SCK;
	flash_Wait3;
    val = ((val & 0x0f) << 4) | ( GPIO_D->PDIR & 0x0f );
	*src++= val;
  }

  while (src < target-4)  {
    GPIO_C->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_C->PTOR = MASK_SCK;
	flash_Wait3;
	val = GPIO_D->PDIR;
    GPIO_C->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_C->PTOR = MASK_SCK;
	flash_Wait3;
    val = ((val & 0x0f) << 4 ) | (GPIO_D->PDIR & 0x0f);

    GPIO_C->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_C->PTOR = MASK_SCK;
	flash_Wait3;
    val |= (GPIO_D->PDIR & 0x0f) << 12;
    GPIO_C->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_C->PTOR = MASK_SCK;
	flash_Wait3;
    val |= (GPIO_D->PDIR & 0x0f) << 8;

    GPIO_C->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_C->PTOR = MASK_SCK;
	flash_Wait3;
	val2 = GPIO_D->PDIR;
    GPIO_C->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_C->PTOR = MASK_SCK;
	flash_Wait3;
    val |= ((val2 & 0x0f) << 20) | (GPIO_D->PDIR & 0x0f) << 16;

    GPIO_C->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_C->PTOR = MASK_SCK;
	flash_Wait3;
	val2 = GPIO_D->PDIR;
    GPIO_C->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_C->PTOR = MASK_SCK;
	flash_Wait3;
    val |= ((val2 & 0x0f)<<28) | (GPIO_D->PDIR & 0x0f) << 24;

	*(uint32_t*) src = val;
	src += 4;
  }

  while (src < target)  {
    GPIO_C->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_C->PTOR = MASK_SCK;
	flash_Wait3;
	val = GPIO_D->PDIR;

    GPIO_C->PTOR = MASK_SCK;
	flash_Wait2;
    GPIO_C->PTOR = MASK_SCK;
	flash_Wait3;
    val = ((val & 0x0f) << 4) | ( GPIO_D->PDIR & 0x0f );
	*src++= val;
  }

}

void BiParFlashChip::wait(void)
{
	uint32_t status;
	//Serial.print("wait-");
	writeByte(0x05); //Read Status Byte #1
	while (1) {
	//	writeByte(0x05); //Read Status Byte #1
		status = readByte();
		//Serial.printf("b=%02x.", status & 0xFF);
		if (!(status & 1)) break;
	}
	CSRELEASE();
	busy = 0;

	//Serial.println();
}

void BiParFlashChip::read(uint32_t addr, void *buf, uint32_t len)
{
	uint8_t *pp = (uint8_t *)buf;
	uint8_t bb, ff, status;

	ff = flags;
	bb = busy;
	if (bb) {
		// read status register ... chip may no longer be busy
		writeByte(0x05); //Read Status Byte #1
		status = readByte();
		if (!(status & 1)) bb = 0;
		CSRELEASE();
		if (bb == 0) {
			// chip is no longer busy :-)
			busy = 0;
		} else if (bb < 3) {
			writeByte(0x06); // write enable
			CSRELEASE();

			writeByte(0x75);// Suspend command
			CSRELEASE();

			writeByte(0x05); //Read Status Byte #1
			do {
				status = readByte();
			} while ((status & 0x01));
			CSRELEASE();

		} else {
			// chip is busy with an operation that can not suspend
			wait();			// should we wait without ending
			bb = 0;			// the transaction??
		}
	}


	if (ff & FLAG_32BIT_ADDR) {
		writeByte(0x0b);
		write32(addr);
		readByte();//dummy
		/*
		readByte();//dummy
		readByte();//dummy
		*/
	} else {
		write32((0x0b << 24) | addr);
		readByte();//dummy
		/*
		readByte();//dummy
		readByte();//dummy
		*/
	}

	readBytes(pp, len);
	CSRELEASE();

	if (bb) {
		writeByte(0x06); // write enable
		CSRELEASE();
		writeByte(0x7A);// Suspend command
		CSRELEASE();
	}
}

void BiParFlashChip::write(uint32_t addr, const void *buf, uint32_t len)
{
	const uint8_t *pp = (const uint8_t *)buf;
	uint32_t max, pagelen;

	 //Serial.printf("WR: addr %08X, len %d\n", addr, len);
	do {
		if (busy) wait();
		writeByte(0x06); // write enable
		CSRELEASE();
		max = 256 - (addr & 0xFF);
		pagelen = (len <= max) ? len : max;
		 //Serial.printf("WR: addr %08X, pagelen %d\n", addr, pagelen);
		if (flags & FLAG_32BIT_ADDR) {
			writeByte(0x02); // program page command
			//write16(addr >> 16);
			//write16(addr);
			write32(addr);
		} else {
			//write16(0x0200 | ((addr >> 16) & 255));
			//write16(addr);
			write32((0x02 << 24) | addr);
		}
		addr += pagelen;
		len -= pagelen;
		do {
			writeByte(*pp++);
		} while (--pagelen > 0);
		CSRELEASE();
		busy = 1;
	} while (len > 0);
}

void BiParFlashChip::eraseAll()
{	// BUGBUG_DOUBLE_THIS
	if (busy) wait();
	uint8_t id[3];
	readID(id);
	//Serial.printf("ID: %02X %02X %02X\n", id[0], id[1], id[2]);
	// bulk erase command
	writeByte(0x06); // write enable
	CSRELEASE();
	writeByte(0xC7);
	CSRELEASE();
	busy = 3;
}

void BiParFlashChip::eraseBlock(uint32_t addr)
{
	uint8_t f = flags;
	if (busy) wait();
	writeByte(0x06); // write enable
	CSRELEASE();
	if (f & FLAG_32BIT_ADDR) {
		writeByte(0xD8);
		//write16(addr >> 16);
		//write16(addr);
		write32(addr);
	} else {
		//write16(0xD800 | ((addr >> 16) & 255));
		//write16(addr);
		write32((0xD8 << 24) | addr);
	}
	CSRELEASE();
	busy = 2;
}


bool BiParFlashChip::ready()
{
	uint32_t status;
	if (!busy) return true;
	// BUGBUG_DOUBLE_THIS

	// all others work by simply reading the status reg
	writeByte(0x05);
	status = readByte();
	CSRELEASE();
	//Serial.printf("ready=%02x\n", status & 0xFF);
	if ((status & 1)) return false;

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
	// BUGBUG_DOUBLE_THIS
	CSASSERT();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x38); //Enter QPI Mode
	CSRELEASESPI();
	GPIO_D->PDDR &= ~0x0f;	// BUGBUG - SET PDDR ??? :: IS THIS WHERE control pins on PORT_C are set???

	/* BUGBUG - IS THIS COMPLETE??
	writeByte(0xC0);
	writeByte(0x20);
	CSRELEASE();
	*/
}

void BiParFlashChip::exitQPI()
{
	if (busy) wait();
	// BUGBUG_DOUBLE_THIS
	writeByte(0xff); // Exit QPI Mode
	CSRELEASE();
	GPIO_C->PCOR = MASK_SCK;
	GPIO_D->PDDR = (GPIO_D->PDDR & ~0x0f) | MASK_SIO0; //BUGBUG
}

bool BiParFlashChip::begin()
{	// BUGBUG_DOUBLE_THIS as needed
	uint8_t id[3];
	uint8_t ff;
	uint32_t size;

	pinMode(flash_cs1, OUTPUT);
	digitalWriteFast(flash_cs1, 1);
	pinMode(flash_cs2, OUTPUT);
	digitalWriteFast(flash_cs2, 1);
	pinMode(flash_sck, OUTPUT);
	digitalWriteFast(flash_sck, 1);


	pinMode(flash_sio0_1, OUTPUT);
	pinMode(flash_sio1, INPUT);
	pinMode(flash_sio0_2, OUTPUT);
	pinMode(flash_sio5, INPUT);

	//Reset/Hold
	pinMode(flash_sio3, INPUT_PULLUP);

	//Software Reset
	CSASSERT();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x66);
	CSRELEASESPI();
	delayMicroseconds(1);
	CSASSERT();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x99);
	CSRELEASESPI();
	delayMicroseconds(100);

	//Configure Pins for fast switching/reading
	*portConfigRegister(flash_sio0) = 0x100;	// First Par Flash
	*portConfigRegister(flash_sio1) = 0x100;
	*portConfigRegister(flash_sio2) = 0x100;
	*portConfigRegister(flash_sio3) = 0x100;
	*portConfigRegister(flash_sio4) = 0x100;	// Second Par Flash
	*portConfigRegister(flash_sio5) = 0x100;
	*portConfigRegister(flash_sio6) = 0x100;
	*portConfigRegister(flash_sio7) = 0x100;
    *portConfigRegister(flash_cs1)	= 0x100;
    *portConfigRegister(flash_cs2)	= 0x100;
	*portConfigRegister(flash_sck)	= 0x100;

	//PORTD_DFER = 0;
	//Don't use pinMode below this point! It would change the PCR settings

	CSASSERT();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x06); //Write Enable
	CSRELEASESPI();
#define DEBUG
#ifdef DEBUG
	CSASSERT();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x05); //Read Status Register 1
	uint8_t rr = shiftIn( flash_sio1 ,  flash_sck , MSBFIRST);
	CSRELEASESPI();
	Serial.print("Status Register 1:0x");
	Serial.println(rr, HEX);

	CSASSERT();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x15); //Read Status Register 3
	rr = shiftIn( flash_sio1 ,  flash_sck , MSBFIRST);
	Serial.print("Status Register 3:0x");
	Serial.println(rr, HEX);
	CSRELEASESPI();

#endif
	//Enter QPI Mode

	//Serial.println("Enabling QPI in SR2");
	//Enable QPI in Statusregister 2:
	//digitalWriteFast(flash_cs, 0);
	CSASSERT();
	shiftOut( flash_sio0,  flash_sck , MSBFIRST, 0x31); //Write Status Register 2
	shiftOut( flash_sio0,  flash_sck , MSBFIRST,  2);
	CSRELEASESPI();
	delay(16); //needed here
#ifdef DEBUG
	CSASSERT();
	shiftOut( flash_sio0 ,  flash_sck , MSBFIRST, 0x35); //Read Status Register 2
	rr = shiftIn( flash_sio1 ,  flash_sck , MSBFIRST);
	CSRELEASESPI();
	Serial.print("Status Register 2:0x");
	Serial.println(rr, HEX);
#endif

	enterQPI();

	readID(id);
	ff = 0;
	size = capacity(id);


	// BUGBUG :: BiPar with two 16MB is 32MB data - This won't APPLY ??
	// Each chip still 16MB addressed - but there will be 32M_nibbles
	if (size > 16777216) {
		// more than 16 Mbyte requires 32 bit addresses
		ff |= FLAG_32BIT_ADDR;
		// micron & winbond & macronix use command
		writeByte(0x06); // write enable
		CSRELEASE();
		writeByte(0xB7); // enter 4 byte addr mode
		CSRELEASE();

	}


	if ((id[0]=ID0_WINBOND) && (id[1]==0x60) && (id[2]>=18)) {

		uint8_t r3;
		writeByte(0x15);
		r3 = readByte();
		CSRELEASE();

		//Serial.print("Status Register 3:0x");
		//Serial.println(r3, HEX);

		//Remove block locks (Winbond)
		writeByte(0x06); // write enable
		CSRELEASE();

		writeByte(0x98); // global block unlock
		CSRELEASE();

		if (r3 != 0x00) {

			writeByte(0x06); // write enable
			CSRELEASE();

			writeByte(0x11); //write statusregister 3
			writeByte(0x00);
			CSRELEASE();
		}

		writeByte(0x06); // write enable
		CSRELEASE();

	}

	flags = ff;
	return true;
}

// chips tested: https://github.com/PaulStoffregen/ParallelFlash./pull/12#issuecomment-169596992
//
void BiParFlashChip::sleep()
{	// BUGBUG_DOUBLE_THIS
	if (busy) wait();
	writeByte(0xB9); // Deep power down command
	CSRELEASE();
}

void BiParFlashChip::wakeup()
{	// BUGBUG_DOUBLE_THIS
	writeByte(0xAB); // Wake up from deep power down command
	CSRELEASE();
}

void BiParFlashChip::readID(uint8_t *buf)
{	// BUGBUG_DOUBLE_THIS :: Double incoming BUFFER !!!!!
	if (busy) wait();

	writeByte(0x9F);
	readBytes(buf, 3);  // manufacturer ID, memory type, capacity
	CSRELEASE();
//	Serial.printf("ID: %02X %02X %02X\n", buf[0], buf[1], buf[2]);
}

void BiParFlashChip::readSerialNumber(uint8_t *buf) //needs room for 8 bytes
{	// BUGBUG_DOUBLE_THIS :: Double incoming BUFFER !!!!!

	exitQPI();

	GPIO_D->PCOR = (1<<5);
	CSASSERT();
	// BUGBUG_DOUBLE_THIS
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

uint32_t BiParFlashChip::capacity(const uint8_t *id)
{	// BUGBUG_DOUBLE_THIS :: Double VALUE !!!!!
	if (id[2] >= 16 && id[2] <= 31) {
		return 1ul << id[2];
	} else
	if (id[2] >= 32 && id[2] <= 37) {
		return 1ul << (id[2] - 6);
	} else
	if ((id[0]==0 && id[1]==0 && id[2]==0) || 
		(id[0]==255 && id[1]==255 && id[2]==255)) {
		return 0;
	} else	
	return 1048576; // unknown chips, default to 1 MByte
}

uint32_t BiParFlashChip::blockSize()
{	// BUGBUG_DOUBLE_THIS :: Confirm BOTH 
	// Spansion chips >= 512 mbit use 256K sectors
	if (flags & FLAG_256K_BLOCKS) return 262144;
	// everything else seems to have 64K sectors
	return 65536;
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
