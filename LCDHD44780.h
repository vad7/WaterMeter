/*
 * LCD HD44780 Character 2x16 I2C supported
 *
 * Created: 19.02.2013 13:48:43
 *  Author: Vadim Kulakov (vad7@yahoo.com)
 */ 
#include <util/delay.h>
#include <avr/pgmspace.h>

#define LCDCH_I2C_INTERFACE // LCD connected to I2C interface, if omitted - direct connection

#ifdef LCDCH_I2C_INTERFACE
#define LCDCH_I2C_Addr			0x4E	// 0x27 (7-bit)
#define LCDCH_DATA				0x01	// RS pin
//#define LCDCH_READ  			0x02	// RW pin, if omitted - write only mode with delays
#define LCDCH_E					0x04
#define LCDCH_LED				0x08	// if omitted - lighting not used
#define LCDCH_DATASHIFT			4		// Starting pin for data 4..7
#define LCDCH_DATAMASK			(0x0F<<LCDCH_DATASHIFT)

uint8_t LCDCH_NotResponding		= 0;
#else

// Direct connected
// DATA PORT: D4..D7
#define LCDCH_DATAPORT			PORTA	// Data wire (4bit)
#define LCDCH_DATAPORTDDR		DDRA	// Direction
#define LCDCH_DATAPORTIN		PINA	// Read
#define LCDCH_DATASHIFT			0		// Starting pin for data 4..7
#define LCDCH_DATAMASK			(0x0F<<LCDCH_DATASHIFT)
// CONTROL PORT: RS,RW,E
#define LCDCH_CTRLPORT			PORTA
#define LCDCH_CTRLPORTDDR		DDRA	// Direction
#define LCDCH_DATA				(1<<PORTA4)	// RS pin, Selects registers (in sequence!)
//#define LCDCH_READ			(1<<PORTA5)	// RW pin, Selects read or write (in sequence!), if omitted then don't used - always write (RW pin to ground)
#define LCDCH_E					(1<<PORTA6)	// Starts data read/write (in sequence!)
#endif

#define LCDCH_TIMESTROBE		1  // us
#define LCDCH_COMMAND			0
#define LCDCH_FLAGBUSY			0x80
#define LCDCH_2LINES			0b1000
#define LCDCH_SECOND_FONT		0b0100

#define LCDCH__ClearDisplay		1
#define LCDCH__ReturnHome		2
#define LCDCH__EntryMode		4   
#define LCDCH__EntryModeShift	1	// Shift
#define LCDCH__EntryModeInc		2	// Increment by 1
#define LCDCH__Display			8
#define LCDCH__DisplayOn		4
#define LCDCH__DisplayCursorOn	2
#define LCDCH__DisplayCursorBlink 1
#define LCDCH__Shift			16  // shift display or cursor
#define LCDCH__ShiftDisplay		8   
#define LCDCH__ShiftCursor		0   
#define LCDCH__ShiftRight		4   
#define LCDCH__ShiftLeft		0   
#define LCDCH__SetCharDataPos	64	 // Low 0-5bits are address
#define LCDCH__SetCursorPos		128  // Low 0-6bits are address

#ifndef Delay100us
void Delay100us(uint8_t ms)
{
	while(ms-- > 0) _delay_us(100);
}
#endif

#ifdef LCDCH_LED
uint8_t LCDCH_Light	= LCDCH_LED; // On
void LCDCH_I2C_Write(uint8_t data)
{
	I2C_Write(data | LCDCH_Light);
}
#else
#define LCDCH_I2C_Write(data) I2C_Write(data);
#endif

void LCDCH_Write4b(uint8_t data) // if 0x8X - Command
{
#ifdef LCDCH_I2C_INTERFACE
	if(I2C_Start(LCDCH_I2C_Addr + I2C_WRITE)) {
		LCDCH_NotResponding = 1;
		return;
	}
	LCDCH_I2C_Write(data); // Strobe up
	LCDCH_I2C_Write(data | LCDCH_E); // Strobe up
	LCDCH_I2C_Write(data); // Strobe down
	I2C_Stop();
#else
	LCDCH_DATAPORTDDR |= LCDCH_DATAMASK; // OUT
#ifdef LCDCH_READ
	LCDCH_CTRLPORT &= ~LCDCH_READ; // Write mode
#endif
	LCDCH_CTRLPORT = (LCDCH_CTRLPORT & ~(LCDCH_DATA | LCDCH_E)) | (data & LCDCH_DATA); // Set RS pin
	LCDCH_DATAPORT = (LCDCH_DATAPORT & ~LCDCH_DATAMASK) | (data & LCDCH_DATAMASK);
	LCDCH_CTRLPORT |= LCDCH_E; // Strobe up
	_delay_us(LCDCH_TIMESTROBE);
	LCDCH_CTRLPORT &= ~LCDCH_E; // Strobe down
#endif
}

#ifdef LCDCH_READ
#ifdef LCDCH_I2C_INTERFACE
uint8_t LCDCH_Read4b(uint8_t data) // LCDCH_DATA flag for data read otherwise command, hi nibble: +LCDCH_READ
{
	I2C_Start(LCDCH_I2C_Addr + I2C_WRITE);
	LCDCH_I2C_Write(data | LCDCH_DATAMASK | LCDCH_READ);
	LCDCH_I2C_Write(data | LCDCH_DATAMASK | LCDCH_READ | LCDCH_E); // Strobe up
	I2C_Stop();
	I2C_Start(LCDCH_I2C_Addr + I2C_READ);
	uint8_t readed = I2C_Read(I2C_NOACK) & LCDCH_DATAMASK;
	I2C_Stop();
	I2C_Start(LCDCH_I2C_Addr + I2C_WRITE);
	LCDCH_I2C_Write(data | LCDCH_READ);
	if((data & LCDCH_READ) == 0) LCDCH_I2C_Write(0); // Strobe down
	I2C_Stop();
	return readed;
}
#else
uint8_t LCDCH_Read4b(const uint8_t data) // Read data flag
{
	LCDCH_DATAPORTDDR &= ~LCDCH_DATAMASK; // IN
	LCDCH_CTRLPORT = (LCDCH_CTRLPORT & ~(LCDCH_DATA | LCDCH_E)) | LCDCH_READ | data; // Set RW, RS pin
	LCDCH_CTRLPORT |= LCDCH_E; // Strobe up
	_delay_us(LCDCH_TIMESTROBE);
	uint8_t readed = LCDCH_DATAPORTIN & LCDCH_DATAMASK;
	LCDCH_CTRLPORT &= ~LCDCH_E; // Strobe down
	if((data & LCDCH_READ) == 0) LCDCH_CTRLPORT &= ~LCDCH_READ;
	return readed;
}
#endif

uint8_t LCDCH_ReadByte(const uint8_t what) // command or data
{
	return (LCDCH_Read4b(what | LCDCH_READ) << (4 - LCDCH_DATASHIFT)) | (LCDCH_Read4b(what) >> LCDCH_DATASHIFT);
}
#endif

uint8_t LCDCH_WaitWhileBusy(void)
{
#ifdef LCDCH_READ
	uint8_t i = 0;
	while(LCDCH_ReadByte(LCDCH_COMMAND) & LCDCH_FLAGBUSY)
	{
		Delay100us(1);
		if((--i) == 0) return 1;
	}
	return 0;
#else
	Delay100us(1);
	return 0;
#endif
}

void LCDCH_WriteByte(const uint8_t data)
{
	if(LCDCH_WaitWhileBusy() == 0) {
		LCDCH_Write4b(((data & 0xF0) >> (4 - LCDCH_DATASHIFT)) | LCDCH_DATA);
		LCDCH_Write4b(((data & 0x0F) << LCDCH_DATASHIFT) | LCDCH_DATA);
	}
}

uint8_t LCDCH_WriteCommand(const uint8_t data)
{
	if(LCDCH_WaitWhileBusy()) return 1;
	LCDCH_Write4b((data & 0xF0) >> (4 - LCDCH_DATASHIFT));
	LCDCH_Write4b((data & 0x0F) << LCDCH_DATASHIFT);
	return 0;
}

void LCDCH_LoadCharacterPGM(const uint8_t code, const uint8_t *chardata)
{
	LCDCH_WaitWhileBusy();
	LCDCH_WriteCommand(LCDCH__SetCharDataPos | code * 8);
	for (register uint8_t i = 0; i <= 7; i++){
		LCDCH_WaitWhileBusy();
		LCDCH_WriteByte(pgm_read_byte(chardata[i]));
	}
}

void LCDCH_SetCursor(const uint8_t y, const uint8_t x)  // y = 1..2, x = 1..16
{
	LCDCH_WaitWhileBusy();
	LCDCH_WriteCommand(LCDCH__SetCursorPos | ((y-1) * 0x40 + x-1));
}

void LCDCH_ClearDisplay(void)
{
	LCDCH_WriteCommand(LCDCH__ClearDisplay);
	#ifndef LCDCH_RW
	Delay100us(20);
	#endif
}

uint8_t LCDCH_Init(const uint8_t setup)  // Return 0 if Ok, setup: 0b1*** - 2 line, 0b*1** - second font table
{
#ifdef LCDCH_I2C_INTERFACE
	LCDCH_NotResponding = 0;
	I2C_Init();
#else
	#ifdef LCDCH_READ
	LCDCH_CTRLPORTDDR |= LCDCH_DATA | LCDCH_E | LCDCH_READ; // Out
	#else
	LCDCH_CTRLPORTDDR |= LCDCH_DATA | LCDCH_E; // Out
	#endif
#endif
	Delay100us(200); // >15ms
	LCDCH_Write4b(0b0011 << LCDCH_DATASHIFT);
#ifdef LCDCH_I2C_INTERFACE	
	if(LCDCH_NotResponding) return 2;
#endif
	Delay100us(41); // 4.1 ms
	LCDCH_Write4b(0b0011 << LCDCH_DATASHIFT);
	Delay100us(1); // 100us
	LCDCH_Write4b(0b0011 << LCDCH_DATASHIFT);
	Delay100us(1); // 100us
	LCDCH_Write4b(0b0010 << LCDCH_DATASHIFT); // 4 bit
	if(LCDCH_WriteCommand(0b00100000 | setup)) return 1; // 2 line + 5x8 font. (set bits -> 0b00101000), for 5x10 font = 0b00101100
	LCDCH_ClearDisplay();
	if(LCDCH_WriteCommand(LCDCH__EntryMode | LCDCH__EntryModeInc)) return 1;
	if(LCDCH_WriteCommand(LCDCH__Display | LCDCH__DisplayOn)) return 1;
	return 0;
}

void LCDCH_WriteString(const char *str) // Print string from PGM at cursor pos
{
	char c;
	while ((c = *str++)) LCDCH_WriteByte(c);
}

void LCDCH_WriteStringPGM(const char *str) // Print string from PGM at cursor pos
{
	char c;
	while ((c = pgm_read_byte(str++))) LCDCH_WriteByte(c);
}

char buffer[12]; // must be insufficient space!
// Write to buffer, number, decimal point position, number of character for formating (negative spacing -> leading '0')
void LCDCH_FormatNumber(uint32_t num, const int8_t dec, int8_t spacing)
{
	char c = 0;
	//uint8_t sign;
	//if ((sign = n) < 0) n = -n;
	int8_t i = 0;
	do { // in reverse order
		buffer[i++] = (c = '0') + num % 10;
		if(i == dec) buffer[i++] = (c = '.');
	} while ((num /= 10) > 0);
	//if (sign < 0) buffer[i++] = '-';
	uint8_t s;
	if(spacing < 0) {
		spacing = -spacing;
		s = '0';
	} else s = ' ';
	while(i < spacing) { 
		if(c == '.' || i < dec) c = '0'; else c = s;
		buffer[i++] = c;
		if(i == dec) buffer[i++] = (c = '.');
	}
	buffer[i] = '\0';
	for (uint8_t j = i - 1, i = 0; i<j; i++, j--) { // reversing
		c = buffer[i];
		buffer[i] = buffer[j];
		buffer[j] = c;
	}
	//i = 0;
	//str[i++] = '0' + num / 10;
	//if(dec > 0) str[i++] = '.';
	//str[i++] = '0' + num % 10;
	//if(dec < 0) str[i++] = '.';
	//str[i] = '\0';
}

void LCDCH_WriteNumberHex(const uint8_t num)
{
	static const char ascii_HEX[] PROGMEM = { "0123456789ABCDEF" };

	LCDCH_WriteByte(pgm_read_byte(&ascii_HEX[(num & 0xF0) >> 4]));
	LCDCH_WriteByte(pgm_read_byte(&ascii_HEX[num & 0x0F]));
}