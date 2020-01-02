/*
 * Counter.c
 *
 * Created: 14.01.2014 14:00:10
 *  Author: Vadim Kulakov, vad7@yahoo.com
 *
 * ATtiny84A
 */ 
// Fuses: BODLEVEL = 1V8
#define F_CPU	8000000UL
//#define DEBUG_IN_PROTEUS // for Release remark this string

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/atomic.h>

#include "I2C.h"
#include "LCDHD44780.h"

#define StopSwitch				(1<<PORTA3)
#define StopSwitch_PRESSING		!(PINA & StopSwitch)

#define ADC_LDR					(1<<MUX1) // ADC2

#define Sensor_LCDPowerOff		!(PINA & (1<<PORTA4))

#define KEY_IN					PINB
#define KEY_LEFT				(1<<PORTB0)
#define KEY_LEFT_PRESSING		!(KEY_IN & KEY_LEFT)
#define KEY_OK					(1<<PORTB1)
#define KEY_OK_PRESSING			!(KEY_IN & KEY_OK)
#define KEY_RIGHT				(1<<PORTB2)
#define KEY_RIGHT_PRESSING		!(KEY_IN & KEY_RIGHT)
volatile uint8_t KeysPressed = 0;
uint8_t KeysPressedTimeOut = 0;
#define KEY_LONG_PRESSING_TIME	12 // *0.125 sec, entering setup = *2
volatile uint8_t Timer = 120;	// used for timeouts: Setup exit, Turn display off if low light
#define TIMER_DEFAULT			255 // *0.125 sec
volatile uint8_t Counter;
uint8_t DisplayNotRespondingTimeout = 0;

// For counters is used: ADC0, ADC1 
uint16_t Cnt1;
uint8_t  Cnt1PreviousState = 0;
uint8_t  Cnt1StopTimeBack = 8; // 1 sec after power on
uint16_t Cnt2;
uint8_t  Cnt2PreviousState = 0;
uint8_t  Cnt2StopTimeBack = 8; // 1 sec after power on
uint8_t  CntLevelThreshold;
uint8_t  PreviousPos;
uint8_t	 ADC_Selector = 0;					//  0..1 - Cnt1..2, 2 - LDR1
uint8_t  CntStopped = 0;
uint8_t  CntStopTimeBackValue;
uint8_t  StopSwitchOffTimer = 0;
uint8_t  DisplayPreviousBy = 0;				// 1 - Show total, 4 - only delta by 4
int8_t   DisplayRefresh = 1;				// 1 - need refresh, -1 - turned off, other value - timeout in 0.125sec
uint8_t  LowLightThreshold;
volatile uint8_t LowLight = 0;

#define EPROM_OSCCAL				0x00
#define EPROM_LowLightThreshold		0x01 // 0x60, Light ON threshold
#define EPROM_CntStopTimeBack		0x02 // 20, *0.125sec
#define EPROM_DisplayOnTimeForced	0x03 // 120 (15 sec), *0.125

#define EPROM_CntLevelThreshold		0x06 // 23, Threshold between 0 - 1 state
#define EPROM_Cnt1Name				0x07 // H (Hot)
#define EPROM_Cnt2Name				0x08 // C (Cold)
#define EPROM_Cnt1Start				0x09 // uint 24bit
#define EPROM_Cnt2Start				0x0C // uint 24bit
#define EPROM_CntStoredIndex		0x0F // 0..CntStoredArrayLen-1
#define EPROM_CntStoredArray		0x10 // deltas (16bit) from previous Cnt1, Cnt2. Array must be initialized with 0 !
#define EPROM_CntStoredArrayEnd		0x1FF
#define CntStoredSize				2
#define CntTotal					2
#define CntStoredArrayLen			(EPROM_CntStoredArrayEnd + 1 - EPROM_CntStoredArray) / (CntTotal * CntStoredSize) // 124
#define CalcCntMemoryPos(idx, cntnum) EPROM_CntStoredArray + (idx * CntTotal + cntnum) * CntStoredSize

////////////////////////////////////////0123456789ABCDEF
const char SetupMenuItem_1[] PROGMEM = "1. Exit";
const char SetupMenuItem_2[] PROGMEM = "2. Change values";
const char SetupMenuItem_3[] PROGMEM = "3. Rollback";
const char SetupMenuItem_4[] PROGMEM = "4. New counter";
const char SetupMenuItem_5[] PROGMEM = "5. Set dark lev";
const char SetupMenuItem_6[] PROGMEM = "6. Modify EEPROM";
const char SetupMenuItem_7[] PROGMEM = "7. Set OSCCAL";
const char SetupMenuItem_8[] PROGMEM = "8. Show ADC sens";
const char* const SetupMenuItems[] PROGMEM = { SetupMenuItem_1, SetupMenuItem_2, SetupMenuItem_3, SetupMenuItem_4, SetupMenuItem_5, SetupMenuItem_6, SetupMenuItem_7, SetupMenuItem_8 };
const char String_SaveOk[] PROGMEM = "Saved OK!";
const char String_Spaces[] PROGMEM = "      ";

void Delay10us(uint8_t ms) {
	while(ms-- > 0) _delay_us(10); //wdt_reset();
}
void Delay1ms(uint8_t ms) {
	while(ms-- > 0) {
		_delay_ms(1); //wdt_reset();
	}
}
void Delay100ms(unsigned int ms) {
	while(ms-- > 0) {
		_delay_ms(100); //wdt_reset();
	}
}

uint8_t EEPROM_read(uint16_t ucAddress) // ATtiny84A only!
{
	while(EECR & (1<<EEPE)) ; // EEWE
	EEAR = ucAddress;
	EECR |= (1<<EERE);
	return EEDR;
}
void EEPROM_write(uint16_t ucAddress, uint8_t ucData) // ATtiny84A only!
{
	while(EECR & (1<<EEPE)) ; // EEWE
	cli();
	EECR = (0<<EEPM1)|(0<<EEPM0);
	EEAR = ucAddress;
	EEDR = ucData;
	EECR |= (1<<EEMPE); //(1<<EEMWE);
	EECR |= (1<<EEPE); //(1<<EEWE);
	sei();
}

ISR(PCINT1_vect)
{
	KeysPressedTimeOut = 1; // Skip keys ripple
}

#define SETUP_WATCHDOG WDTCSR = (1<<WDCE) | (1<<WDE); WDTCSR = (1<<WDE) | (1<<WDIE) | (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (1<<WDP0) //  Watchdog 0.125 s
#ifdef DEBUG_IN_PROTEUS
ISR(TIM1_COMPA_vect) {
#else
ISR(WATCHDOG_vect) {
	SETUP_WATCHDOG;
#endif
	Counter++;
	if(Timer) Timer--;
	if(Cnt1StopTimeBack) Cnt1StopTimeBack--;
	if(Cnt2StopTimeBack) Cnt2StopTimeBack--;
	if(StopSwitchOffTimer) if(--StopSwitchOffTimer == 0) { 
		CntStopped = 0;
		if(DisplayRefresh == 0) DisplayRefresh = 1;
	}
	if(DisplayRefresh > 1) DisplayRefresh--;
	if(DisplayNotRespondingTimeout && !Sensor_LCDPowerOff) DisplayNotRespondingTimeout--;
	if(KeysPressedTimeOut && --KeysPressedTimeOut == 0) {
		KeysPressed = (KEY_IN & (KEY_LEFT | KEY_RIGHT | KEY_OK)) ^ (KEY_LEFT | KEY_RIGHT | KEY_OK);
	}
	ADCSRA |= (1<<ADEN); // ADC on
	ADCSRA |= (1<<ADSC); // ADC Start conversion
}

ISR(ADC_vect) // Light sensor
{
	// 12 - no pulse, 35 - pulse (change every 0.010 m3)
	uint8_t adc = ADCH;
	if(ADC_Selector == ADC_LDR) { // Light Depending Resistor (last ADC port)
		LowLight = adc >= LowLightThreshold;
		ADC_Selector = 0;
	} else { 
		adc = adc < CntLevelThreshold;
		if(ADC_Selector == 0) {
			if(adc != Cnt1PreviousState) {
				if(Cnt1PreviousState != 0xFF && CntStopped == 0) { // skip first time
					if(Cnt1StopTimeBack) {
						Cnt1StopTimeBack = 0; // skip wrong pulse
						Cnt1--;
					} else if(adc == 0) { // falling edge assumes switching 
						Cnt1++;
						Cnt1StopTimeBack = CntStopTimeBackValue;
					}
					if(DisplayRefresh == 0) DisplayRefresh = 1;
				}
				Cnt1PreviousState = adc;
			}
		} else {
			if(adc != Cnt2PreviousState) {
				if(Cnt2PreviousState != 0xFF && CntStopped == 0) { // skip first time
					if(Cnt2StopTimeBack) {
						Cnt2StopTimeBack = 0; // skip wrong pulse
						Cnt2--;
					} else if(adc == 0) {
						Cnt2++;
						Cnt2StopTimeBack = CntStopTimeBackValue;
					}
					if(DisplayRefresh == 0) DisplayRefresh = 1;
				}
				Cnt2PreviousState = adc;
			}
		}
		ADC_Selector += 1; // next ADC port
	}
	ADCSRA &= ~(1<<ADEN); // ADC off
	ADMUX = ADC_Selector;
	if(ADC_Selector) {
		ADCSRA |= (1<<ADEN); // ADC on
		ADCSRA |= (1<<ADSC); // ADC Start conversion
	}
}

ISR(PCINT0_vect)
{
	if(StopSwitch_PRESSING) {
		if(Cnt1StopTimeBack) {
			Cnt1StopTimeBack = 0;
			Cnt1--;
		}
		if(Cnt2StopTimeBack) {
			Cnt2StopTimeBack = 0;
			Cnt2--; 
		}
		CntStopped = 1;
		StopSwitchOffTimer = 0;
		if(DisplayRefresh == 0) DisplayRefresh = 1;
	} else if(CntStopped) {
		StopSwitchOffTimer = CntStopTimeBackValue;
	}
	if(Sensor_LCDPowerOff) {
		LCDCH_NotResponding = 1;
		DisplayNotRespondingTimeout = 80; // 10 sec
	}
}

uint8_t ReadLastADC(uint8_t adc)
{
	do { __asm__ volatile ("" ::: "memory"); sleep_cpu(); } while(ADC_Selector != adc);
	do { __asm__ volatile ("" ::: "memory"); sleep_cpu(); } while(ADC_Selector == adc);
	return ADCH;
}

uint16_t GetCurrentStorePos(void)
{
	return EPROM_CntStoredArray + EEPROM_read(EPROM_CntStoredIndex) * CntTotal * CntStoredSize;
}

int32_t ReadTotalCntPrevious(uint8_t cntnum) // skip current delta
{
	uint16_t i = EPROM_Cnt1Start;
	uint16_t storepos = GetCurrentStorePos();
	if(cntnum) {
		i += EPROM_Cnt2Start - EPROM_Cnt1Start;
		storepos += CntStoredSize;
	}
	int32_t cnt; cnt = EEPROM_read(i) + EEPROM_read(i + 1) * 256L + EEPROM_read(i + 2) * 65536L;
	for(i = CalcCntMemoryPos(0, cntnum); i <= EPROM_CntStoredArrayEnd; i += CntTotal * CntStoredSize) {
		if(i == storepos) continue;
		cnt += EEPROM_read(i) + EEPROM_read(i + 1) * 256L;
	}
	return cnt;
}

void PrintHeader(uint8_t cntnum)
{
	LCDCH_WriteByte(EEPROM_read(EPROM_Cnt1Name + cntnum));
}

uint8_t ArrayIndexSub(uint8_t index, uint8_t dec)
{
	uint8_t newidx;
	if(index < dec) newidx = CntStoredArrayLen + index; else newidx = index;
	newidx -= dec;
	uint8_t cur = EEPROM_read(EPROM_CntStoredIndex);
	return index > cur && newidx <= cur ? index : newidx;
}

// if index == 255 then print Cnt1 or Cnt2
void PrintPreviousValue(uint8_t index, uint8_t cntnum)
{
	uint16_t tmp;
	if(index == 255) {
		ATOMIC_BLOCK(ATOMIC_FORCEON) tmp = cntnum == 0 ? Cnt1 : Cnt2;
	} else {
		tmp = CalcCntMemoryPos(index, cntnum);
		tmp = EEPROM_read(tmp) + EEPROM_read(tmp + 1) * 256;
	}
	LCDCH_FormatNumber(tmp, 2, 5);
	if(buffer[2] == '.') {
		LCDCH_WriteByte(buffer[0]);
		LCDCH_WriteByte(buffer[1] == ' ' ? '0' : buffer[1]);
	} else LCDCH_WriteStringPGM(PSTR("##")); // overflow ( > 99.99)
	LCDCH_WriteByte(';');
}

void PrintPreviousValues(uint8_t cntnum)
{
	uint8_t index = PreviousPos;
	LCDCH_SetCursor(1 + cntnum, 4);	
	PrintHeader(cntnum);
	LCDCH_WriteByte('\x7E'); // '->'
	for(uint8_t i = 0; i < 4; i++) {
		PrintPreviousValue(index, cntnum);
		index = ArrayIndexSub(index, 1);
	}
}

void PrintPreviousTotalValue(uint8_t cntnum)
{
	LCDCH_SetCursor(1 + cntnum, 4);
	PrintHeader(cntnum);
	LCDCH_WriteByte(':');
	uint32_t num = ReadTotalCntPrevious(cntnum);
	uint8_t poscurr = EEPROM_read(EPROM_CntStoredIndex);
	uint16_t idx = PreviousPos;
	while(1) {
		if(++idx >= CntStoredArrayLen - 1) idx = 0;
		if(idx == poscurr) break;
		uint16_t tmp = CalcCntMemoryPos(idx, cntnum);
		num -= EEPROM_read(tmp) + EEPROM_read(tmp + 1) * 256L;
	}
	LCDCH_FormatNumber(num, 2, 8);
	LCDCH_WriteString(buffer);
	LCDCH_WriteByte('\x7E'); // '->'
	PrintPreviousValue(PreviousPos, cntnum);
}

void PrintCnt(uint8_t cntnum)
{
	if(LCDCH_NotResponding) return;
	LCDCH_SetCursor(1 + cntnum, 1);
	PrintHeader(cntnum);
	if(CntStopped) LCDCH_WriteByte('*'); else LCDCH_WriteByte(':');
	int32_t num = ReadTotalCntPrevious(cntnum);
	ATOMIC_BLOCK(ATOMIC_FORCEON) num += cntnum == 0 ? Cnt1 : Cnt2;
	LCDCH_FormatNumber(num, 2, 8);
	LCDCH_WriteString(buffer);
	LCDCH_WriteByte('\x7E'); // '->'
	PrintPreviousValue(255, cntnum);
	PrintPreviousValue(ArrayIndexSub(EEPROM_read(EPROM_CntStoredIndex), 1), cntnum);
}

void SaveCnts(uint16_t storepos)
{
	typeof (Cnt1) tmp;
	ATOMIC_BLOCK(ATOMIC_FORCEON) tmp = Cnt1;
	if(EEPROM_read(storepos) != tmp % 256) EEPROM_write(storepos, tmp % 256);
	if(EEPROM_read(storepos + 1) != tmp / 256) EEPROM_write(storepos + 1, tmp / 256);
	ATOMIC_BLOCK(ATOMIC_FORCEON) tmp = Cnt2;
	if(EEPROM_read(storepos + 2) != tmp % 256) EEPROM_write(storepos + 2, tmp % 256); 
	if(EEPROM_read(storepos + 3) != tmp / 256) EEPROM_write(storepos + 3, tmp / 256);
}

void CheckAddToCntStart(int32_t num, uint8_t CntStart)
{
	if(num != 0) { // Non zero - add it to Start
		num += EEPROM_read(CntStart) + EEPROM_read(CntStart + 1) * 256L + EEPROM_read(CntStart + 2) * 65536L;
		EEPROM_write(CntStart, num % 256L); 
		EEPROM_write(CntStart + 1, num % 65536L / 256L);
		EEPROM_write(CntStart + 2, num / 65536L);
	}
}

// return EEPROM address of Cnt1
uint16_t ReadCnts(void)
{
	uint16_t storepos = GetCurrentStorePos();
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		Cnt1 = EEPROM_read(storepos) + EEPROM_read(storepos + 1) * 256;
		Cnt2 = EEPROM_read(storepos + 2) + EEPROM_read(storepos + 3) * 256;
	}
	return storepos;
}

void PrintSpaces(void)
{
	LCDCH_WriteStringPGM(String_Spaces);
}

void PrintSaveOk(void)
{
	LCDCH_ClearDisplay();
	LCDCH_WriteStringPGM(String_SaveOk);
	Delay100ms(20);
}

uint8_t WaitKeyOk(void)
{
	while(KEY_OK_PRESSING) ;
	Delay100ms(3);
	KeysPressed = 0;
	Timer = TIMER_DEFAULT;
	do { sleep_cpu(); } while(KeysPressed == 0 && Timer != 0);
	return KeysPressed == KEY_OK;
}

uint8_t AskQuestion(void)
{
	uint8_t i = 0;
	goto xShowQuestion;
	do {
		sleep_cpu(); 
		if(KeysPressed) {
			if(KeysPressed == KEY_OK) {
				break;
			} else {
				i ^= 1;
xShowQuestion:
				LCDCH_SetCursor(2, 1);
				PrintSpaces();
				LCDCH_WriteStringPGM(i ? PSTR("[YES]") : PSTR("[NO]"));
				PrintSpaces();
			}
			KeysPressed = 0;
		}
		if(Timer == 0) {
			i = 0;
			break;
		}
	} while(1);
	KeysPressed = 0;
	return i;
}

// After return: KeysPressed = last pressed key
// Keys: moving - left, right; short press Ok - change
// After long pressing OK will be show "OK?" and wait key, after that returning
int32_t EditBufferValue(uint8_t row, int8_t maxpos) // maxpos starting from 0
{
	int32_t tmp;
	int8_t pos = maxpos;
	LCDCH_WriteCommand(LCDCH__Display | LCDCH__DisplayOn | LCDCH__DisplayCursorOn | LCDCH__DisplayCursorBlink);
	goto xSetCursor;
	do { __asm__ volatile ("" ::: "memory"); // Need memory barrier
		sleep_cpu(); 
		if(KeysPressed == KEY_LEFT) {
			do { if(--pos < 0) pos = maxpos; } while(buffer[pos] == '.');
			goto xSetCursor;
		} else if(KeysPressed == KEY_RIGHT) {
			do { if(++pos > maxpos) pos = 0; } while(buffer[pos] == '.');
			goto xSetCursor;
		} else if(KeysPressed == KEY_OK) {
			Counter = 0;
			while(KEY_OK_PRESSING && Counter < KEY_LONG_PRESSING_TIME) ; 
			if(Counter >= KEY_LONG_PRESSING_TIME) {
				LCDCH_SetCursor(2, row + maxpos + 2);
				LCDCH_WriteStringPGM(PSTR("OK?"));
				WaitKeyOk();
				tmp = 0;
				for(pos = 0; pos <= maxpos; pos++) {
					uint8_t b = buffer[pos];
					if(b != '.') tmp = tmp * 10 + (b == ' ' ? 0 : (b - '0'));
				}
				LCDCH_WriteCommand(LCDCH__Display | LCDCH__DisplayOn);
				return tmp;
			} else { // Change digit
				buffer[pos]++;
				if(buffer[pos] > '9' || buffer[pos] < '0') buffer[pos] = '0';
				LCDCH_WriteByte(buffer[pos]);
			}
xSetCursor:	LCDCH_SetCursor(2, row + pos);
			Timer = TIMER_DEFAULT;
			KeysPressed = 0;
		}
	} while(Timer);
	KeysPressed = 0;
	return 0;
}

int main(void)
{
	uint8_t i;
	CLKPR = (1<<CLKPCE); CLKPR = (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0); // Clock prescaler division factor: 1
	MCUCR = (1<<SE) | (0<<SM1) | (0<<SM0); // Idle sleep enable
	//DDRA = ; // Out
	PORTA = StopSwitch | (1<<PORTA5) | (1<<PORTA2) | (1<<PORTA1) | (1<<PORTA0); // pullup: Cnt1, Cnt2, LDR1, not used pins
	PORTB = (1<<PORTB2) | (1<<PORTB1) | (1<<PORTB0); // pullup
	SETUP_WATCHDOG;
	GIMSK = (1<<PCIE1) | (1<<PCIE0); // Pin Change Interrupt Enable 0, 1
	PCMSK0 = (1<<PCINT4) | (1<<PCINT3); // Pin Change Mask Register 0 - reed switch, power sensor
	PCMSK1 = (1<<PCINT10) | (1<<PCINT9) | (1<<PCINT8); // Pin Change Mask Register 1 - Keys
	i = EEPROM_read(EPROM_OSCCAL);
	if(i == 0xFF) EEPROM_write(EPROM_OSCCAL, OSCCAL); else OSCCAL = i;
	CntStopTimeBackValue = EEPROM_read(EPROM_CntStopTimeBack);
	CntLevelThreshold = EEPROM_read(EPROM_CntLevelThreshold);
	LowLightThreshold = EEPROM_read(EPROM_LowLightThreshold);
	ReadCnts();
	if(StopSwitch_PRESSING) CntStopped = 1;
	//
#ifdef DEBUG_IN_PROTEUS
	// for debug in Proteus
	OCR1A = F_CPU / (4 * 2 * 64); // 0.125 sec; = F_CPU / (FREQ * 2 * Prescaller); min = 61Hz;  FREQ = F_CPU / (2 * Prescaller * (1 + OCR1A)); 
	TIMSK1 = (1<<OCIE1A); // overflow interrupt
	//DDRA |= (1<<PORTA5); TCCR1A = (1<<COM1B0); // FREQ OUT OC1B
	TCCR1B = (1<<WGM12) | (0 << CS12) | (1 << CS11) | (1 << CS10); // CTC mode, Timer0 prescaller: 64
#endif	
	//
	sei();
	LCDCH_Init(LCDCH_2LINES); // | LCDCH_SECOND_FONT));
#ifndef DEBUG_IN_PROTEUS	
	LCDCH_WriteStringPGM(PSTR("Counter of water"));
	LCDCH_SetCursor(2,1);
	LCDCH_WriteStringPGM(PSTR("v1.1 (c) 2014"));
	Delay100ms(20);
#endif
	// ADC
	ADMUX = (0<<REFS1) | (0<<MUX2)|(0<<MUX1)|(0<<MUX0); // ADC0 (PA0) - start from Cnt1
	ADCSRB = (1<<ADLAR) | (0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0); // ADC Left Adjust Result
	ADCSRA = (1<<ADEN) | (0<<ADSC) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (0<<ADPS0); // ADC Int, No Start, divider: 64 (125Khz)
    while(1)
    {	__asm__ volatile ("" ::: "memory"); // Need memory barrier
		sleep_cpu(); 
		if(LCDCH_NotResponding) {
			if(DisplayNotRespondingTimeout == 0) {
				if(LCDCH_Init(LCDCH_2LINES)) // | LCDCH_SECOND_FONT));
				{
					DisplayNotRespondingTimeout = 80; // 10 sec
				}
			}
			continue;
		}
		if(LowLight) {
			if(DisplayRefresh == 0 && Timer == 0) {
				LCDCH_Light = 0; // LED off
				LCDCH_WriteCommand(LCDCH__Display); // off
				DisplayRefresh = -1;
			}
		} else if(DisplayRefresh == 1 || DisplayRefresh == -1) {
xRefreshDisplayNow:
			if(DisplayRefresh == -1) {
				LCDCH_Light = LCDCH_LED; // LED on
				Timer = EEPROM_read(EPROM_DisplayOnTimeForced);
			}
			if(LCDCH_NotResponding) continue;
			LCDCH_WriteCommand(LCDCH__Display | LCDCH__DisplayOn); // on
			PrintCnt(0);
			PrintCnt(1);
			DisplayRefresh = 0;
			DisplayPreviousBy = 0;
		}
		if(KeysPressed & (KEY_LEFT | KEY_RIGHT)) {
			// Show previous values
			if(DisplayRefresh < 0) {
				goto xShowDisplayForSomeTime;
			}
			i = EEPROM_read(EPROM_CntStoredIndex);
			if(DisplayPreviousBy == 0) {
				PreviousPos = ArrayIndexSub(i, 1);
				if(KeysPressed == KEY_RIGHT) 
					DisplayPreviousBy = 4; 
				else 
					DisplayPreviousBy = 1; 
			}
			LCDCH_SetCursor(1, 1);
			i = PreviousPos > i ? CntStoredArrayLen - PreviousPos + i : i - PreviousPos;
			LCDCH_WriteByte(DisplayPreviousBy == 1 ? 'n' : 'p');
			LCDCH_WriteStringPGM(PSTR(" |"));
			LCDCH_SetCursor(2, 1);
			LCDCH_FormatNumber(i / DisplayPreviousBy, 0, i > 99 ? 3 : 2);
			LCDCH_WriteString(buffer);
			if(i <= 99) LCDCH_WriteByte('|');
			if(i / DisplayPreviousBy <= 9) {
				LCDCH_SetCursor(2, 1);
				LCDCH_WriteByte('-');
			}
			if(DisplayPreviousBy == 1) {	// Show previous delta values by 4
				PrintPreviousTotalValue(0);
				PrintPreviousTotalValue(1);
			} else {						// Show previous total values by 1
				PrintPreviousValues(0);
				PrintPreviousValues(1);
			}
			PreviousPos = ArrayIndexSub(PreviousPos, DisplayPreviousBy);
			KeysPressed = 0;
			DisplayRefresh = 100; // 12.5 sec
		} else if(KeysPressed & KEY_OK)	{
			if(DisplayRefresh == 0) {
				uint16_t storepos = GetCurrentStorePos();
				SaveCnts(storepos);
				if(LCDCH_NotResponding) continue;
				Counter = 0;
				while(KEY_OK_PRESSING && Counter < KEY_LONG_PRESSING_TIME * 2) ; 
				if(Counter >= KEY_LONG_PRESSING_TIME * 2) {
					// Setup
					uint8_t Setup, SetupItem = 0, i = 0;
					goto xShowSetupMenu;
					do { __asm__ volatile ("" ::: "memory"); // Need memory barrier
						sleep_cpu(); 
						if(KeysPressed == KEY_LEFT) {
							if(Setup) {
								if(SetupItem == 6 -1) {
									if(--storepos == 65535) storepos = EPROM_CntStoredArrayEnd;
									if(Setup == 2 && KEY_LEFT_PRESSING) {
										Delay100ms(1);
									} else {
										Counter = 0;
										while(KEY_LEFT_PRESSING && Counter < KEY_LONG_PRESSING_TIME) ; 
										if(Counter >= KEY_LONG_PRESSING_TIME) {
											Setup = 2; // fast scrolling
										} else Setup = 1; // normal scrolling
									}
									goto xSetupModifyEEPROM_Show;
								}
							} else {
								if(--SetupItem == 0xFF) SetupItem = sizeof(SetupMenuItems)/sizeof(SetupMenuItems[0]) - 1;
xShowSetupMenu:					Setup = 0;
								LCDCH_ClearDisplay();
								LCDCH_WriteStringPGM(PSTR("Setup:"));
								LCDCH_SetCursor(2,1);
								LCDCH_WriteStringPGM((const char *)pgm_read_word(&SetupMenuItems[SetupItem]));
							}
							Timer = TIMER_DEFAULT;
							KeysPressed = 0;
						} else if(KeysPressed == KEY_RIGHT) {
							if(Setup) {
								if(SetupItem == 6 -1) {
xSetupModifyEEPROM_Next:			if(++storepos >= EPROM_CntStoredArrayEnd) storepos = 0;
									if(Setup == 2 && KEY_RIGHT_PRESSING) {
										Delay100ms(1);
									} else {
										Counter = 0;
										while(KEY_RIGHT_PRESSING && Counter < KEY_LONG_PRESSING_TIME) ; 
										if(Counter >= KEY_LONG_PRESSING_TIME) {
											Setup = 2; // fast scrolling
										} else Setup = 1; // normal scrolling
									}
									goto xSetupModifyEEPROM_Show;
								}
							} else {
								if(++SetupItem >= sizeof(SetupMenuItems)/sizeof(SetupMenuItems[0])) SetupItem = 0;
								goto xShowSetupMenu;
							}
							Timer = TIMER_DEFAULT;
							KeysPressed = 0;
						} else if(KeysPressed == KEY_OK) {
							switch (SetupItem) {
							case 1 -1:
								goto xSetupExit;
							case 2 -1: // Change current values
								i = 0;
								goto xSetupChangeValues_Show;
								do {
									int32_t num, num2;
									sleep_cpu();
									if(KeysPressed == KEY_LEFT) {
										if(i == 0) break;
										i--;
										goto xSetupChangeValues_Show;
									} else if(KeysPressed == KEY_RIGHT) {
										if(++i > 1) i = 0;
xSetupChangeValues_Show:				LCDCH_ClearDisplay();
										LCDCH_WriteStringPGM(PSTR("Set total value"));
										LCDCH_SetCursor(2,1);
										LCDCH_WriteStringPGM(PSTR("Select: "));
										PrintHeader(i);
										Timer = TIMER_DEFAULT;
										KeysPressed = 0;
									} else if(KeysPressed == KEY_OK) {
										LCDCH_SetCursor(2,1);
										PrintHeader(i);
										LCDCH_WriteStringPGM(PSTR(": "));
										num = ReadTotalCntPrevious(i);
										ATOMIC_BLOCK(ATOMIC_FORCEON) num += i == 0 ? Cnt1 : Cnt2;
										LCDCH_FormatNumber(num, 2, -8);
										LCDCH_WriteString(buffer);
										num2 = EditBufferValue(4, 7) - num;
										if(KeysPressed == KEY_OK) {
											uint16_t tmp; tmp = i == 0 ? Cnt1 : Cnt2;
											if(num2 + tmp > 9999L || num2 + tmp < 0L) { // overflow
												LCDCH_ClearDisplay();
												LCDCH_WriteStringPGM(PSTR("Delta: "));
												if(num2 < 0) LCDCH_WriteByte('-');
												LCDCH_FormatNumber(num2 < 0 ? -num2 : num2, 2, 8);
												LCDCH_WriteString(buffer);
												LCDCH_SetCursor(2,1);
												LCDCH_WriteStringPGM(PSTR("Change StartNum?"));
												if(WaitKeyOk()) {
													CheckAddToCntStart(num2, i == 0 ? EPROM_Cnt1Start : EPROM_Cnt2Start);
													PrintSaveOk();
												}
											} else {
												tmp += (int16_t) num2;
												if(i == 0) Cnt1 = tmp; else Cnt2 = tmp;
												SaveCnts(storepos);
												PrintSaveOk();
											}
										}
										goto xSetupChangeValues_Show;
									}
								} while(Timer);
								goto xSetupExit;
							case 3 -1: // Roll back
								LCDCH_ClearDisplay();
								LCDCH_WriteStringPGM(PSTR("Rollback period?"));
								Timer = TIMER_DEFAULT / 3;
								if(AskQuestion()) {
									if(Cnt1 != 0 || Cnt2 != 0) {
										LCDCH_ClearDisplay();
										LCDCH_WriteStringPGM(PSTR("Counts are not 0"));
										Timer = TIMER_DEFAULT / 3;
										if(!AskQuestion()) goto xSetupExit;
									}
									if((i = EEPROM_read(EPROM_CntStoredIndex)) == 0) i = CntStoredArrayLen - 1; else i--;
									typeof (storepos) tmp = CalcCntMemoryPos(i, 0);
									ATOMIC_BLOCK(ATOMIC_FORCEON) {
										Cnt1 += EEPROM_read(tmp) + EEPROM_read(tmp + 1) * 256;
										Cnt2 += EEPROM_read(tmp + CntStoredSize) + EEPROM_read(tmp + CntStoredSize + 1) * 256;
										SaveCnts(tmp);
										EEPROM_write(EPROM_CntStoredIndex, i);
										Cnt1 = 0;
										Cnt2 = 0;
										SaveCnts(storepos);
										ReadCnts();
									}
									goto xShowSaveOkAndExit;
								}
								goto xSetupExit;
							case 4 -1: // New counter
								LCDCH_ClearDisplay();
								LCDCH_WriteStringPGM(PSTR("CLEAR ALL DATA:"));
								i = 0;
								goto xSetupNewCounter_Show;
								do {
									sleep_cpu(); 
									if(KeysPressed) {
										if(KeysPressed == KEY_OK) {
											LCDCH_SetCursor(1,16);
											PrintHeader(i);
											if(AskQuestion()) {
												ATOMIC_BLOCK(ATOMIC_FORCEON) if(i == 0) {
													Cnt1 = 0;
													storepos = EPROM_Cnt1Start;
												} else {
													Cnt2 = 0;
													storepos = EPROM_Cnt2Start;
												}
												EEPROM_write(storepos, 0);
												EEPROM_write(storepos + 1, 0);
												EEPROM_write(storepos + 2, 0);
												for(storepos = CalcCntMemoryPos(0, i); storepos <= EPROM_CntStoredArrayEnd; storepos += CntTotal * CntStoredSize) {
													EEPROM_write(storepos, 0);
													EEPROM_write(storepos + 1, 0);
												}
												goto xShowSaveOkAndExit;
											}
										} else {
											i ^= 1;
xSetupNewCounter_Show:						LCDCH_SetCursor(2,1);
											LCDCH_WriteStringPGM(PSTR("For Counter: "));
											PrintHeader(i);
										}
										KeysPressed = 0;
										Timer = TIMER_DEFAULT;
									}
								} while(Timer);
								goto xSetupExit;
							case 5 -1: // Set dark level
								LCDCH_ClearDisplay();
								LCDCH_WriteStringPGM(PSTR("Set dark level"));
								for(i = 5; i > 0; i--) {
									LCDCH_SetCursor(2,1);
									LCDCH_WriteStringPGM(PSTR("after "));
									LCDCH_WriteByte('0' + i);
									Delay100ms(10);
								}
								EEPROM_write(EPROM_LowLightThreshold, ReadLastADC(ADC_LDR) - 3);
								goto xShowSaveOkAndExit;
							case 6 -1: // Modify EEPROM
								if(Setup) {
									Counter = 0;
									while(KEY_OK_PRESSING && Counter < KEY_LONG_PRESSING_TIME) ; 
									if(Counter >= KEY_LONG_PRESSING_TIME) { // Exiting
										LCDCH_ClearDisplay();
										LCDCH_WriteStringPGM(PSTR("OK - RESET?"));
										if(WaitKeyOk()) {
											cli(); // disable interrupts
											while(1) ; // wait for watchdog reset
										}
										goto xSetupExit;
									}
									i = EditBufferValue(7, 2);
									if(KeysPressed == KEY_OK) {
										EEPROM_write(storepos, i);
										goto xSetupModifyEEPROM_Next;
									}
									goto xSetupModifyEEPROM_Show;
								} else {
									LCDCH_ClearDisplay();
									Setup = 1;
									LCDCH_WriteStringPGM(PSTR("Modify EEPROM:"));
									storepos = 1;
xSetupModifyEEPROM_Show:			i = EEPROM_read(storepos);
								}
								LCDCH_SetCursor(2,1);
								LCDCH_WriteByte('#');
								LCDCH_FormatNumber(storepos, 0, -3);
								LCDCH_WriteString(buffer);
								LCDCH_WriteStringPGM(PSTR(": "));
								LCDCH_FormatNumber(i, 0, -3);
								LCDCH_WriteString(buffer);
								PrintSpaces();
								LCDCH_SetCursor(2, 4);
								LCDCH_WriteCommand(LCDCH__Display | LCDCH__DisplayOn | LCDCH__DisplayCursorOn);
								if(Setup == 2) { // fast scrolling
									Timer = TIMER_DEFAULT;
									continue;
								}
								break;
							case 7 -1: // Set OSCCAL. Freq out 10000 Hz -> OC1B (PORTA5).
								LCDCH_ClearDisplay();
								LCDCH_WriteStringPGM(PSTR("Pin.8 out: 10kHz"));
								OCR1A = F_CPU / (10000 * 2 * 1); // = F_CPU / (FREQ * 2 * Prescaller)
								TCCR1A = (1<<COM1B0);
								TCCR1B = (1<<WGM12) | (0 << CS12) | (0 << CS11) | (1 << CS10); // CTC mode, Timer0 prescaller: 1
								i = DDRA & (1<<PORTA5);
								DDRA |= (1<<PORTA5); // out
								goto xSetupOSCCAL_Show;
								do {
									sleep_cpu();
									if(KeysPressed) {
										if(KeysPressed == KEY_RIGHT) {
											OSCCAL++;
											} else if(KeysPressed == KEY_LEFT) {
											OSCCAL--;
											} else {
											EEPROM_write(EPROM_OSCCAL, OSCCAL);
											break;
										}
xSetupOSCCAL_Show:						LCDCH_SetCursor(2,1);
										LCDCH_WriteStringPGM(PSTR("OSCCAL = "));
										LCDCH_FormatNumber(OSCCAL, 0, 3);
										LCDCH_WriteString(buffer);
										KeysPressed = 0;
										Timer = TIMER_DEFAULT;
									}
								} while(Timer);
								//TCCR1A = ; // restore timer
								//TCCR1B = ;
								//OCR1A = ;
								if(i == 0) DDRA &= ~(1<<PORTA5);
								goto xSetupExit;
							case 8 -1: // Show ADC sensors
								if(Setup == 1) goto xSetupExit; // exit item
								Setup = 1;
								LCDCH_ClearDisplay();
								KeysPressed = 0;
xSetupShowADC_Show:				LCDCH_SetCursor(1,1);
								if(LCDCH_NotResponding) goto xSetupExit;
								for(i = 0; i < 2; i++) {
									PrintHeader(i);
									if(i == 0 ? Cnt1StopTimeBack : Cnt2StopTimeBack) {
										LCDCH_FormatNumber((i == 0 ? Cnt1StopTimeBack : Cnt2StopTimeBack) / 4, 0, 2);
										LCDCH_WriteString(buffer);
									} else LCDCH_WriteStringPGM(PSTR("  "));
									LCDCH_WriteByte(':');
									LCDCH_FormatNumber(ReadLastADC(i), 0, 3); // ADC_i
									LCDCH_WriteString(buffer);
									LCDCH_WriteStringPGM(PSTR("  "));
								}
								LCDCH_SetCursor(2,1);
								LCDCH_WriteByte(CntStopped ? '*' : ' ');
								LCDCH_WriteStringPGM(PSTR("LDR: "));
								LCDCH_FormatNumber(ReadLastADC(ADC_LDR), 0, 3); // Light sensor
								LCDCH_WriteString(buffer);
								LCDCH_WriteStringPGM(LowLight ? PSTR(" - LOW") : String_Spaces);
								Delay100ms(1);
								Timer = TIMER_DEFAULT;
								continue;
							}
							KeysPressed = 0;
							Timer = TIMER_DEFAULT;
						} else if(Setup == 1 && SetupItem == 8 -1) goto xSetupShowADC_Show;
					} while(Timer);
xSetupExit:			goto xShowDisplayForSomeTime;
				}
				// Save values to EEPROM
				LCDCH_ClearDisplay();
				LCDCH_WriteStringPGM(PSTR("  Next period?"));
				Timer = 40; // 5 sec
				if(AskQuestion()) {
					LCDCH_ClearDisplay();
					LCDCH_WriteStringPGM(PSTR("Save values, OK?"));
					LCDCH_SetCursor(2,1);
					PrintHeader(0);
					LCDCH_WriteByte('\x7E'); // '->'
					ATOMIC_BLOCK(ATOMIC_FORCEON) LCDCH_FormatNumber(Cnt1, 2, 5);
					LCDCH_WriteString(buffer);
					LCDCH_WriteStringPGM(PSTR(", "));
					PrintHeader(1);
					LCDCH_WriteByte('\x7E'); // '->'
					ATOMIC_BLOCK(ATOMIC_FORCEON) LCDCH_FormatNumber(Cnt2, 2, 5);
					LCDCH_WriteString(buffer);
					if(WaitKeyOk()) { // Save
						ATOMIC_BLOCK(ATOMIC_FORCEON) {
							SaveCnts(storepos);
							if((i = EEPROM_read(EPROM_CntStoredIndex) + 1) >= CntStoredArrayLen) i = 0;
							EEPROM_write(EPROM_CntStoredIndex, i);
							storepos = ReadCnts();
							CheckAddToCntStart(Cnt1, EPROM_Cnt1Start);
							CheckAddToCntStart(Cnt2, EPROM_Cnt2Start);
							Cnt1 = 0;
							Cnt2 = 0;
							SaveCnts(storepos);
						}
xShowSaveOkAndExit:		PrintSaveOk();
						goto xShowDisplayForSomeTime;
					}
				}
			}
xShowDisplayForSomeTime:
			KeysPressed = 0;
			Timer = TIMER_DEFAULT;
			goto xRefreshDisplayNow;
		}
    }
}

