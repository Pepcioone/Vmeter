/* main.cpp
 *  Created on: Feb 1, 2015
 *      Author: pepcio v3.0
*/

/* USBasp
   G * -----] MISO (PB4-18)
   N * |    | SCK  (PB5-19)
   D * |      RST  (PC6-1)
   GND |    | GND  (GND-8,22)
   VCC -----| MOSI (PB3-17)
*/

#define F_CPU 1000000UL
#include <avr/iom8.h>			### Sprawdzic czy dobty include i poprawic w properties/make gcc-avr
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#define tau0 131   //
#define vliczt0 31 //przerwanie 0.25sek
#define tklaw 1	//opoznienie przerwan
#define t1wire 3
#define tlcd 4
#define tled 4
#define tadc 2
#define ti2c 1
#define sbuf 16 //bufor 16 bajtow

#define tabl_znakow         \
	{                       \
		{                   \
			0b00011000,     \
				0b00011100, \
				0b00000000, \
				0b00000111, \
				0b00001000, \
				0b00001000, \
				0b00000111, \
				0b00000000  \
		}                   \
	}

EEMEM char tekst[] = {0x00,		  //eeprom adres 0, staramy się nie zapisywać, ponieważ przy zaniku napięcia może ulec nadpisaniu
					  0b00011000, //st.C = 0
					  0b00011000,
					  0b00000000,
					  0b00000111,
					  0b00001000,
					  0b00001000,
					  0b00000111,
					  0b00000000,
					  0b00011011, //mA = 1
					  0b00010101,
					  0b00000000,
					  0b00001110,
					  0b00010001,
					  0b00011111,
					  0b00010001,
					  0b00000000,
					  0b00011011, //mV = 2
					  0b00010101,
					  0b00000000,
					  0b00010001,
					  0b00010001,
					  0b00001010,
					  0b00000100,
					  0b00000000,
					  0b00011011, //mW = 3
					  0b00010101,
					  0b00000000,
					  0b00010001,
					  0b00010101,
					  0b00011011,
					  0b00010001,
					  0b00000000};

// ----------------------------------------------
// ----------- DEKLARACJE -----------------------
// ----------------------------------------------
inline void LCD_RSlow();
inline void LCD_RShigh();
inline void LCD_Elow();
inline void LCD_Ehigh();

void LCD_init();
void LCD_on();
void LCD_trigger();
void LCD_pisz(char instrukcja, char data);
void LCD_defchar(char tablica[][8], char numer, char ile);
void LCD_cls();
void LCD_wk(char w, char k);
char LCD_txt(char tekst[]);

void Wyswietl_pomiar(float V, float A, float W, float C);
int FloatToASCII(char fAscii[], float liczba, char digit, int poz);
int IntToASCII(char iAscii[], int liczba, int poz);
int BinToASCII(char bAscii[], char liczba, int poz);
void FillArray(char bufor[], char znak, int poz);
int MergeStr(char dst[], char tekst[], int pdest);

void ADC_init();
void ADC_trigger();
void ADC_select(char nr);

void ONEWIRE_init();
char ONEWIRE_reset();
void ONEWIRE_writeByte(unsigned char byte);
unsigned char ONEWIRE_readByte();

unsigned char ds18b20_convertT();
unsigned char ds18b20_read(unsigned char scratchpad[]);
unsigned char ds18b20_setADCresolution(char Th, char Tl, char Resolution);
float ds18b20_pomiar();
unsigned char CRC8(unsigned char inData[], unsigned char len);

void I2C_init();
void I2C_start();
void I2C_stop();
void I2C_write(unsigned char dane);
unsigned char I2C_read(unsigned char ACK);
void I2C_odczyt(unsigned char I2CaddrSlave, unsigned char iData[], unsigned char iDataCount);

void USART_init();

void INT_init();

// ----------------------------------------------
// ------------- GLOBALNE -----------------------
// ----------------------------------------------
volatile char liczt0;
volatile char fklaw = tklaw;
volatile char f1wire = t1wire;
volatile char flcd = tlcd;
volatile char fled = tled;
volatile char fadc = tadc;
volatile char fi2c = ti2c;
volatile char sklaw = 0;			 //stan klawiatury
volatile char OW_maska = 0b00000100; //DS18B20 przylaczony do PORTC,2
volatile unsigned int err1Wire = 0;  //licznik błedów 1wire
volatile unsigned char erri2c = 0;   //status I2C

// ----------------------------------------------
// -------------- MAIN --------------------------
// ----------------------------------------------
int main()
{
	// Inicjalizacja układów 0-wejscie, 1-wyjscie
	DDRB = 0b11111011; //PB0-LED , PB1-LED , PB2-przycisk(I), PB3-, PB4-, PB5-, PB6-, PB7-,
	DDRC = 0b11111000; //PC0-ADC1(I), PC1-ADC2(I), PC2-OneWire(I), PC3-, PC4-SDA, PC5-SCL, PC6-, PC7-,
	DDRD = 0b11111111; //PD0-, PD1-, PD2-lcdRS, PD3-lcdE, PD4-lcdD4, PD5-lcdD5, PD6-lcdD6, PD7-lcdD7,

	PORTB = 0xff; //port z podciąganiem
	PORTC = 0xff; //port z podciąganiem
	PORTD = 0xff; //port z podciąganiem

	LCD_init();
	ADC_init();
	ONEWIRE_init();
	I2C_init();
	//USART_init();
	INT_init();

	float V = 0.0, A = 0.0, W = 0.0, C = 0.0;
	unsigned char i2cData[2];

	char tekst[sbuf + 1] = {0};
	int poz;

	while (1)
	{
		//przerwanie z klawiatury
		if (!fklaw)
		{
			sklaw = PINB & 0b00000100; //	w sklaw jest stan portu
			if (!(sklaw & (1 << PB2)))
			{
				PORTB ^= 1;
			} //przycisk PB2
			fklaw = tklaw;
		}

		//przerwanie lcd
		if (!flcd)
		{
			PORTB ^= 2;
			//			Wyswietl_pomiar(V,A,W,C);

			poz = 0;
			poz = IntToASCII(tekst, err1Wire, poz);
			tekst[poz++] = ',';
			poz = FloatToASCII(tekst, C, 1, poz);
			tekst[poz++] = 1;
			FillArray(tekst, '.', poz);
			LCD_wk(0, 0);
			LCD_txt(tekst);

			poz = 0;
			FillArray(tekst, '.', poz);
			LCD_wk(1, 0);
			LCD_txt(tekst);

			flcd = tlcd;
		}

		//przerwanie led
		if (!fled)
		{
			//PORTB ^= 2;
			fled = tled;
		}

		//przerwanie adc
		if (!fadc)
		{
			int odczyt;
			int i;

			ADC_select(0);
			i = 10;
			odczyt = 0;
			while (i--)
			{
				ADC_trigger();
				odczyt += ADC;
			}					 //pobierz próbki
			odczyt /= 10;		 //uśrednij
			V = odczyt * 0.0025; //przeliczenie wartosci na napiecie

			ADC_select(1);
			i = 10;
			odczyt = 0;
			while (i--)
			{
				ADC_trigger();
				odczyt += ADC;
			}					 //pobierz próbki
			odczyt /= 10;		 //uśrednij
			A = odczyt * 0.0025; //przeliczenie wartosci na prad

			W = V * A;
			fadc = tadc;
		}

		//przerwanie 1wire
		if (!f1wire)
		{
			if (ds18b20_convertT())
			{
				float temp = ds18b20_pomiar();
				if (!(temp == -99))
				{
					C = temp;
				}
			}
			f1wire = t1wire;
		}

		//przerwanie i2c
		if (!fi2c)
		{
			//I2C_odczyt(0xA1,i2cData,2);
			fi2c = ti2c;
		}

		_delay_ms(10);
	}

	return 0;
}

// ----------------------------------------------
// -------------- FUNKCJE -----------------------
// ----------------------------------------------
inline void LCD_RSlow() { PORTD &= ~(1 << PORTD2); }
inline void LCD_RShigh() { PORTD |= (1 << PORTD2); }
inline void LCD_Elow() { PORTD &= ~(1 << PORTD3); }
inline void LCD_Ehigh() { PORTD |= (1 << PORTD3); }

void LCD_init()
{
	//PINY: GND,VDD,Vo ,RS ,RW ,E ,D4 ,D5 ,D6 ,D7 ,BL ,NC
	//		Zie,Bia,Cza,Nie,Cze,Ż ,Br ,Zie,Bia,Cza,Nie,Cze
	//		0v ,5v ,Rez,D2 ,0v ,D3,D4 ,D5 ,D6 ,D7 ,5V ,-

	//Wczytanie tablicy znakow z eeprom
	char ile = 4;		//ile znakow dodatkowych
	char znaki[ile][8]; // = tabl_znakow;
	for (unsigned char t = 0; t <= ile; t++)
	{
		for (unsigned char x = 0; x <= 7; x++)
		{
			if (t == 0)
			{
				znaki[t][x] = 0;
			} // znak 0 jest pusty, uzywamy jako koniec linii, którego nie wyswietlamy
			else
			{
				eeprom_busy_wait();
				znaki[t][x] = eeprom_read_byte((uint8_t *)((t - 1) * 8 + x + 1));
				//eeprom_write_byte((uint8_t *)0,0x0);		//przyklad zapisu
				//eeprom_write_update((uint8_t *)0,0x0);	//uzywamy update / wydluza zywotnosc
				//byte,word,dword,float,block
			}
		}
	}

	LCD_on();
	LCD_pisz('i', 0x28);		//4bity x 2line x 5x7
	LCD_pisz('i', 0x08);		//LCD off, Cursor off, Blink off
	LCD_cls();					//clear LCD
	LCD_pisz('i', 0x06);		//Shift right off
	LCD_pisz('i', 0x0C);		//LCD on, Cursor off, Blink off
	LCD_defchar(znaki, 0, ile); //definiuj znaki o adresie 0-7 w CGRAM,
}
void LCD_on()
{
	LCD_Elow();
	LCD_RSlow();
	_delay_ms(45);
	for (char t = 0; t < 3; t++) //wyslij 3x $3x
	{
		LCD_Ehigh();
		PORTD = (PORTD & 0x0f) | 0x30;
		asm("nop"); //wydluzenie impulsu
		asm("nop");
		asm("nop");
		LCD_Elow();
		_delay_ms(5);
	}
	LCD_Ehigh();
	PORTD = (PORTD & 0x0f) | 0x20; //wyslij $2x
	asm("nop");					   //wydluzenie impulsu
	asm("nop");
	asm("nop");
	LCD_Elow();
	_delay_us(100);
}
void LCD_trigger()
{
	LCD_Ehigh();
	_delay_us(5);
	LCD_Elow();
}
void LCD_pisz(char instrukcja, char data)
{
	(instrukcja == 'i') ? LCD_RSlow() : LCD_RShigh(); //czy dana czy instrukcja
	LCD_Ehigh();
	PORTD = (PORTD & 0x0f) | (data & 0xf0); //starszy polbajt
	asm("nop");
	asm("nop");
	asm("nop");
	LCD_Elow();
	_delay_us(100);

	LCD_Ehigh();
	PORTD = (PORTD & 0x0f) | ((data & 0x0f) << 4); //mlodszy polbajt
	asm("nop");
	asm("nop");
	asm("nop");
	LCD_Elow();
	_delay_us(100);
}
void LCD_defchar(char tablica[][8], char numer, char ile)
{
	numer &= 0x07; //0-7 adres nowych znaków
	ile &= 0x07;
	LCD_pisz('i', 0x40 | (numer * 8));
	for (unsigned char t = 0; t <= ile; t++)
	{
		for (unsigned char x = 0; x <= 7; x++)
		{
			LCD_pisz('d', tablica[t][x]);
		}
	}
	LCD_pisz('i', 0x80);
}
void LCD_cls()
{
	LCD_pisz('i', 0x01); //clear LCD, x,y = 0,0
	_delay_ms(2);		 //opoznienie dla czyszczenia ekranu
}
void LCD_wk(char w, char k)
{
	w &= 0x01;
	k &= 0x0f;
	LCD_pisz('i', (w * 0x40 + k) | 0x80);
}
char LCD_txt(char tekst[])
{
	unsigned char licz = 0;
	char zn = 0;
	while (1)
	{
		zn = tekst[licz++];
		if (!zn)
			break; //0 koniec stringa
		LCD_pisz('d', zn);
	}
	return licz;
}

void Wyswietl_pomiar(float V, float A, float W, float C)
{
	char tekst[sbuf + 1] = {0};
	int poz;
	char jedn;

	poz = 0;
	if (V < 0.1)
	{
		V *= 1000;
		jedn = 3;
	}
	else
	{
		jedn = 'V';
	} //mV
	poz = FloatToASCII(tekst, V, 2, poz);
	tekst[poz++] = jedn;
	tekst[poz++] = ' ';
	if (A < 0.1)
	{
		A *= 1000;
		jedn = 2;
	}
	else
	{
		jedn = 'A';
	} //mA
	poz = FloatToASCII(tekst, A, 2, poz);
	tekst[poz++] = jedn;
	FillArray(tekst, ' ', poz);
	LCD_wk(0, 0);
	LCD_txt(tekst);

	poz = 0;
	if (W < 0.1)
	{
		W *= 1000;
		jedn = 4;
	}
	else
	{
		jedn = 'W';
	} //mW
	poz = FloatToASCII(tekst, W, 2, poz);
	tekst[poz++] = jedn;
	tekst[poz++] = ' ';
	//poz = MergeStr(tekst, (char *)" ", poz);
	jedn = 1; //st.C
	poz = FloatToASCII(tekst, C, 1, poz);
	tekst[poz++] = jedn;
	FillArray(tekst, ' ', poz);
	LCD_wk(1, 0);
	LCD_txt(tekst);
}
int FloatToASCII(char fAscii[], float liczba, char digit, int poz)
{
	digit &= 0x03;
	poz &= 0x0f;
	float liczbaZ;
	if (liczba < 0)
	{
		liczbaZ = -liczba;
	}
	else
	{
		liczbaZ = liczba;
	} //abs liczby
	if (liczbaZ < 100)
	{
		fAscii[poz++] = ' ';
	} //miejsca znaczace
	if (liczbaZ < 10)
	{
		fAscii[poz++] = ' ';
	}
	if (liczba < 0)
	{
		fAscii[poz - 1] = '-';
	} //znak liczby

	poz = IntToASCII(fAscii, int(liczbaZ), poz);
	if (digit)
	{
		fAscii[poz++] = '.';
		int potega = 1;
		int tmp;
		while (digit > 0) //miejsca po przecinku
		{
			potega *= 10;
			digit--;
			tmp = (liczbaZ - (int)liczbaZ) * potega;
			(tmp >= 0 ? tmp : -tmp); //abs liczby
			if (!tmp && digit)
			{
				fAscii[poz++] = '0';
			} //zera w dziesietnych
		}
		poz = IntToASCII(fAscii, int(tmp), poz);
	}
	fAscii[poz] = 0; //koniec stringa
	return poz;		 //dlugosc tekstu
}
int IntToASCII(char iAscii[], int liczba, int poz)
{
	poz &= 0x0f;
	unsigned char poz1;
	unsigned char poz2 = poz;
	if (!(liczba))
	{
		iAscii[poz++] = '0';
	}
	while (liczba)
	{
		int x = liczba % 10;
		liczba /= 10;
		iAscii[poz++] = x + '0';
	}

	poz1 = poz - 1;
	char c;
	while (poz1 > poz2) //reverse
	{
		c = iAscii[poz1];
		iAscii[poz1] = iAscii[poz2];
		iAscii[poz2] = c;
		poz1--;
		poz2++;
	}

	iAscii[poz] = 0; //koniec stringa
	return poz;		 //dlugosc tekstu
}
int BinToASCII(char bAscii[], char liczba, int poz)
{
	poz &= 0x0f;
	char temp;
	for (char x = 8; x > 0; x--)
	{
		if (x > 1)
		{
			temp = (liczba >> (x - 1));
		}
		else
		{
			temp = liczba;
		}
		temp &= 0x01;
		bAscii[poz++] = temp + '0';
	}
	bAscii[poz] = 0; //koniec stringa
	return poz;		 //dlugosc tekstu
}
void FillArray(char bufor[], char znak, int poz)
{
	for (unsigned char x = poz; x < sbuf; x++)
	{
		bufor[x] = znak;
	}
	bufor[sbuf] = 0;
}
int MergeStr(char dst[], char tekst[], int pdest)
{
	pdest &= 0x0f;
	int ptekst = 0;
	while (tekst[ptekst])
	{
		dst[pdest++] = tekst[ptekst++];
	}
	dst[pdest] = 0;
	return pdest;
}

void ADC_init()
{
	ADCSRA = 0b10000011;
	//ADEN=1 włączenie przetwornika ADC
	//ADSC=1 pojedyncza konwersja
	//ADFR=1 konwersja ciągła
	//ADIF=1 flaga ustawiana po konwersji
	//ADIE=1 włącz przerwania
	//ADPS2:0: ustawienie preskalera, preskaler= 8
	//0 0 0 2
	//0 0 1 2
	//0 1 0 4
	//0 1 1 8
	//1 0 0 16
	//1 0 1 32
	//1 1 0 64
	//1 1 1 128
	ADMUX = 0b11000000;
	//REFS1:0: wybór napięcia odniesienia ADC
	//00   - Zewnętrzne AREF, wewnętrzne Vref wyłączone
	//01   - AVCC z zewnętrznym kondensatorem na wyprowadzeniu AREF
	//10   - zarezerwowane
	//11   - wewnętrzne źródło 2,56V z zewnętrznym kondensatorem na pinie AREF
	//ADLAR=1 ADC Left Adjust Result
	//bit 4 - NA
	//MUX3:0: Analog Channel Selection Bits
	//0000 - ADC0
	//0001 - ADC1
	//1110 - 1.30V (Vbg)
	//1111 - 0V	   (GND)
}
void ADC_trigger()
{
	ADCSRA |= 0b01000000; //ADSC=1 pojedyncza konwersja
	while (ADCSRA & 0b01000000)
		; //wait for end conversion
}
void ADC_select(char nr)
{
	ADMUX = (ADMUX & 0xf0) | (nr & 0x0f); //Select ADC 0,1
}

void ONEWIRE_init()
{
	OW_maska = 0b00000100; //DS18B20 przylaczony do PORTC,2
	ds18b20_setADCresolution(25, 0, 0x03);
	ds18b20_convertT();
	//RegByte 0 TH	//Sxxxxxxx
	//RegByte 0 TL	//Sxxxxxxx
	//R1R0 = CONF, scratchpad[4] = xR1R0xxxxx
	//	     00 -  9bit 94ms
	//	     01 - 10bit 188ms
	//	     10 - 11bit 375ms
	//	     11 - 12bit 750ms
}
char ONEWIRE_reset()
{
	//	funkcja zwraca:	   -1-nic nie wykryto
	//						0-wykryto urzadzenie
	//						1-wykryto zwarcie

	char InitVal = -1;
	PORTC &= ~OW_maska; //stan 0 na linii
	if (!(PINC & OW_maska))
		return InitVal; //pomimo 0, jest 1 na linii
	DDRC |= OW_maska;   //port jako wyjscie z podciaganiem
	_delay_us(500);		//opoznienie 500us
	DDRC &= ~OW_maska;  //port jako wejscie stan 1 pochodzi z rezystora PullUp
	_delay_us(75);
	if (!(PINC & OW_maska))
	{
		InitVal++;
	}
	_delay_us(500);
	if (!(PINC & OW_maska))
	{
		InitVal++;
	}
	return InitVal;
}
void ONEWIRE_writeByte(unsigned char byte)
{
	PORTC &= ~OW_maska; //stan 0 na linii
	for (char i = 0; i <= 7; i++)
	{
		DDRC |= OW_maska;
		if (byte & 0x01) //write 1
		{
			_delay_us(7);
			DDRC &= ~OW_maska;
			_delay_us(70);
		}
		else //write 0
		{
			_delay_us(70);
			DDRC &= ~OW_maska;
			_delay_us(7);
		}
		byte >>= 1;
	}
}
unsigned char ONEWIRE_readByte()
{
	unsigned char byte = 0;
	DDRC &= ~OW_maska;

	for (char i = 0; i <= 7; i++)
	{
		DDRC |= OW_maska;
		_delay_us(7);
		DDRC &= ~OW_maska;
		_delay_us(7);
		byte >>= 1;
		if (PINC & OW_maska)
		{
			byte |= 0x80;
		}
		_delay_us(70);
	}
	return byte;
}

unsigned char ds18b20_convertT()
{
	if (ONEWIRE_reset())
		return 0; //wysyła do układu ds18b20 polecenie pomiaru

	ONEWIRE_writeByte(0xcc); // SKIP ROM
	ONEWIRE_writeByte(0x44); // CONVERT_T
	return 1;
}
unsigned char ds18b20_read(unsigned char scratchpad[])
{
	if (ONEWIRE_reset())
		return 0;

	ONEWIRE_writeByte(0xcc); // SKIP ROM
	ONEWIRE_writeByte(0xbe); // READ SCRATCHPAD

	for (unsigned char t = 0; t < 9; t++)
	{
		scratchpad[t] = ONEWIRE_readByte();
	} //2xTEMP(TLSB,THSB),2xREGISTER ALARM(TH, TL),1xCONF, 3xRESERVED, 1xCRC
	return 1;
}
unsigned char ds18b20_setADCresolution(char Th, char Tl, char Resolution)
{
	Resolution = ((Resolution & 0x03) << 5) | 0x1f;
	if (ONEWIRE_reset())
		return 0;				   //wysyła do układu ds18b20 polecenie pomiaru
	ONEWIRE_writeByte(0xcc);	   // SKIP ROM
	ONEWIRE_writeByte(0x4e);	   // WRITE on SCRATCHPAD
	ONEWIRE_writeByte(Th);		   // RegByte 0 - unused
	ONEWIRE_writeByte(Tl);		   // RegByte 1 - unused
	ONEWIRE_writeByte(Resolution); // RegByte 2 - configuration

	if (ONEWIRE_reset())
		return 0;			 //wysyła do układu ds18b20 polecenie pomiaru
	ONEWIRE_writeByte(0xcc); // SKIP ROM
	ONEWIRE_writeByte(0x48); // COPY SCRATCHPAD ti EEPROM
	return 1;
}
float ds18b20_pomiar()
{
	unsigned char ds18b20_pad[9];

	//_delay_ms(750);		//czas konwersji dla 12 bit, u nas przerwanie
	/* 	Odczyt z układu ds18b20, dane zapisywane są w tablicy ds18b20_pad.
		Dwie pierwsze pozycje w tablicy to kolejno mniej znaczący bajt i bardziej
		znaczący bajt wartość zmierzonej temperatury */
	ds18b20_read(ds18b20_pad);
	/*	Składa dwa bajty wyniku pomiaru w całość. Cztery pierwsze bity mniej
		znaczącego bajtu to część ułamkowa wartości temperatury, więc całość
		dzielona jest przez 16 */

	float temp;

	if (CRC8(ds18b20_pad, 8) == ds18b20_pad[8])
	{
		temp = ((ds18b20_pad[1] << 8) + ds18b20_pad[0]) / 16.0;
		if (ds18b20_pad[1] & 0xf8)
		{
			temp = -temp;
		} //temp. ujemna
	}
	else
	{
		err1Wire++;
		temp = -99;
	} //licznik błędów 1wire
	return temp;
}
unsigned char CRC8(unsigned char inData[], unsigned char len)
{
	unsigned char crc = 0;
	for (; len; len--)
	{
		crc ^= *inData++;
		crc ^= (crc << 3) ^ (crc << 4) ^ (crc << 6);
		crc ^= (crc >> 4) ^ (crc >> 5);
	}
	return crc;
}

void I2C_init()
{
	TWBR = 0b00010001;
	/* Rejestr odpowiedzialny za wybór współczynnika podziału dla generatora.
	     Generator ten odpowiada za czestotliwosc która jest dzielona przez
	     sygnał zegarowy SCL w trybie pracy Master.
	     Czestotliwosc TWI(i2c) = 100kHz (max 100kHz)
	     ->> TWBR musi byc wieksze od 10 dla stabilnej pracy TWI(i2c)
	   	 ->> TWBR=((częstotliwość Atmegi/częstotliwość TWI)-16)/2
	   	 dla 20kHz
	 	 Czyli ((1MHz/20kHz)-16)/2/4^TWPS=17 */
	/* SCL = (CPU CLOCK)/(16+2*TWBR*4^TWPS)
		 SCL = 1MHz/352=2841Hz */
	TWSR = 0b00000000;
	//TWS7:3  status
	//TWS2	  reserved
	//TWPS1:0 preskaler (00->1, 01->4, 10->16, 11->64)
}
void I2C_start()
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA); //wyslanie sekwencji start, inicjacja w trybie master
		//TWINT=1 rozpoczęcie operacji i2c
		//TWEA =1 generowanie ACK
		//TWSTA=1 wygenerowanie START
		//TWSTO=1 wygenerowanie STOP
		//TWWC =1 ustawiany przed lub po zapisie do TWDR
		//TWEN =1 włącz interfejs i2c
		//Reserved
		//TWIE =1 przerwanie od i2c
	while (!(TWCR & (1 << TWINT)))
		; //oczekiwanie na flage TWINT
	erri2c = TWSR & 0xF8;
	if ((erri2c & 0x08) || (erri2c & 0x10))
	{
		erri2c = 0;
	} //0->ok, pozostale -> kod bledu
	  //0x08 - TW_START
	  //0x10 - TW_REP_START
}
void I2C_stop()
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO); //wyslanie sekwencji stop, zakonczenie transmisji
	while (!(TWCR & (1 << TWSTO)))
		; //oczekiwanie na bit stopu
	erri2c = TWSR & 0xF8;
	if (erri2c & 0xF8)
	{
		erri2c = 0;
	} //0->ok, pozostale -> kod bledu
	  //0xF8 - TW_NO_INFO
}
void I2C_write(unsigned char dane)
{
	TWDR = dane;					   //TWDR7:0 -> data
	TWCR = (1 << TWINT) | (1 << TWEN); //wyslanie 1 bajtu
	while (!(TWCR & (1 << TWINT)))
		; //oczekiwanie na zakonczenie
	erri2c = TWSR & 0xF8;
	if ((erri2c & 0x18) || (erri2c & 0x28) || (erri2c & 0x40))
	{
		erri2c = 0;
	} //0->ok, pozostale -> kod bledu
	  //0x18 - TW_MT_SLA_ACK	- adres
	  //0x28 - TW_MT_DATA_ACK	- data
	  //0x40 - TW_MR_SLA_ACK	- tryb Master Receiver
}
unsigned char I2C_read(unsigned char ACK)
{
	TWCR = (1 << TWINT) | (ACK << TWEA) | (1 << TWEN); //ustawienie rejestru TWCR, ACK=1 (acknowledge)
	while (!(TWCR & (1 << TWINT)))
		; //oczekiwanie na flage
	erri2c = TWSR & 0xF8;
	if ((erri2c & 0x50) || (erri2c & 0x58))
	{
		erri2c = 0;
	} //0->ok, pozostale -> kod bledu
	//0x50 - TW_MR_DATA_ACK
	//0x58 - TW_MR_DATA_NACK
	return TWDR; //zwrocenie wartosci
}
void I2C_odczyt(unsigned char I2CaddrSlave, unsigned char iData[], unsigned char iDataCount)
{
	unsigned char erri2cTemp = 0;
	I2C_start();
	erri2cTemp += erri2c;
	I2C_write(I2CaddrSlave);
	erri2cTemp += erri2c;
	unsigned char iCounter;
	for (iCounter = 0; iCounter < iDataCount; iCounter++)
	{
		iData[iCounter] = I2C_read(1); //odczyt wysylamy ACK
		erri2cTemp += erri2c;
	}
	iData[iCounter] = I2C_read(0); //ostatnia dana, bez ACK
	erri2c += erri2cTemp;
}

void USART_init()
{
	UCSRC = (1 << URSEL) | (1 << UPM1) | (1 << UCSZ1) | (1 << UCSZ0);
	//URSEL - select UCSRC(1), UBRRH(0)
	//UMSEL - asynch(0), synch(1)
	//UPM1:0 - parzystosc 00-disabled, 01-reserved, 10-Even, 01-Odd
	//USBS  - bity stopu 1(0), 2(1)
	//UCSZ1:0 -
	//		000	- 5bit
	//		001	- 6bit
	//		010	- 7bit
	//		011	- 8bit
	//		100	- Reserved
	//		101	- Reserved
	//		110	- Reserved
	//		111	- 9bit
	//UCPOL - tylko w synch, dla asynch = 0, zbocza opadajace i narastajace
	UCSRB = 0b00000000;
	//RXCIE - 1 -> właczenie przerwan RX
	//TXCIE - 1 -> właczenie przerwan TX
	//UDRIE - 1 -> właczenie przerwan DATA REGISTER EMPTY
	//RXEN  - 1 -> właczenie RX
	//TXEN  - 1 -> właczenie TX
	//UCSZ2
	//RXB8	-	-> bit 8 (dla ramki 9bit)
	//TXB8	-	-> bit 8 (dla ramki 9bit)
	//UCSRA
	//RXC	- receive complete
	//TXC	- transmit complete
	//UDRE	- 1 - buffer is empty
	//FE	- frame error
	//DOR	- data overRun
	//PE	- bład parzystości
	//U2X	- tylko dla asynchr, synch(0), zwiekszenie szybkosci x2, (z16 na 8)
	//MPCM	- Multi Processor Communication Mode
	//UBRRL7:0
	//UBRRH = UCSRC
	//UBRRH15	- URSEL
	//UBRRH14:12- Reserved
	//UBRRH11:8

	//UBRR = (fwy/16BAUD)-1	;	U2X=0	;BAUD=fwy/16(UBRR+1)
	//UBRR = (fwy/8BAUD)-1	;	U2X=1	;BAUD=fwy/8(UBRR+1)
	//UBRR = (fwy/2BAUD)-1	;	synchr	;BAUD=fwy/2(UBRR+1)
	//ERROR[%] = ((BAUDRATEwyliczony/BAUDRATE) -1)*100%
}

void INT_init()
{
	liczt0 = vliczt0;
	TCNT0 = tau0;
	TIMSK = 0b00000001; //TOIE0 enable
	TCCR0 = 0b00000011; //CS0:7-3 - nieuzywane
						//CS0:2-0 - preskaler, 011 = /64
						//fwy=(1/fosc) * (256-TCNT0) * preskaler * liczt0
	sei();
}

// ----------------------------------------------
// -------------- PRZERWANIA --------------------
// ----------------------------------------------
ISR(TIMER0_OVF_vect)
{
	TCNT0 = tau0;
	if (!--liczt0)
	{
		liczt0 = vliczt0;

		if (fklaw)
		{
			fklaw--;
		}
		if (flcd)
		{
			flcd--;
		}
		if (fled)
		{
			fled--;
		}
		if (fadc)
		{
			fadc--;
		}
		if (fi2c)
		{
			fi2c--;
		}
		if (f1wire)
		{
			f1wire--;
		}
	}
}
