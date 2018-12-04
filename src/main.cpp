/*

Home Automation Richard Selles
Januari 2018

VERSION 01-12-2018

*/

/*
* Includes libraries
*/

#include <SPI.h>
#include <Ethernet2.h>// Webserver
#include <OneWire.h> // 1-wire temperatuur sensors
#include <LiquidCrystal_I2C.h> // I2C LCD Display
#include <Wire.h> // I2C RTC
#include "RTClib.h" //RTC 
RTC_DS1307 RTC;  



#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
	char top;
#ifdef __arm__
	return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
	return &top - __brkval;
#else  // __arm__
	return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}


/*
* Global ==============================
*/

//extended debug info
bool debug = true;
bool debugminimum = true;

//RUN mode data
int run_thermostaat = 20;
char run_mode = 'S';
// 'S' = Stoppen
// 'F' = Filteren
// 'P' = Programma filteren
// 'V' = Verwarmen

// 0=OFF, 1=ON, 2=OFF,CONSTAIN
int run_pomp = 0;
int run_uv = 0;
int run_heater = 0;
int pomp_startup_speed = 161;

//data array
float data[15];
/*
[0] = na
[1] = na
[2] = pressure
[3] = temp in
[4] = temp out
[5] = temp inverter
[6] = test temp
[7] = delta temp in vs out
[8] = debiet (liters/uur)
[9] = totaal flow (liters)
[10] = potentiometer
[11] = now.hour
*/

//Pressure
int pressure_pin = A0; // This is te pin number connected to pressure sensor

//I2C
//SCA (serial data) A4
//SCL (serial clock) A5

// LCD (I2C)
LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
int wipeCounter = 0;

//RTC (I2C)
DateTime now;

//Potentiometer
int potentiometerPin = 3;

//Devices
int heaterSSRpin = 7;
int uvSSRpin = 6;
int pompPWMpin = 9;
int pompStartpin = 8;


// Flow-hall sensor
byte flow_interruptPin = 2; // interrupt 0 op pin x -- Alleen pins 2&3 zijn bij UNO beschikbaar voor interrupts
float flow_calibrationFactor = 12; // The hall-effect flow sensor x pulses per litre
unsigned long flow_pulseTotalEver = 0; // overall aantal pulsen
volatile byte flow_pulseCount;
float flow_debiet;
unsigned long flow_oldTime;


// Webserver Circuit: Ethernet shield attached to pins 10, 11, 12, 13
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; // Enter a MAC address and IP address for your controller below.
IPAddress ip(192, 168, 178, 188 ); // The IP address will be dependent on your local network:
EthernetServer server(80); // (port 80 is default for HTTP):
EthernetClient client;
String readString = String(25); //read buffer oa GET/POST data

// Temperatuur Sensors
OneWire  ds(3); // Sensor(s) data pin is connected to Arduino pin x in non-parasite mode!

//int
int IteratorLoop = 0;



/*
* Functions
*/

void PompOn()
{
	if (run_pomp == 1) {
		// great!
	}
	else if (run_pomp == 0) {
		analogWrite(pompPWMpin, pomp_startup_speed / 4); // opstart snelheid
		digitalWrite(pompStartpin, HIGH); //start pomp
		delay(4000); // starten die pomp!
		analogWrite(pompPWMpin, data[10] / 4); // running speed - 1023 max input vs 255 max output
		run_pomp = 1;
	}
}

/*
active hour?
2h 4h ...
*/
bool activeHour() {
	int h = now.hour();
	if ((h == 4) || (h == 8) || (h == 12) || (h == 16) || (h == 18) || (h == 20) || (h == 22) || (h == 0))
		return true;
	else
		return false;
}


/*
effectueer status naar apparaten
*/
void Calculate_Status_Devices()
{
	// 0=OFF, 1=ON, 2=OFF,CONSTAIN

	switch (run_mode) {
		case 'S':
			run_pomp = 0;
			run_uv = 0;
			run_heater = 0;
			break;
		case 'F':   
			PompOn();
			run_heater = 0;
			if(data[8]>1) //debiet > x
				run_uv = 1;
			else
				run_uv = 2; //constrain violation
			break;
		case 'P':    
			if (activeHour()) {
				PompOn();
				if (data[8] > 1) { //debiet > x
					run_uv = 1;

					// ### heater regeltechniek ###
					float boven_grens = run_thermostaat + 0.3;
					float onder_grens = run_thermostaat - 0.3;
					float temperatuur = data[3];

					if (run_heater == 1) {
						if (temperatuur >= boven_grens) {
							// warm genoeg; uitzetten
							run_heater = 0;
						}
					}
					else {
						if (temperatuur <= onder_grens) {
							// te koud; aanzetten
							run_heater = 1;
						}
					}
					// ### heater regeltechniek ###

				} else {
					run_uv = 2; //constrain violation
					run_heater = 2; //constrain violation
				}

			} else {
				run_pomp = 0;
				run_uv = 0;
				run_heater = 0;
			}
			break;
		case 'V':    
			PompOn();
			if (data[8] > 1) { //debiet > x
				run_uv = 1;

				// ### heater regeltechniek ###
				float boven_grens = run_thermostaat + 0.3;
				float onder_grens = run_thermostaat - 0.3;
				float temperatuur = data[3];

				if (run_heater == 1) {
					if (temperatuur >= boven_grens) {
						// warm genoeg; uitzetten
						run_heater = 0;
					}
				} else {
					if (temperatuur <= onder_grens) {
						// te koud; aanzetten
						run_heater = 1;
					}
				}
				// ### heater regeltechniek ###


			} else {
				run_uv = 2; //constrain violation
				run_heater = 2; //constrain violation
			}
			break;
	}

	//set UV
	if(run_uv==1)
		digitalWrite(uvSSRpin, HIGH);
	else
		digitalWrite(uvSSRpin, LOW);

	//set HEATER
	if (run_heater == 1)
		digitalWrite(heaterSSRpin, HIGH);
	else
		digitalWrite(heaterSSRpin, LOW);

	//set POMP
	analogWrite(pompPWMpin, data[10] / 4); //1023 max input vs 255 max output
	if (run_pomp == 1)
		digitalWrite(pompStartpin, HIGH);
	else
		digitalWrite(pompStartpin, LOW);  
}

/*
flowmeter counter
*/
void FlowInterruptFunction()
{
	// Increment the pulse counter
	flow_pulseCount++; // binnen loop, debiet calc
	flow_pulseTotalEver++; // total
}

void ReadFlowsensor()
{
	if ((millis() - flow_oldTime) > 1000)    // Only process counters once per second
	{
		// Disable the interrupt while calculating flow rate
		detachInterrupt(digitalPinToInterrupt(flow_interruptPin));

		// litres/hour
		flow_debiet = (((1000.0 / (millis() - flow_oldTime)) * flow_pulseCount) / flow_calibrationFactor) * 60.0 * 60;
		flow_oldTime = millis(); // reset timer

								 // Reset the pulse counter so we can start incrementing again
		flow_pulseCount = 0;

		//store global
		data[8] = int(flow_debiet); // debiet(liters / uur)   
		data[9] = flow_pulseTotalEver / 12.0; // totaal flow(liters)

											  // Enable the interrupt again now that we've finished sending output
		attachInterrupt(digitalPinToInterrupt(flow_interruptPin), FlowInterruptFunction, FALLING);
	}
}


void ReadPressureSensor() {
	/*
	Sensor specs:
	type : SKU237545
	output : 0.5-4.5VDC
	Working pressure : 0-1.2MPa = 0-12Bar

	Analog port specs
	port 0-5 Volt / 1024 waarden / 0-1023 bitwaarde
	*/

	//calculatie volgens video: https://www.youtube.com/watch?v=AB7zgnfkEi4
	float voltage = (analogRead(pressure_pin) * 5.0) / 1024.0; // port 0-5 Volt ::= 0-1023 bitwaarde
	float pressure_pascal = (3.0*((float)voltage - 0.47))*1000000.0;
	float pressure_bar = pressure_pascal / 10e5;

	//store global
	data[2] = pressure_bar;

	//debug info
	if (debug) {
		Serial.print("Pressure:");
		Serial.print(data[2]);
		Serial.println();
	}
}

void ReadTemperatureSensors() {
	byte counter;
	byte present;
	byte sensor_type;
	byte tdata[12];
	byte addr[8];
	float celsius;
	String serialnb;

	ds.reset_search();

	while (ds.search(addr)) {

		// Get Serial number
		if (debug) Serial.print("Temp ");

		serialnb = "";
		for (counter = 0; counter < 8; counter++)
		{
			if (addr[counter] < 0x10) serialnb.concat("0");
			serialnb.concat(String(addr[counter], HEX));
		}
		if (debug) Serial.print(serialnb);
		if (debug) Serial.print(" ");

		// Check CRC
		if (OneWire::crc8(addr, 7) != addr[7])
		{
			Serial.println("   ERROR\n");
			return;
		}

		// Get Chip type (the first ROM byte indicates which chip)
		switch (addr[0])
		{
		case 0x10:
			// DS18S20
			sensor_type = 1;
			break;
		case 0x28:
			// DS18B20
			sensor_type = 0;
			break;
		case 0x22:
			// DS1822
			sensor_type = 0;
			break;
		default:
			//undefined
			return;
		}
		if (debug) Serial.print(sensor_type);

		ds.reset();
		ds.select(addr);
		ds.write(0x44);  // start conversion, with regular (non-parasite!) power

		delay(1000);     // maybe 750ms is enough, maybe not (original 1000)

		present = ds.reset();
		ds.select(addr);
		ds.write(0xBE);  // Read Scratchpad

		// Get Raw Temp Data, we need 9 bytes
		for (counter = 0; counter < 9; counter++)
		{
			tdata[counter] = ds.read();
		}

		// Convert the data to actual temperature
		// because the result is a 16 bit signed integer, it should
		// be stored to an "int16_t" type
		int16_t raw = (tdata[1] << 8) | tdata[0];

		if (sensor_type)
		{
			raw = raw << 3; // 9 bit resolution default
			if (tdata[7] == 0x10) {
				// "count remain" gives full 12 bit resolution
				raw = (raw & 0xFFF0) + 12 - tdata[6];
			}
		}
		else
		{
			// at lower res, the low bits are undefined, so let's zero them
			byte cfg = (tdata[4] & 0x60);

			//// default is 12 bit resolution, 750 ms conversion time
			if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
			else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
			else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
		}

		celsius = (float)raw / 16.0;

		if (debug) {
			Serial.print(":");
			Serial.print(celsius);
			Serial.println();
		}

		//store global
		if (celsius>0) { //storing op serial bus; workaroud;-)
			if (serialnb.compareTo("28ff14ba85160352") == 0) data[3] = celsius; //temp in
			if (serialnb.compareTo("28ff6722861604eb") == 0) data[4] = celsius; //temp out
			if (serialnb.compareTo("28ffc9b8851603e7") == 0) data[5] = celsius; //temp inverter
			if (serialnb.compareTo("28ff60f6861605eb") == 0) data[6] = celsius; //test temp
			data[7] = data[4] - data[3];
		}
		else {
			Serial.println("error: temp out of range");
		}
	}
}

// show debug data
void EchoData() {
	if (debugminimum) {
		Serial.print("[");
		Serial.print(data[0]);
		Serial.print("|");
		Serial.print(data[1]);
		Serial.print("|");
		Serial.print(data[2]);
		Serial.print("|");
		Serial.print(data[3]);
		Serial.print("|");
		Serial.print(data[4]);
		Serial.print("|");
		Serial.print(data[5]);
		Serial.print("|");
		Serial.print(data[6]);
		Serial.print("|");
		Serial.print(data[7]);
		Serial.print("|");
		Serial.print(data[8]);
		Serial.print("|");
		Serial.print(data[9]);
		Serial.print("|");
		Serial.print(data[10]);
		Serial.print("|");
		Serial.print(data[11]);

		Serial.print("]{");
		Serial.print(run_mode);
		Serial.print("}{");
		Serial.print(run_thermostaat);
		Serial.print("}{");
		Serial.print(run_pomp);
		Serial.print("}{");
		Serial.print(run_uv);
		Serial.print("}{");
		Serial.print(run_heater);
		Serial.print("}<");
		Serial.print(freeMemory());
		Serial.print(">");
		Serial.println();
		
	}
}

/*
toon float netjes op lcd op 4 characters
*/
void lcdPrintFloat(float f) {
	// -10 tot 100
	if (f <= -10) {
		lcd.print("LOW_");
	}
	else if (f < 0) {
		char outstr[10];
		dtostrf(f, 4, 1, outstr);
		lcd.print(outstr);
	}
	else if (f >= 100) {
		lcd.print("HIGH");
	}
	else {
		if (f < 10) {
			char outstr[10];
			dtostrf(f, 4, 2, outstr);
			lcd.print(outstr);
		}
		else {
			//>=10
			char outstr[10];
			dtostrf(f, 4, 1, outstr);
			lcd.print(outstr);
		}
	}
}

/*
toon int netjes op lcd op 2 characters
*/
void lcdPrintInt2Digits(int i) {
	if (i < 10) {
		lcd.print(" ");
		lcd.print(i);
	} else if (i <100) {
		lcd.print(i);
	} else {
		lcd.print("ER");
	}
}

void DisplayDataLCD() {
	//mode
	lcd.setCursor(0, 0);
	switch (run_mode) {
		case 'S':
			lcd.print(F("STOPPEN    "));
			break;
		case 'F':
			lcd.print(F("FILTEREN   "));
			break;
		case 'P':
			lcd.print(F("FILTER-PROG"));
			break;
		case 'V':
			lcd.print(F("VERWARMEN  "));
			break;
	}

	//thermostaat
	lcd.setCursor(17, 0);
	lcdPrintInt2Digits(run_thermostaat);

	//temp in
	lcd.setCursor(5, 1);
	lcdPrintFloat(data[3]);

	//temp out
	lcd.setCursor(12, 1);
	lcdPrintFloat(data[4]);

	//Flow
	lcd.setCursor(5, 2);
	lcdPrintFloat(data[8] / 1000);

	//pressure
	lcd.setCursor(5, 3);
	lcdPrintFloat(data[2]);

	//pomp
	lcd.setCursor(17, 3);
	if (run_pomp==1)
		lcd.print("X");
	else if(run_pomp==0)
		lcd.print("-");
	else
		lcd.print("!");

	//uv
	lcd.setCursor(18, 3);
	if (run_uv==1)
		lcd.print("X");
	else if(run_uv == 0)
		lcd.print("-");
	else
		lcd.print("!");

	//heater
	lcd.setCursor(19, 3);
	if (run_heater==1)
		lcd.print("X");
	else if(run_heater == 0)
		lcd.print("-");
	else
		lcd.print("!");
	
	//hours
	lcd.setCursor(12, 0);
	lcdPrintInt2Digits(now.hour());
}

/*
0 - [FILTER-PROG 23h  35C]
1 - [Temp:21.6 > 22.3    ]
2 - [Flow:4.55 m3/h   PUH]
3 - [Pres:0.67 bar    XX-]
     |    |      |    |
	 0    5      12   17
*/
void DisplayFrameLCD() {
	lcd.setCursor(0, 0);
	lcd.print("            __h  __C");
	lcd.setCursor(0, 1);
	lcd.print("Temp:     >");
	lcd.setCursor(0, 2);
	lcd.print("Flow:     m3/h ");
	lcd.setCursor(0, 3);
	lcd.print("Pres:     bar");
	lcd.setCursor(17, 2);
	lcd.print("PUH");
}






/*
* Setup ==============================
*/

void setup() {

	// Open serial communications and wait for port to open:
	Serial.begin(9600);

	//Welkom
	Serial.println("Program start");

	// RTC
	Wire.begin();
	RTC.begin();
	if (!RTC.isrunning()) {
		Serial.println("RTC is NOT running!");
		// following line sets the RTC to the date & time this sketch was compiled
		RTC.adjust(DateTime(__DATE__, __TIME__));
	} else {
		now = RTC.now();
		Serial.print(now.year(), DEC);
		Serial.print('/');
		Serial.print(now.month(), DEC);
		Serial.print('/');
		Serial.print(now.day(), DEC);
		Serial.print(' ');
		Serial.print(now.hour(), DEC);
		Serial.print(':');
		Serial.print(now.minute(), DEC);
		Serial.println();
	}

	//SSR init / heater & UV-C / Pomp
	pinMode(heaterSSRpin, OUTPUT);
	pinMode(uvSSRpin, OUTPUT);
	pinMode(pompStartpin, OUTPUT);
	pinMode(pompPWMpin, OUTPUT);

	digitalWrite(heaterSSRpin, LOW);
	digitalWrite(uvSSRpin, LOW);
	digitalWrite(pompStartpin, LOW);
	analogWrite(pompPWMpin, 128); //helft van max 255


	//LCD
	lcd.init();
	lcd.backlight();
	DisplayFrameLCD();

	//Init Flow sensor
	pinMode(flow_interruptPin, INPUT);
	digitalWrite(flow_interruptPin, HIGH);
	flow_pulseCount = 0; // teller binnen loop
	flow_debiet = 0.0; // debiet
	flow_oldTime = 0; // timer binnen loop
	attachInterrupt(digitalPinToInterrupt(flow_interruptPin), FlowInterruptFunction, FALLING); // config interrupt 
																								   // start the Ethernet connection and the server:
	Ethernet.begin(mac, ip);
	server.begin();
	Serial.print("webserver is at ");
	Serial.println(Ethernet.localIP());
}


/*
* Programm loop ==============================
*/
void loop() {

	// listen for incoming clients
	EthernetClient client = server.available();
	if (client) {

		Serial.println("http-request");
		// an http request ends with a blank line
		boolean currentLineIsBlank = true;
		while (client.connected()) {


			if (client.available()) {
				char c = client.read();
				if (debug) Serial.print(c);

				//read char / bepaald max ivm memory
				if (readString.length() < 20) {
					//store characters to string 
					readString.concat(c);
				}

				// if you've gotten to the end of the line (received a newline
				// character) and the line is blank, the http request has ended,
				// so you can send a reply
				if (c == '\n' && currentLineIsBlank) {

					if (readString.indexOf("?STOPPEN") >= 0) { //actie herkend
						Serial.println("STOPPEN");
						run_mode = 'S';
					}
					if (readString.indexOf("?FILTEREN") >= 0) { //actie herkend
						Serial.println("FILTEREN");
						run_mode = 'F';
					}
					if (readString.indexOf("?VERWARMEN") >= 0) { //actie herkend
						Serial.println("VERWARMEN");
						run_mode = 'V';
					}
					if (readString.indexOf("?FILTER-PROG") >= 0) { //actie herkend
						Serial.println("FILTER-PROG");
						run_mode = 'P';
					}
					if (readString.indexOf("?5") >= 0) { //actie herkend
						Serial.println("5");
						run_thermostaat = 5;
					}
					if (readString.indexOf("?10") >= 0) { //actie herkend
						Serial.println("10");
						run_thermostaat = 10;
					}
					if (readString.indexOf("?15") >= 0) { //actie herkend
						Serial.println("15");
						run_thermostaat = 15;
					}
					if (readString.indexOf("?20") >= 0) { //actie herkend
						Serial.println("20");
						run_thermostaat = 20;
					}
					if (readString.indexOf("?25") >= 0) { //actie herkend
						Serial.println("25");
						run_thermostaat = 25;
					}
					if (readString.indexOf("?30") >= 0) { //actie herkend
						Serial.println("30");
						run_thermostaat = 30;
					}
					if (readString.indexOf("?34") >= 0) { //actie herkend
						Serial.println("34");
						run_thermostaat = 34;
					}
					if (readString.indexOf("?35") >= 0) { //actie herkend
						Serial.println("35");
						run_thermostaat = 35;
					}
					if (readString.indexOf("?36") >= 0) { //actie herkend
						Serial.println("36");
						run_thermostaat = 36;
					}
					if (readString.indexOf("?37") >= 0) { //actie herkend
						Serial.println("37");
						run_thermostaat = 37;
					}
					if (readString.indexOf("?38") >= 0) { //actie herkend
						Serial.println("38");
						run_thermostaat = 38;
					}
					if (readString.indexOf("?39") >= 0) { //actie herkend
						Serial.println("39");
						run_thermostaat = 39;
					}

					//Bepaal statussen apparaten
					Calculate_Status_Devices();

					readString = "";

					if (debug) Serial.println("Sending response");

					//Response headers
					client.println(F("HTTP/1.1 200 OK"));	// send a standard http response header
					client.println(F("Content-Type: text/xml"));
					client.println(F("Connection: keep-alive")); //alternatief = close
					client.println();

					//XML response
					Serial.println(F("collect & send data"));
					client.print(F("<?xml version = \"1.0\" ?>"));
					client.print(F("<sensordata>"));

					client.print(F("<pressure>"));
					client.print(data[2]);
					client.print(F("</pressure>"));

					client.print(F("<flow>"));
					client.print(F("<debiet>"));
					client.print(data[8] / 1000);
					client.print(F("</debiet>"));
					client.print(F("<total>"));
					client.print(data[9]);
					client.print(F("</total>"));
					client.print("</flow>");

					client.print(F("<temperature>"));
					client.print(F("<in>"));
					client.print(data[3]);
					client.print(F("</in>"));
					client.print(F("<out>"));
					client.print(data[4]);
					client.print(F("</out>"));
					client.print(F("<delta>"));
					client.print(data[7]);
					client.print(F("</delta>"));
					client.print(F("<inverter>"));
					client.print(data[5]);
					client.print(F("</inverter>"));
					client.print(F("</temperature>"));

					client.print(F("<run>"));
					client.print(F("<mode>"));

					switch (run_mode) {
					case 'S':
						client.print(F("STOPPEN"));
						break;
					case 'F':
						client.print(F("FILTEREN"));
						break;
					case 'P':
						client.print(F("PROG FILTEREN"));
						break;
					case 'V':
						client.print(F("VERWARMEN  "));
						break;
					}

					client.print(F("</mode>"));
					client.print(F("<thermostaat>"));
					client.print(run_thermostaat);
					client.print(F("</thermostaat>"));
					client.print(F("<run_pomp>"));
					client.print(run_pomp);
					client.print(F("</run_pomp>"));
					client.print(F("<run_uv>"));
					client.print(run_uv);
					client.print(F("</run_uv>"));
					client.print(F("<run_heater>"));
					client.print(run_heater);
					client.print(F("</run_heater>"));
					client.print(F("</run>"));

					client.println(F("</sensordata>"));


					break;
				}
				if (c == '\n') {
					// you're starting a new line
					currentLineIsBlank = true;
				}
				else if (c != '\r') {
					// you've gotten a character on the current line
					currentLineIsBlank = false;
				}
			}
		}
		// give the web browser time to receive the data
		delay(100);
		// close the connection:
		client.stop();
		Serial.println("client disconnected");

	}
	else {

		// collect data sensors
		ReadPressureSensor();
		ReadFlowsensor();

		//Potentiometer
		data[10] = analogRead(potentiometerPin);

		if (IteratorLoop > 10) {
			
			//RTC
			now = RTC.now();
			data[11] = now.hour();

			//Temperatuur sensors
			ReadTemperatureSensors();

			//reset
			IteratorLoop = 0;

		}
		else {
			IteratorLoop++;
		}


		//Bepaal statussen apparaten
		Calculate_Status_Devices();

		//echo data
		if (debugminimum) EchoData();

		// write lcd display
		DisplayDataLCD();
	}
}