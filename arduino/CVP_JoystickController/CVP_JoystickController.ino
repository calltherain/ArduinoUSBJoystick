/* Arduino USB Joystick HID demo */
/* Author: Darran Hunt, 
*/

/* 
Mega2560 R3, digitalPin 22~ 37 used as row0 ~ row 15, 
digital pin 38~53 used as column 0 ~ 15,
it's a 16 * 16  matrix, 

row 0, 1, 2 ,3 will be used to support 32 rotary encoder
row 4, 5 will be used to support 16 On - off - On toggle switches, 
note: this application will make the On-off-on toggle switch generate a button push signal when toggle from On to off, 
for normal on- on toggle switch or if you don't need this feature then just plug the switch to push button area
row 6~15 will be used to support push button or normal on-on toggle switch
*/

#define NUM_BUTTONS 256
#define NUM_AXES	8	       // 8 axes, X, Y, Z, etc

typedef struct joyReport_t {
	int16_t axis[NUM_AXES];
	uint8_t button[(NUM_BUTTONS+7)/8]; // 8 buttons per byte
};

joyReport_t joyReport;

/*
RotaryEncoder
BA BA BA BA
CW >>>  00 01 11 10    <<< CCW
*/

/*
this value sets the threshold for the CW/CCW drift in milliseconds
any direction drift within this time limit will be ignored, the previous direction will be used.
*/
byte CWCCWDriftTimeLimit = 60;

//totally 32 rotary encoders, 
const byte ENCODER_COUNT = 32;

//store the rotary encoder's AB pin value in last scan
byte prevEncoderValue[ENCODER_COUNT];

//store the rotary encoder's last effective CW/ CCW state
int prevEncoderDirection[ENCODER_COUNT];

//indicate the current stage  of thie encoder
// for every pulse, left shift stage 2 bits and append the new 2 bits at the lower end, 
// and compare the value with B00011110 and B00101101, then it's CW or CCW, 
// if it's CW or CCW, reset stage to 0
byte EncoderState[ENCODER_COUNT];

const byte CWPattern1 = B00011110, CWPattern2 = B01111000, CWPattern3 = B11100001, CWPattern4 = B10000111;
const byte CCWPattern1 = B00101101, CCWPattern2 = B10110100, CCWPattern3 = B11010010, CCWPattern4 = B01001110;

//store the rotary encoder's last pulse time, this will be used to remove the drift effect
unsigned long prevEncoderMillis[ENCODER_COUNT];

//unsigned long loopTime;

void setup() 
{
	for( int portId = 22; portId < 38; portId ++ )
	{
		pinMode( portId, OUTPUT );
	}
	for( int portId = 38; portId < 54; portId ++ )
	{
		pinMode( portId, INPUT_PULLUP);
	}
	PORTD = B10000000;
	PORTG = B00000111;
	PORTL = B11111111;
	PORTB = B00001111;

	Serial.begin(115200);
	delay(200);

	/*================axis and key state initialization=======================*/
	for (uint8_t ind=0; ind < NUM_AXES; ind++) {
		joyReport.axis[ind] = 0;
	}

	for (uint8_t ind=0; ind<sizeof(joyReport.button); ind++) {
		joyReport.button[ind] = 0;
	}

	for ( int i = 0; i < ENCODER_COUNT; i++)
	{
		prevEncoderMillis[i] = millis();
	}
	/*---------------------------------------*/

}

// Send an HID report to the USB interface
void sendJoyReport(struct joyReport_t *report)
{
	//check if time has elapsed more then 5mS since last report sent.
	long static prevMillis;
	if ( millis() - prevMillis >= 10 )
	{
		Serial.write((uint8_t *)report, sizeof(joyReport_t));

		//clear the pulse buttons state
		for( int i = 0; i < 8; i ++ )
		{
			joyReport.button[i] = 0;
		}
		prevMillis = millis();
	}
}

char oldMsg[10];

void loop() 
{
	//Serial.println( millis() - loopTime );
	//loopTime = millis();

	//scan rotary encoder
	for ( int rowid = 0; rowid < 4; rowid ++ )
	{
		//turn off all rows first
		PORTA = 0xFF;  PORTC = 0xFF;
		//turn on the current row
		PORTA &= ~(0x1 << rowid);

		byte colResult[16];

		//	//pin 38, PD7
		//	colResult[0] = (PIND & B10000000 )== 0 ? 1 : 0;
		//	//pin 39, PG2
		//	colResult[1] = (PING & B00000100 )== 0 ? 1 : 0;
		//	//pin 40, PG1
		//	colResult[2] = (PING & B00000010 )== 0 ? 1 : 0;
		//	//pin 41, PG0
		//	colResult[3] = (PING & B00000001 )== 0 ? 1 : 0;

		//	//pin 42, PL7
		//	colResult[4] = (PINL & B10000000 )== 0 ? 1 : 0;
		//	//pin 43, PL6
		//	colResult[5] = (PINL & B01000000 )== 0 ? 1 : 0;
		//	//pin 44, PL5
		//	colResult[6] = (PINL & B00100000 )== 0 ? 1 : 0;
		//	//pin 45, PL4
		//	colResult[7] = (PINL & B00010000 )== 0 ? 1 : 0;

		//	//pin 46, PL3
		//	colResult[8] = (PINL & B00001000 )== 0 ? 1 : 0;
		//	//pin 47, PL2
		//	colResult[9] = (PINL & B00000100 )== 0 ? 1 : 0;
		//	//pin 48, PL1
		//	colResult[10] =( PINL & B00000010)== 0 ? 1 : 0;
		//	//pin 49, PL0
		//	colResult[11] =( PINL & B00000001)== 0 ? 1 : 0;

		//	//pin 50, PB3
		//	colResult[12] =( PINB & B00001000)== 0 ? 1 : 0;
		//	//pin 51, PB2
		//	colResult[13] =( PINB & B00000100)== 0 ? 1 : 0;
		//	//pin 52, PB1
		//	colResult[14] =( PINB & B00000010)== 0 ? 1 : 0;
		//	//pin 53, PB0
		//	colResult[15] =( PINB & B00000001)== 0 ? 1 : 0;

		for ( int colid = 0; colid < 16; colid += 2 )
		{
			//byte encoderValue = colResult[colid] << 1 | colResult[colid + 1];
			byte encoderValue = digitalRead(colid+38) << 1 | digitalRead(colid + 39);
			byte encoderId = rowid * 8 + colid / 2;
			//int encoderState = encoderTable[ prevEncoderValue[ encoderId ] << 2 | encoderValue ];

			if ( encoderValue != prevEncoderValue[encoderId]  )
			{
				//we have a "pulse" from rotary encoder
				int direction = 0;
				EncoderState[encoderId] = EncoderState[encoderId] << 2 | encoderValue;

				if (EncoderState[encoderId] == CWPattern1  || EncoderState[encoderId] == CWPattern3 )
				{
					//this is a full CW pattern,
					direction = 1;
					//EncoderState[encoderId] = 0;
				}
				else if (EncoderState[encoderId] == CCWPattern1  || EncoderState[encoderId] == CCWPattern3 )
				{
					//this is a full CCW pattern,
					direction = -1;
				//	EncoderState[encoderId] = 0;
				}
				prevEncoderValue[encoderId ] = encoderValue;

				unsigned long timeDiff = millis() - prevEncoderMillis[encoderId];
				if ( timeDiff < CWCCWDriftTimeLimit && direction != 0 )
				{
					// we get a CW/CCW in less then a very short time, let's check if the current direction is identical with the previous direction
					if ( prevEncoderDirection[encoderId] != direction && prevEncoderDirection[encoderId] != 0 )
					{
						//the current direction is different with the previous one, let's use the previous direction instead
						direction = prevEncoderDirection[encoderId];
					}
				}

				if ( direction == 1 )
				{
					//set bit to 10
					joyReport.button[ rowid * 2 + colid / 8 ] |= 0x1 << ( colid % 8 + 1);
					joyReport.button[ rowid * 2 + colid / 8 ] &= ~(0x1 << ( colid % 8 ));
					prevEncoderMillis[ encoderId ] = millis();
					prevEncoderDirection[ encoderId ] = direction;
				}
				else if ( direction == -1 )
				{
					//set bit to 01
					joyReport.button[ rowid * 2  + colid / 8 ] &= ~(0x1 << ( colid % 8 + 1));
					joyReport.button[ rowid * 2 + colid / 8 ] |= 0x1 << ( colid % 8 );
					prevEncoderMillis[ encoderId ] = millis();
					prevEncoderDirection[ encoderId ] = direction;
				}
			}

		}
	}

	for ( int rowid = 4; rowid < 16; rowid ++ )
	{
		//turn off all rows first
		PORTA = 0xFF;  PORTC = 0xFF;
		//turn on the current row
		if (rowid < 8)
		{
			PORTA &= ~(0x1 << rowid);
		}
		else
		{
			PORTC &= ~(0x1 << (16 - rowid) );
		}
		//digitalWrite( rowid + 22, LOW );
		byte colResult[16];
		//pin 38, PD7
		colResult[0] = (PIND & B10000000)== 0 ? 1 : 0;
		//pin 39, PG2
		colResult[1] = (PING & B00000100)== 0 ? 1 : 0;
		//pin 40, PG1
		colResult[2] = (PING & B00000010)== 0 ? 1 : 0;
		//pin 41, PG0
		colResult[3] = (PING & B00000001)== 0 ? 1 : 0;

		//pin 42, PL7
		colResult[4] = (PINL & B10000000)== 0 ? 1 : 0;
		//pin 43, PL6
		colResult[5] = (PINL & B01000000)== 0 ? 1 : 0;
		//pin 44, PL5
		colResult[6] = (PINL & B00100000)== 0 ? 1 : 0;
		//pin 45, PL4
		colResult[7] = (PINL & B00010000)== 0 ? 1 : 0;

		//pin 46, PL3
		colResult[8] = (PINL & B00001000)== 0 ? 1 : 0;
		//pin 47, PL2
		colResult[9] = (PINL & B00000100)== 0 ? 1 : 0;
		//pin 48, PL1
		colResult[10] =(PINL & B00000010) == 0 ? 1 : 0;
		//pin 49, PL0
		colResult[11] =(PINL & B00000001) == 0 ? 1 : 0;

		//pin 50, PB3
		colResult[12] =(PINB & B00001000) == 0 ? 1 : 0;
		//pin 51, PB2
		colResult[13] =(PINB & B00000100) == 0 ? 1 : 0;
		//pin 52, PB1
		colResult[14] =(PINB & B00000010) == 0 ? 1 : 0;
		//pin 53, PB0
		colResult[15] =(PINB & B00000001) == 0 ? 1 : 0;

		for ( int colid = 0; colid < 16; colid ++ )
		{
			if ( colResult[ colid ] == 1 )
				//if ( digitalRead( colid + 38 ) == HIGH )
			{
				joyReport.button[ rowid * 2 + colid / 8 ] |= (0x1 << ( colid % 8 ));
			}
			else
			{
				joyReport.button[ rowid * 2 + colid / 8 ] &= ~(0x1 << ( colid % 8 ));
			}
		}
	}

	/* Move all of the axes */
	for (uint8_t ind=0; ind< NUM_AXES; ind++) {
		//joyReport.axis[ind] = map(analogRead(54+ind), 0, 1023, -32768,32767 );
	}
	sendJoyReport(&joyReport);
}
