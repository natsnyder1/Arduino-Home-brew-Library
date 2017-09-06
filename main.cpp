/*
 * Arduino_Home_Brew.cpp
 *
 * Created: 4/4/2017 8:33:31 AM
 * Author : natsn
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>



// Peripherals
class LCD
{
	private:
	unsigned char volatile *  DataDir_MrLCDsCrib;
	unsigned char volatile *  DataDir_MrLCDsControl;
	
	unsigned char volatile *  MrLCDsCrib;
	unsigned char volatile *  MrLCDsControl;
	long watchDog = 0;
	const unsigned int watchDogIterations = 5000;
	const uint8_t LCDCOUNTS = 32;
	uint8_t LightSwitch;
	uint8_t	ReadWrite;
	uint8_t	BiPolarMood;
	void Check_IF_MrLCD_isBusy()
	{
		*DataDir_MrLCDsCrib = 0;
		*MrLCDsControl |= (1<<ReadWrite);
		*MrLCDsControl &= ~(1<<BiPolarMood);
		watchDog = 0;
		while(*MrLCDsCrib >= 0x80 )
		{
			watchDog++;
			Peek_A_Boo();
			if (watchDog > watchDogIterations)
			{
				DDRA |= (1<<PINA5);
				PORTA ^= (1<<PINA5);
				_delay_ms(10);
				PORTA ^= (1<<PINA5);
				//setUpTheLCD();
				break;
			}
		}
		*DataDir_MrLCDsCrib = 0xFF;
	}
	void Peek_A_Boo(void)
	{
		*MrLCDsControl|= (1<<LightSwitch);
		_delay_us(100);
		*MrLCDsControl &= ~(1<<LightSwitch);
	}
	void Send_A_Command(unsigned char command)
	{
		Check_IF_MrLCD_isBusy();
		*MrLCDsCrib = command;
		// Turn the Control off
		*MrLCDsControl &= ~(1<<ReadWrite);
		*MrLCDsControl &= ~(1<<BiPolarMood);
		Peek_A_Boo();
		*MrLCDsCrib = 0;
		
	}
	void Send_A_Character(unsigned char character)
	{
		Check_IF_MrLCD_isBusy();
		*MrLCDsCrib = character;
		*MrLCDsControl &= ~(1<<ReadWrite);
		*MrLCDsControl |= (1<<BiPolarMood);
		Peek_A_Boo();
		*MrLCDsCrib	= 0;
	}
	public:
	// Should take which port it's connected to, and the reset, read/write, and enable
	LCD(char _dataBusPort, char _commandBusPort, uint8_t _LightSwitch, uint8_t _ReadWrite, uint8_t _BiPolarMood)
	{
		// Set the data bus's pointer to send information to the LCD
		if (_dataBusPort == 'A')
		{
			 DataDir_MrLCDsCrib = &DDRA;
			 MrLCDsCrib = &PORTA;
		}
		else if(_dataBusPort == 'B')
		{
			 DataDir_MrLCDsCrib = &DDRB;
			 MrLCDsCrib = &PORTB;
			// Test
		}
		else if(_dataBusPort == 'C')
		{
			 DataDir_MrLCDsCrib = &DDRC;
			 MrLCDsCrib =  &PORTC;

		}
		else if(_dataBusPort == 'D')
		{
			 DataDir_MrLCDsCrib = &DDRD;
			 MrLCDsCrib =  &PORTD;
		}
		else
		{
			// Something went wrong
			 DataDir_MrLCDsCrib = 0;
			 MrLCDsCrib = 0;
		}
		
		// Set the command bus ddr and port
		if(_commandBusPort == 'A')
		{
			DataDir_MrLCDsControl = &DDRA;
			MrLCDsControl =  &PORTA;
		}
		else if(_commandBusPort == 'B')
		{
			DataDir_MrLCDsControl =  &DDRB;
			MrLCDsControl =  &PORTB;

		}
		else if(_commandBusPort == 'C')
		{
			DataDir_MrLCDsControl =  &DDRC;
			MrLCDsControl =  &PORTC;
		}
		else if(_commandBusPort == 'D' )
		{
			DataDir_MrLCDsControl =  &DDRD;
			MrLCDsControl =  &PORTD;
		}
		else
		{
			// Something went wrong
			DataDir_MrLCDsControl = 0;
			MrLCDsControl = 0;
		}
		
		// Set the command bus pins
		LightSwitch = _LightSwitch;
		BiPolarMood = _BiPolarMood;
		ReadWrite = _ReadWrite;
		
		// In order to Use PORT C fully, disable the JTD PIN
		MCUCR |= (1<<JTD);
		MCUCR |= (1<<JTD);
	}
	
	void Send_A_String( const char* letter)
	{
		//setUpTheLCD();
		int count = 0;
		bool gate = true;
		watchDog = 0;
		while(*letter != 0)
		{
			watchDog++;
			Send_A_Character(*letter);
			letter++;
			count++;
			if(count > 15 && gate)
			{
				gate = false;
				count = 40;
				Send_A_Command(0x80 + count);
			}
			
			if (watchDog > LCDCOUNTS)
			{
				DDRA |= (1<<PINA5);
				PORTA ^= (1<<PINA5);
				//setUpTheLCD();
				_delay_ms(10);
				PORTA ^= (1<<PINA5);
				break;
			}
			
		}
	}

	// ANY String to Position function requires you to setUpTheLCD MANUALLY. This is because 
	// using this function is associated with writing things on different parts of the screen
	
	void Send_A_String_To_Position(const char* letter, uint8_t row, uint8_t column)
	{
		if (row == 1) // If they want the first row then the formula is row + column = location
		{
			row = 0;
		}
		if (row == 2)
		{
			row = 39; // Really, 40 is the first number command that writes to the second row for some reason
		}
		int startingPosition = row + column;
		Send_A_Command(0x80 + startingPosition);
		watchDog = 0;
		while(*letter != 0)
		{
			watchDog++;
			Send_A_Character(*letter);
			_delay_us(10);
			letter++;
			if (watchDog > LCDCOUNTS)
			{
				DDRA |= (1<<PINA5);
				PORTA ^= (1<<PINA5);
				//setUpTheLCD();
				_delay_ms(10);
				PORTA ^= (1<<PINA5);
				break;
			}
		}
		
	}
	
	void Send_An_Integer_To_Position_Decimal(int value, uint8_t row, uint8_t column)
	{
		if (row == 1)
		{
			row = 0;
		}
		else if (row == 2)
		{
			row = 39;
		}
		else // they fucked up and you gotta lend a hand
		{
			row = 0;
			column = 1;
		}
		int screenPosition = row + (column-1); // Column starts at zero
		char stringValue[4];
		itoa(value,stringValue,10);
		Send_A_Command(0x80 + screenPosition); // Put first character starting position
		char* strPtr = stringValue;
		int positionChecker = screenPosition;
		watchDog = 0;
		while (*strPtr != 0)
		{
			watchDog++;
			if (positionChecker >= 16 && positionChecker < 40)
			{
				positionChecker = 40;
				Send_A_Command(0x80 + positionChecker);
				_delay_us(10);
			}
			Send_A_Character(*strPtr);
			_delay_us(10);
			strPtr++;
			positionChecker++;
			if (watchDog > LCDCOUNTS)
			{
				DDRA |= (1<<PINA5);
				PORTA ^= (1<<PINA5);
				_delay_ms(10);
				PORTA ^= (1<<PINA5);
				//setUpTheLCD();
				break;
			}
		}
		
	}
	// Use this one to convert 
	void Send_An_Integer_To_Position_Binary(uint8_t value, uint8_t row, uint8_t column)
	{
		if (row == 1)
		{
			row = 0;
		}
		else if (row == 2)
		{
			row = 39;
		}
		else // they fucked up and you gotta lend a hand
		{
			row = 0;
			column = 1;
		}
		int screenPosition = row + column;
		char stringValue[4];
		itoa(value,stringValue,2);
		Send_A_Command(0x80 + screenPosition); // Put first character starting position
		char* strPtr = stringValue;
		int positionChecker = screenPosition;
		watchDog = 0;
		while (*strPtr != 0)
		{
			watchDog++;
			if (positionChecker >= 16 && positionChecker < 40)
			{
				positionChecker = 40;
				Send_A_Command(0x80 + positionChecker);
			}
			Send_A_Character(*strPtr);
			strPtr++;
			positionChecker++;
			if (watchDog > LCDCOUNTS)
			{
				DDRA |= (1<<PINA5);
				PORTA ^= (1<<PINA5);
				_delay_ms(10);
				PORTA ^= (1<<PINA5);
				//setUpTheLCD();
				break;
			}
		}	
	}
	
	void setUpTheLCD()
	{
		*DataDir_MrLCDsControl |= (1<<LightSwitch) | (1<<ReadWrite) | (1<<BiPolarMood);
		_delay_ms(5);
		Send_A_Command(0x01);
		_delay_ms(5);
		Send_A_Command(0x38); // Set 8 Bit Mode
		_delay_ms(5);
		Send_A_Command(0b00001110);
		_delay_ms(5);
	}

};
class NonContinousServo
{
	// Uses the timer the user specifies
	// Uses the pin number the user specifies
	// Assumed to be a frequency of 50 Hz
	// 1,000,000 cycles per second. IRC1 = 20,000 cycles. OCR1A = mappedDegrees * Gain
	private:
	char servoOutputComparePin;
	uint8_t timerSelection;
	bool setUpTimer;
	public:
	NonContinousServo(uint8_t timer, char _servoOutputComparePin) // Choose an output compare pin OCRx
	{
		servoOutputComparePin = _servoOutputComparePin;
		timerSelection = timer;
		// In order to Use PORT C fully, disable the JTD PIN
		MCUCR |= (1<<JTD);
		MCUCR |= (1<<JTD);
		setUpTimer = false;
	}
	// Turn Servo To specific Angle
	void rotateDegrees(int angle)
	{
		// A pulse of .5ms over 20 ms is a 2.5% duty cycle and will be 0 degrees
		// A pulse of 1.5ms over 20 ms is a 7.5% duty cycle and will be 90 degrees
		// A pulse of 2.5ms over 20 ms is a 12.5% duty cycle and will be 180 degrees
		
		
		// Timer 0 Selection
		if (timerSelection == 0)
		{
			
			if (setUpTimer == false)
			{
				// Set the timer up
				if (servoOutputComparePin == 'A')
				{ // Setup output compare toggle for the
					const uint8_t pinOCRA = 3;
					DDRB |= (1<<pinOCRA); // Set pin to output
					// Sets High on OCRA match and Low on bottom of timer
					TCCR0A |= (1<<COM0A1) | (1<<COM0A0) | (1<<WGM00) | (1<<WGM01); // Set Output compare toggle on pin 7
					TCCR0B |= (1<<CS01) | (1<<CS00); // 64 prescale for OCRA compare
				}
				else if (servoOutputComparePin == 'B')
				{
					const uint8_t pinOCRB = 4;
					DDRB |= (1<<pinOCRB); // Set pin to output
					// Sets High on OCRA match and Low on bottom of timer
					TCCR0A |= (1<<COM0B1) | (1<<COM0B0) | (1<<WGM00) | (1<<WGM01); // Set Output compare toggle on pin 7
					TCCR0B |= (1<<CS01) | (1<<CS00); // 64 prescale for OCRB compare
				}
				setUpTimer = true;
			}
			
			// OCR0A/B [0-255] : .025 * 255 is ~6 and .125*255 is 32
			const uint8_t  RANGEDEGREES = 180;
			const int RANGEPWM = 26; // 6 - 32
			const uint16_t OFFSET = 6;
			if(angle > RANGEDEGREES) angle = 180;
			if(angle < 0) angle = 0;
			
			// Map Value
			float normalizedAngle = (float) angle / RANGEDEGREES; // [0,1]
			int PWMwrite = normalizedAngle * RANGEPWM + OFFSET;
			if(servoOutputComparePin == 'A') OCR0A = 255 - PWMwrite;
			if(servoOutputComparePin == 'B') OCR0B = 255 - PWMwrite;
		}
		
		// Timer 1 Setup
		if (timerSelection == 1)
		{ 
			
			if (setUpTimer == false)
			{
				// Set the timer up
				if (servoOutputComparePin == 'A')
				{ // Setup output compare toggle for the 
					const uint8_t pinOCRA = 5;
					DDRD |= (1<<pinOCRA); // Set pin to output
					TCCR1A |= (1<<WGM11) | (1<<COM1A1) ; // Set Output compare toggle on pind5
					TCCR1B |= (1<<WGM12) | (1 << WGM13) | (1<<CS10);
					ICR1 = 19999;
				}
				else if (servoOutputComparePin == 'B')
				{
					const uint8_t pinOCRB = 4;
					
					DDRD |= (1<<pinOCRB); // Set pin to output
					TCCR1A |= (1<<WGM11) | (1<<COM1B1) ; // Set Output compare toggle on pind5
					TCCR1B |= (1<<WGM12) | (1 << WGM13) | (1<<CS10);
					ICR1 = 19999;
				}
				setUpTimer = true;
			}
			
			// OCR1A Values are from 300- 2300 [I think], therefore we need to map 0-180 to 300-2300
			const uint8_t  RANGEDEGREES = 180;
			const int RANGEPWM = 2000; // 300 - 2300
			const uint16_t OFFSET = 300;
			if(angle > RANGEDEGREES) angle = 180;
			if(angle < 0) angle = 0;
			
			// Map Value
			float normalizedAngle = (float) angle / RANGEDEGREES; // [0,1]
			int PWMwrite = normalizedAngle * RANGEPWM + OFFSET;
			if(servoOutputComparePin == 'A') OCR1A = PWMwrite;
			if(servoOutputComparePin == 'B') OCR1B = PWMwrite;
		}
		
		// Timer 2 Selection
		if (timerSelection == 2)
		{
			
			if (setUpTimer == false)
			{
				// Set the timer up
				if (servoOutputComparePin == 'A')
				{ // Setup output compare toggle for the
					const uint8_t pinOCRA = 7;
					DDRD |= (1<<pinOCRA); // Set pin to output
					// Sets High on OCRA match and Low on bottom of timer
					TCCR2A |= (1<<WGM21) | (1<<COM2A1) | (1<<COM2A0) | (1<<WGM20); // Set Output compare toggle on pin 7
					TCCR2B |= (1<<CS22); // 64 prescale for OCRA compare
				}
				else if (servoOutputComparePin == 'B')
				{
					const uint8_t pinOCRB = 6;
					DDRD |= (1<<pinOCRB); // Set pin to output
					// Sets High on OCRA match and Low on bottom of timer
					TCCR2A |= (1<<WGM21) | (1<<COM2B1) | (1<<COM2B0) | (1<<WGM20); // Set Output compare toggle on pin 7
					TCCR2B |= (1<<CS22); // 64 prescale for OCRA compare
				}
				setUpTimer = true;
			}
			
			// OCR2A/B [0-255] : .025 * 255 is ~6 and .125*255 is 32
			const uint8_t  RANGEDEGREES = 180;
			const int RANGEPWM = 26; // 6 - 32
			const uint16_t OFFSET = 6;
			if(angle > RANGEDEGREES) angle = 180;
			if(angle < 0) angle = 0;
			
			// Map Value
			float normalizedAngle = (float) angle / RANGEDEGREES; // [0,1]
			int PWMwrite = normalizedAngle * RANGEPWM + OFFSET;
			if(servoOutputComparePin == 'A') OCR2A = 255 - PWMwrite;
			if(servoOutputComparePin == 'B') OCR2B = 255 - PWMwrite;
		}
	}
	
	// Maps positive numbers
	int mapVal(int valueToMap, int ilow, int ihigh, int olow, int ohigh)
	{
		int i_range = ihigh - ilow;
		int o_range = ohigh - olow;
		valueToMap -= ilow;
		int mappedValue = ((float) valueToMap / i_range) * o_range;
		// Add offset and return to user
		mappedValue += olow;
		return mappedValue;
	}
	
};
class AnalogToDigitalConverter
{	
	private:
	
	public:
	uint8_t channel;
	uint16_t currentVal;
	AnalogToDigitalConverter(uint8_t _channel)
	{
		ADCSRA |= (1<<ADPS2); // Enable prescale for ADC clock of 50-200KHz range
		channel = _channel;
		DDRA &= ~(1<<_channel);
		ADMUX |= (1<<REFS0);
		
		// In order to Use PORT C fully, disable the JTD PIN
		MCUCR |= (1<<JTD);
		MCUCR |= (1<<JTD);
	}
	
	void switchToMyChannel()
	{
		// Configure the ADMUX and the ADCSRA
		// Clear the current Admux channel selection and put the new one in
		ADMUX &= ~(1<<MUX0);
		ADMUX &= ~(1<<MUX1);
		ADMUX &= ~(1<<MUX2);
		if(channel == 0){ADMUX &= ~(1<<MUX0);} // ADC0
		if(channel == 1){ADMUX |= (1<<MUX0);} // ADC1
		if(channel == 2){ADMUX |= (1<<MUX1);} // ADC2
		if(channel == 3){ADMUX |= (1<<MUX0) | (1<<MUX1);} // ADC3
		if(channel == 4){ADMUX |= (1<<MUX2);} // ADC4
		if(channel == 5){ADMUX |= (1<<MUX2) | (1<<MUX0);} // ADC5
		if(channel == 6){ADMUX |= (1<<MUX2) | (1<<MUX1);} // ADC6
		if(channel == 7){ADMUX |= (1<<MUX2) | (1<<MUX1) | (1<<MUX0);}
	}
	int readADC()
	{
		// Start ADC
		ADCSRA |= (1<<ADPS2);
		ADCSRA |= (1<<ADEN);
		ADCSRA |= (1<<ADSC);
		
		// This will loop until the conversion is complete
		 // wait until conversion complete ADSC=0 -> Complete
		while (ADCSRA & (1<<ADSC));
		
		uint8_t theLow = ADCL;
		uint16_t ADCValue = (ADCH << 8) | theLow;
		
		// Stop ADC
		ADCSRA &= ~(1<<ADEN);
		ADCSRA &= ~(1<<ADSC);
		currentVal = ADCValue;
		return ADCValue;
	}
	int runningAverageReadADC(const uint8_t ITERATIONS )
	{
		float sum = 0;

		// Start ADC
		ADCSRA |= (1<<ADPS2);
		ADCSRA |= (1<<ADEN);
		
		// This will loop until the conversion is complete
		// wait until conversion complete ADSC=0 -> Complete
		while (ADCSRA & (1<<ADSC));
		
		uint8_t theLow;
		uint16_t ADCValue;
		for (int i = 0; i < ITERATIONS; i++)
		{
			ADCSRA |= (1<<ADSC);
			while (ADCSRA & (1<<ADSC));
			theLow = ADCL;
			ADCValue = (ADCH << 8) | theLow;
			sum += ADCValue;
			ADCSRA &= ~(1<<ADSC);
		}
		// Stop ADC
		ADCSRA &= ~(1<<ADEN);
		ADCSRA &= ~(1<<ADSC);
		sum = sum / ITERATIONS;
		currentVal = sum;
		return sum;
	}
	void PIDFilterReadADC(const uint8_t ITERATIONS)
	{
		// Configure the ADMUX and the ADCSRA
		// Clear the current Admux channel selection and put the new one in
		ADMUX &= ~(1<<0);
		ADMUX &= ~(1<<1);
		ADMUX &= ~(1<<2);
		ADMUX |= (1<<channel);
		ADMUX |= (1<<REFS0);
		// Finish...
	}
};
class DCMotor
{
	private:
	uint8_t timer;
	uint8_t forwardPin;
	uint8_t reversePin;
	char OCRxPinEnable;
	unsigned char volatile * MotorDDR;
	unsigned char volatile * MotorPort;
	unsigned char volatile * OCR_ControllerDDR;
	unsigned char volatile * OCR_ControllerPORT;
	
	void setUpTimerForOCRCompare()
	{
		// Set up Timer
		if (OCRxPinEnable == 'A')
		{ // Setup output compare toggle for the
			if(timer == 0)
			{
				OCR_ControllerDDR = &DDRB;
				OCR_ControllerPORT = &PORTB;
				// Broken, need to fix for timers 0 and 2
				*OCR_ControllerDDR   |= (1<<PINB3); // Enable the OCR1 pin
				TCCR0A |= (1<<COM0A1) | (1<<COM0A0) | (1<<WGM01) | (1<<WGM00); // Set Output compare toggle on pind5
				TCCR0B |= (1<<CS01) | (1<<CS00); // 64 Pre-scale, Hz is 60 cycles /sec -- Changed from CS02
			}
			else if(timer == 1)
			{
				OCR_ControllerDDR = &DDRD;
				OCR_ControllerPORT = &PORTD;
				*OCR_ControllerDDR |= (1<<PIND5);
				DDRD   |= (1<<PIND5); // Enable the OCR1 pin
				TCCR1A |= (1<<WGM11) | (1<<COM1A1) | (1<< COM1A0); // Set Output compare toggle on pind5 with inverting mode
				TCCR1B |= (1<<WGM12) | (1 << WGM13) | (1<<CS10);
				ICR1 = 19999;
				
			}
			else if(timer == 2)
			{
				// Set OCR controller to correct port
				OCR_ControllerDDR = &DDRD;
				OCR_ControllerPORT = &PORTD;
				// OC2A will be toggled upon compare
				*OCR_ControllerDDR   |= (1<<PIND7);
				// Set Up Timer 
				TCCR2A |= (1<<COM2A1) | (1<<COM2A0) |(1<<WGM21) | (1<<WGM20); // Fast PWM, Set on match, clear at bottom
				// Turn Timer on
				TCCR2B |= (1<<CS22); // A prescale of 64. 60 cycles / sec
			}
			else exit(1);
		}
		else if (OCRxPinEnable == 'B')
		{
			if(timer == 0)
			{
				OCR_ControllerDDR = &DDRB;
				OCR_ControllerPORT = &PORTB;
				*OCR_ControllerDDR |= (1<<PINB4);
				TCCR0A |= (1<<COM0B1) | (1<<COM0B0) | (1<<WGM01) | (1<<WGM00); // Set Output compare toggle on pind5
				TCCR0B |= (1<<CS01) | (1<<CS00); // 64 Pre-scale, Hz is 60 cycles /sec -- Change from CS02
			}
			else if(timer == 1)
			{
				OCR_ControllerDDR = &DDRD;
				OCR_ControllerPORT = &PORTD;
				*OCR_ControllerDDR |= (1<<PIND4);
				TCCR1A |= (1<<WGM11) | (1<<COM1B1) | (1<< COM1B0); // Set Output compare toggle on pind5 with inverting mode
				TCCR1B |= (1<<WGM12) | (1 << WGM13) | (1<<CS10);
				ICR1 = 19999;
			}
			else if(timer == 2)
			{
				// Set OCR controller to correct port
				OCR_ControllerDDR = &DDRD;
				OCR_ControllerPORT = &PORTD;
				// OC2A will be toggled upon compare
				*OCR_ControllerDDR   |= (1<<PIND6);
				// Set Up Timer
				TCCR2A |= (1<<COM2B1) | (1<<COM2B0) |(1<<WGM21) | (1<<WGM20); // Fast PWM, Set on match, clear at bottom
				TCCR2B &= ~(1<<WGM22); // Make sure this bit is off.
				// Turn Timer on
				TCCR2B |= (1<<CS22); // A prescale of 64. Frequency of 60 cycles / sec
				
			}
			else exit(1);
		}
	}
	public:
	DCMotor(char _PORT, uint8_t _timer, char _OCRxEnable, uint8_t _forwardPin, uint8_t _reversePin)
	{
		// Parameter Capture 
		timer = _timer;
		forwardPin = _forwardPin;
		reversePin = _reversePin;
		OCRxPinEnable = _OCRxEnable;
		
			 if (_PORT == 'A'){MotorDDR = &DDRA; MotorPort = &PORTA;}
		else if (_PORT == 'B'){MotorDDR = &DDRB; MotorPort = &PORTB;}
		else if (_PORT == 'C'){MotorDDR = &DDRC; MotorPort = &PORTC;}
		else if (_PORT == 'D'){MotorDDR = &DDRD; MotorPort = &PORTD;}
		
		// In order to Use PORT C fully, disable the JTD PIN
		MCUCR |= (1<<JTD);
		MCUCR |= (1<<JTD);
		
		// Initialize Timer 
		setUpTimerForOCRCompare();
	}
	void goForwards()
	{
		// Set Directions
		*MotorDDR |= (1<<forwardPin);
		*MotorDDR |= (1<<reversePin);
		*MotorPort |= (1<<forwardPin);
		*MotorPort &= ~(1<<reversePin);
	}
	
	void goReverse()
	{
		
	}
	
	void pwmSpeed(int speed, uint8_t offset = 0) // [-100, to 100] Negative means reverse and Positive is forwarads
	{
		if (speed >= 0)
		{
			// Turn pins on for forwards driving
			*MotorDDR  |= (1<<reversePin);
			*MotorPort &= ~(1<<reversePin);
			*MotorDDR  |= (1<<forwardPin);
			*MotorPort |= (1<<forwardPin);
		}
		else
		{
			// Turn pins on for Reverse driving
			*MotorDDR  |= (1<<reversePin);
			*MotorPort |= (1<<reversePin);
			*MotorDDR  |= (1<<forwardPin);
			*MotorPort &= ~(1<<forwardPin);
			speed *= -1;
		}
		
		// Initialize variables for PWM mapping
		const uint8_t  RANGEPERCENT = 100;
		long RANGEPWM;
		float normalizedAngle;
		long PWMwrite;
		
		if(speed > RANGEPERCENT) speed = RANGEPERCENT;
		if(speed < 0) speed = 0;
		
		// Map Value
		if(timer == 1) // Use range of 65535;
		{
			RANGEPWM = 19999; // The value of ICR1
			normalizedAngle = ((float) speed) / RANGEPERCENT; // [0,1]
			PWMwrite = (normalizedAngle * RANGEPWM) + offset;
			
		}
		else
		{
			RANGEPWM = 255;
			normalizedAngle = ((float) speed) / RANGEPERCENT; // [0,1]
			PWMwrite = (normalizedAngle * RANGEPWM) + offset;
		}
		
		if(OCRxPinEnable == 'A' && timer == 0) OCR0A = RANGEPWM - PWMwrite;
		if(OCRxPinEnable == 'A' && timer == 1) OCR1A = RANGEPWM - PWMwrite;
		if(OCRxPinEnable == 'A' && timer == 2) OCR2A = RANGEPWM - PWMwrite;
		if(OCRxPinEnable == 'B' && timer == 0) OCR0B = RANGEPWM - PWMwrite;
		if(OCRxPinEnable == 'B' && timer == 1) OCR1B = RANGEPWM - PWMwrite;
		if(OCRxPinEnable == 'B' && timer == 2) OCR2B = RANGEPWM - PWMwrite;
		
	}
 
	int mapVal(int valueToMap, int ilow, int ihigh, int olow, int ohigh)
	{
		int i_range = ihigh - ilow;
		int o_range = ohigh - olow;
		valueToMap -= ilow;
		int mappedValue = ((float) valueToMap / i_range) * o_range;
		// Add offset and return to user
		mappedValue += olow;
		return mappedValue;
	}
	
};
class ADXL345
{
	private:
	uint8_t deviceID = 0x1D;
	uint8_t xAxisRegisterAddressL = 0x32;
	uint8_t xAxisRegisterAddressH = 0x33;
	uint8_t yAxisRegisterAddressL = 0x34;
	uint8_t yAxisRegisterAddressH = 0x35;
	uint8_t zAxisRegisterAddressL = 0x36;
	uint8_t zAxisRegisterAddressH = 0x37;
	
	long TWBR_val = ((1000000/50000) -16) / 2; // ((F_CPU/F_SCL)-16)/2
	uint8_t I2C_READ = 0x01;
	uint8_t I2C_WRITE = 0x00;
	uint8_t ADXL345_WRITE = 0x3A;
	uint8_t ADXL345_READ  = 0x3B;
	
	unsigned char volatile * DDRController = NULL;
	unsigned char volatile * PORTController = NULL;
	uint8_t successPin;
	uint8_t failedPin;
	
	void initializeTWI_I2C()
	{
		// Initialize I2C / TWI
		PRR0 &= ~(1<<PRTWI); // Turn off power reduction so I2C/TWI is not disabled
		TWCR &= ~(1<<TWIE); // Turn off interrupt enable
		TWBR = 2; // 1Mhz clock and I2c will be running at 50 kHz
		TWSR &= ~(1<<TWPS1) | (1 << TWPS0);
	}
		
	uint8_t i2c_start(uint8_t address)
	{
		// reset TWI control register
		TWCR = 0;
		// transmit START condition
		TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
		// wait for end of transmission
		while( !(TWCR & (1<<TWINT)) );
			
		// check if the start condition was successfully transmitted
		if((TWSR & 0xF8) != TW_START){ return 1; }
			
		// load slave address into data register
		TWDR = address;
		// start transmission of address
		TWCR = (1<<TWINT) | (1<<TWEN);
		// wait for end of transmission
		while( !(TWCR & (1<<TWINT)) );
			
		// check if the device has acknowledged the READ / WRITE mode
		uint8_t twst = TW_STATUS & 0xF8;
		if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;
			
		return 0;
	}
	uint8_t i2c_write(uint8_t data)
	{
		// load data into data register
		TWDR = data;
		// start transmission of data
		TWCR = (1<<TWINT) | (1<<TWEN);
		// wait for end of transmission
		while( !(TWCR & (1<<TWINT)) );
			
		if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){ return 1; }
			
		return 0;
	}
		
	uint8_t i2c_read_ack(void)
	{
			
		// start TWI module and acknowledge data after reception
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
		// wait for end of transmission
		while( !(TWCR & (1<<TWINT)) );
		// return received data from TWDR
		return TWDR;
	}
		
	uint8_t i2c_read_nack(void)
	{
			
		// start receiving without acknowledging reception
		TWCR = (1<<TWINT) | (1<<TWEN);
		// wait for end of transmission
		while( !(TWCR & (1<<TWINT)) );
		// return received data from TWDR
		return TWDR;
	}
	uint8_t i2c_transmit(uint8_t address, uint8_t* data, uint16_t length)
	{
		if (i2c_start(address | I2C_WRITE)) return 1;
			
		for (uint16_t i = 0; i < length; i++)
		{
			if (i2c_write(data[i])) return 1;
		}
			
		i2c_stop();
			
		return 0;
	}

	void i2c_stop(void)
	{
		// transmit STOP condition
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	}
		
	void transmissionCheck(uint8_t returned)
	{
		if(returned == 0) {*PORTController ^= (1<<successPin); _delay_ms(50);*PORTController ^= (1<<successPin); _delay_ms(50);}
		else {*PORTController ^= (1<<failedPin); _delay_ms(50); *PORTController ^= (1<<failedPin); _delay_ms(50);}
	}
		
	public:
	// Tell this class what port your two pins are connected to 
	ADXL345(char _PORT, uint8_t pinSuccessOperation, uint8_t pinFailedOperation)
	{
		// In order to Use PORT C fully, disable the JTD Debug Mode
		MCUCR |= (1<<JTD);
		MCUCR |= (1<<JTD);
		
		if (_PORT== 'A') {PORTController = &PORTA; DDRController = &DDRA; }
		if (_PORT== 'B') {PORTController = &PORTB; DDRController = &DDRB; }
		if (_PORT== 'C') {PORTController = &PORTC; DDRController = &DDRC; }
		if (_PORT== 'D') {PORTController = &PORTD; DDRController = &DDRD; }
		
		*DDRController |= (1 << pinSuccessOperation) | (1 << pinFailedOperation);
		successPin = pinSuccessOperation;
		failedPin = pinFailedOperation;
		initializeTWI_I2C();
	}
	
	uint16_t data[3]; // [x, y, z]
		
	void initializeADXL()
	{
		// Inizialization: Configure the device for 4g mode
		initializeTWI_I2C();
		transmissionCheck(i2c_start(ADXL345_WRITE));
		transmissionCheck(i2c_write(0x27)); // Make Sure Register for Activity/ Inactivity is turned off for no interrupts
		transmissionCheck(i2c_write(0x00));
		
		transmissionCheck(i2c_start(ADXL345_WRITE));
		transmissionCheck(i2c_write(0x2D));
		transmissionCheck(i2c_write(0x08));
		
		transmissionCheck(i2c_start(ADXL345_WRITE));
		transmissionCheck(i2c_write(0x31));
		transmissionCheck(i2c_write(0x09));
		
		i2c_stop();
	}
	
	void readData()
	{
		// Start at LSB of X Register and go up to the MSB of Z
		i2c_start(ADXL345_WRITE);
		i2c_write(0x32);
		i2c_stop();
		
		i2c_start(ADXL345_READ);
		uint16_t raw_x_low = ((uint8_t)i2c_read_ack());
		uint16_t raw_x = (i2c_read_ack()<<8) | raw_x_low;
		
		uint16_t raw_y_low = ((uint8_t)i2c_read_ack());
		uint16_t raw_y = (i2c_read_ack()<<8) | raw_y_low;
		
		uint16_t raw_z_low = ((uint8_t)i2c_read_ack());
		uint16_t raw_z = (i2c_read_nack()<<8) | raw_z_low;
		
		i2c_stop();
		
		
		data[0] = raw_x;
		data[1] = raw_y;
		data[2] = raw_z;
		
	}

	int mapVal(int valueToMap, int ilow, int ihigh, int olow, int ohigh)
	{
		int i_range = ihigh - ilow;
		int o_range = ohigh - olow;
		valueToMap -= ilow;
		int mappedValue = ((float) valueToMap / i_range) * o_range;
		// Add offset and return to user
		mappedValue += olow;
		return mappedValue;
	}
};
class LED
{
	private:
	unsigned char volatile * DDRController = NULL;
	unsigned char volatile * PORTController = NULL;
	
	uint8_t pin;
		
	public:
	LED(char _PORT, uint8_t _pin)
	{
		if(_PORT == 'A'){DDRController = &DDRA; PORTController = &PORTA;}
		if(_PORT == 'B'){DDRController = &DDRB; PORTController = &PORTB;}
		if(_PORT == 'C'){DDRController = &DDRC; PORTController = &PORTC;}
		if(_PORT == 'D'){DDRController = &DDRD; PORTController = &PORTD;}
					
		pin = _pin;		
		
		// Initialize the LED off
		*DDRController |= (1<<pin);
		*PORTController &= ~(1<<pin);
					
	};
	
	void turnOff()
	{
		// Turn LED off
		*DDRController |= (1<<pin);
		*PORTController &= ~(1<<pin);
	}
	
	void turnOn()
	{
		// Turn LED off
		*DDRController |= (1<<pin);
		*PORTController |= (1<<pin);
	}
	
	void toggle()
	{
		// Turn LED off
		*DDRController |= (1<<pin);
		*PORTController ^= (1<<pin);
	}	
	void delay(uint16_t milliseconds)
	{
		_delay_ms(milliseconds);		
	}

};
class Button
{
	private:
	unsigned char volatile * DDRController;
	unsigned char volatile * PORTController;
	unsigned char volatile * PINController;
	uint8_t pin;
	
	// Button Down and Up Logic:
	uint8_t pressedConfidenceLevel = 80;
	uint8_t releasedConfidenceLevel = 80;
	
	uint16_t pressedCounter = 0;
	uint16_t releasedCounter = 0;
	
	bool pressedBoolean;
	bool pressed_enter_once_gate;
	bool released_enter_once_gate;
	bool pullDownResistor;
	
	
	public:
	// Assuming the button is connected to ground. The Class sets DDRx to input and Portx to high
	Button(char _PORT, uint8_t _pin, bool _pullDownResister = true)
	{
		if(_PORT == 'A'){DDRController = &DDRA; PORTController = &PORTA; PINController = &PINA;}
		if(_PORT == 'B'){DDRController = &DDRB; PORTController = &PORTB; PINController = &PINB;}
		if(_PORT == 'C'){DDRController = &DDRC; PORTController = &PORTC; PINController = &PINC;}
		if(_PORT == 'D'){DDRController = &DDRD; PORTController = &PORTD; PINController = &PIND;}
			
		// Initialize Button
		pin = _pin;
		pullDownResistor = _pullDownResister;
		if (pullDownResistor)
		{ // Set Button DDR to input and the port low
			*DDRController &= ~(1<<pin);
			*PORTController |= (1<<pin);
		}
		else
		{// Sett Button DDR to input and port high
			*DDRController &= ~(1<<pin);
			*PORTController &= (1<<pin);
		}
		
		pressedBoolean = false;
		pressed_enter_once_gate = true;
		released_enter_once_gate = true;
		pullDownResistor = true;
	}	

	bool isDown(uint8_t ITERATIONS = 100)
	{	
		for(int i = 0; i < ITERATIONS; i++)
		{
				if(!(*PINController & (1<<pin))) // Button Down
				{
					pressedCounter++;
					if (pressedCounter >= pressedConfidenceLevel) // Button has been down for enough cycles its safe to say were good
					{
						if (pressed_enter_once_gate) // One attempt entrance until released pops open. Instructions just need to be sent once
						{
							pressedBoolean = true;
							
							pressed_enter_once_gate = false;
							released_enter_once_gate = true;
							releasedCounter = 0;
						}
					}
					if (pressedCounter > 65000) pressedCounter = 0;
				}
				else
				{
					if(*PINController & (1<<pin)) // Button Up
					{
						releasedCounter++;
						if (releasedCounter >= releasedConfidenceLevel)
						{
							if (released_enter_once_gate)
							{
								pressedBoolean = false; // Return that the button is not pushed
								
								pressed_enter_once_gate = true;
								released_enter_once_gate = false;
								pressedCounter = 0;
							}
						}
						if (releasedCounter > 65000) releasedCounter = 0;
					}
				}
		}
		return pressedBoolean;
	}
	
	bool isUp(uint8_t ITERATIONS = 100)
	{
		for(int i = 0; i < ITERATIONS; i++)
		{
				if(*PINController & (1<<pin)) // Button Down
				{
					pressedCounter++;
					if (pressedCounter >= pressedConfidenceLevel) // Button has been down for enough cycles its safe to say were good
					{
						if (pressed_enter_once_gate) // One attempt entrance until released pops open. Instructions just need to be sent once
						{
							pressedBoolean = true;
							
							pressed_enter_once_gate = false;
							released_enter_once_gate = true;
							releasedCounter = 0;
						}
					}
					if (pressedCounter > 65000) pressedCounter = 0;
				}
				else
				{
					if(!(*PINController & (1<<pin))) // Button Up
					{
						releasedCounter++;
						if (releasedCounter >= releasedConfidenceLevel)
						{
							if (released_enter_once_gate)
							{
								pressedBoolean = false; // Return that the button is not pushed
								
								pressed_enter_once_gate = true;
								released_enter_once_gate = false;
								pressedCounter = 0;
							}
						}
						if (releasedCounter > 65000) releasedCounter = 0;
					}
				}

		}
		
		return pressedBoolean;
	}
	
};
class SparkFunIMU_MPU9250
{
	private:
	DCMotor* leftMotor;
	DCMotor* rightMotor;
	uint8_t deviceID = 0x68;
	
	uint8_t ACCEL_XOUT_H = 0x3B; // Accel Start Read
	uint8_t GYRO_XOUT_H = 0x43; // Gyro Start Read
	
	uint8_t TEMP_OUT_H = 0x41; // Check Temp
	uint8_t TEMP_OUT_L = 0x42; // Check Temp
	
	// Initialization
	uint8_t SIGNAL_PATH_RESET = 0x68;
	uint8_t USER_CTRL = 0x6A;
	uint8_t PWR_MGMT_1 = 0x6B;	
	uint8_t PWR_MGMT_2 = 0x6C;
	uint8_t WHO_AM_I = 0x75;
	
	// Offset Registers
	uint8_t XG_OFFSET_H  = 0x13;
	uint8_t XG_OFFSET_L  = 0x14;
	uint8_t YG_OFFSET_H  = 0x15;
	uint8_t YG_OFFSET_L  = 0x16;
	uint8_t ZG_OFFSET_H  = 0x17;
	uint8_t ZG_OFFSET_L =  0x18;
	uint8_t XA_OFFSET_H  = 0x77;
	
	
	uint8_t TWBR_val = ((1000000/50000) -16) / 2; // ((F_CPU/F_SCL)-16)/2 .. Running SCL at 50KHz
	uint8_t I2C_READ =  0x01;
	uint8_t I2C_WRITE = 0x00;
	uint8_t IMU_WRITE = 0xD0; 
	uint8_t IMU_READ  = 0xD1; 
	
	unsigned char volatile * DDRController = NULL;
	unsigned char volatile * PORTController = NULL;
	uint8_t successPin;
	uint8_t failedPin;
	long watchDog = 0;
	bool exitGate = false;
	uint16_t WATCHDOGITERTIONS = 50000;
	
	// Calibration variables
	int initializationArray[100];
	float varianceX;
	float varianceY;
	
	void initializeTWI_I2C()
	{
		// Initialize I2C / TWI
		PRR0 &= ~(1<<PRTWI); // Turn off power reduction so I2C/TWI is not disabled
		TWCR &= ~(1<<TWIE); // Turn off interrupt enable
		TWBR = 2; // 1Mhz clock and I2c will be running at 50 kHz
		TWSR &= ~(1<<TWPS1) | (1 << TWPS0);
	}
	
	uint8_t i2c_start(uint8_t address)
	{
		// reset TWI control register
		TWCR = 0;
		// transmit START condition
		TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
		// wait for end of transmission
		watchDog = 0;
		while( !(TWCR & (1<<TWINT)) )
		{
			if (exitGate) break;
			watchDog++;
			if (watchDog > WATCHDOGITERTIONS)
			{
				DDRA |= (1 << PINA4);
				PORTA ^= (1<<PINA4);
				_delay_ms(2);
				PORTA ^= (1<<PINA4);
				exitGate = true;
				break;
			}
		}
		
		// check if the start condition was successfully transmitted
		if((TWSR & 0xF8) != TW_START){ return 1; }
		
		
		// load slave address into data register
		TWDR = address;
		// start transmission of address
		TWCR = (1<<TWINT) | (1<<TWEN);
		// wait for end of transmission
		watchDog = 0;
		while( !(TWCR & (1<<TWINT)) )
		{
			if(exitGate) break;
			watchDog++;
			if (watchDog > WATCHDOGITERTIONS)
			{
				DDRA |= (1 << PINA4);
				PORTA ^= (1<<PINA4);
				_delay_ms(2);
				PORTA ^= (1<<PINA4);
				exitGate = true;
				break;
			}
		}
		
		// check if the device has acknowledged the READ / WRITE mode
		uint8_t twst = TW_STATUS & 0xF8;
		if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;
		
		return 0;
	}
	uint8_t i2c_write(uint8_t data)
	{
		// load data into data register
		TWDR = data;
		// start transmission of data
		TWCR = (1<<TWINT) | (1<<TWEN);
		// wait for end of transmission
		watchDog = 0;
		while( !(TWCR & (1<<TWINT)) )
		{
			if (exitGate) break; // If the exit gate was triggered, skip over each while loop inside the readGyro and readAccel
			watchDog++;
			if (watchDog > WATCHDOGITERTIONS)
			{
				DDRA |= (1 << PINA4);
				PORTA ^= (1<<PINA4);
				_delay_ms(2);
				PORTA ^= (1<<PINA4);
				exitGate = true;
				break;
			}
			
		}
		
		if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){ return 1; }
		
		return 0;
	}
	
	uint8_t i2c_read_ack(void)
	{
		// start TWI module and acknowledge data after reception
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
		// wait for end of transmission
		watchDog= 0;
		while( !(TWCR & (1<<TWINT)) )
		{
			if (exitGate) break; // If the exit gate was triggered, skip over each while loop inside the readGyro and readAccel	
			watchDog++;
			if (watchDog > WATCHDOGITERTIONS)
			{
				DDRA |= (1 << PINA4);
				PORTA ^= (1<<PINA4);
				_delay_ms(2);
				PORTA ^= (1<<PINA4);
				exitGate = true;
				break;
			}
		}
		// return received data from TWDR
		return TWDR;
	}
	
	uint8_t i2c_read_nack(void)
	{
		// start receiving without acknowledging reception
		TWCR = (1<<TWINT) | (1<<TWEN);
		// wait for end of transmission
		watchDog = 0;
		while( !(TWCR & (1<<TWINT)) )
		{
			if (exitGate) break; // If the exit gate was triggered, skip over each while loop inside the readGyro and readAccel
			watchDog++;
			if (watchDog > WATCHDOGITERTIONS)
			{
				DDRA |= (1 << PINA4);
				PORTA ^= (1<<PINA4);
				_delay_ms(2);
				PORTA ^= (1<<PINA4);
				exitGate = true;
				break;
			}
		}
		// return received data from TWDR
		return TWDR;
	}
	void i2c_stop(void)
	{
		// transmit STOP condition
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	}
	
	void transmissionCheck(uint8_t returned)
	{
		if(returned == 0) {*PORTController ^= (1<<successPin); _delay_ms(10);*PORTController ^= (1<<successPin); _delay_ms(10);}
		else {*PORTController ^= (1<<failedPin); _delay_ms(10); *PORTController ^= (1<<failedPin); _delay_ms(10);}
	}
	void resetExitGate()
	{
		exitGate = false;
	}
	void initializeGyro()
	{
		resetExitGate();
		int gyroX = 0;
		int gyroY = 0;
		int gyroZ = 0;
		const uint8_t ITERATIONS = 100;
		gyroData[0] = 0;
		gyroData[1] = 0;
		gyroData[2] = 0;
		
		
		// Start at LSB of X Register and go up to the MSB of Z
		for (int i = 0; i < ITERATIONS; i++)
		{
			readGyro();
			gyroX += gyroData[0];
			gyroY += gyroData[1];
			gyroZ += gyroData[2];
		}
		
		gyroOffsets[0] = gyroX / ITERATIONS;
		gyroOffsets[1] = gyroY / ITERATIONS;
		gyroOffsets[2] = gyroZ / ITERATIONS;
		
		this->Va = 1;
		this->Vg = 1;
		/*
		// Turn on Motors Temporarily
		leftMotor->pwmSpeed(50);
		rightMotor->pwmSpeed(50);
		// Data Collection For Gyro in X Direction
		
		int temp1;
		int gyroSum = 0;
		float gyroXMean = 0;
		for (int i = 0; i < 100; i++)
		{
			this->readGyro();
			temp1 = this->gyroData[0]; // QUID
			temp1 = this->mapVal(temp1,-32768,32767,-250,250); // DPS
			initializationArray[i] = temp1; // STored in DPS
			gyroSum += temp1;
		}
		// Find Mean:
		gyroXMean = ((float)gyroSum / 100);
		float temp2;
		float runningSum = 0;
		for (int i = 0; i< 100; i++)
		{
			temp2 = pow((initializationArray[i] - gyroXMean),2);
			runningSum += temp2;
		}
		Vg = runningSum/ (99); // Save Variance of Gyro
		
		// Data Collection For Pitch Angles
		temp1 = 0; // Reset
		temp2 = 0; // Reset
		runningSum = 0; // Reset
		int pitchSum = 0;
		float pitchMean;
		for (int i = 0; i < 100; i++)
		{
			this->calculatePitch();
			initializationArray[i] = this->pitch;
			pitchSum += this->pitch;
		}
		// Find Mean:
		pitchMean = ((float)pitchSum / 100);
		for (int i = 0; i< 100; i++)
		{
			temp2 = pow(initializationArray[i] - pitchMean,2);
			runningSum += temp2;
		}
		Va = runningSum/ 99; // Save variance of Accel
		
		// Turn off Motors Temporarily
		leftMotor->pwmSpeed(0);
		rightMotor->pwmSpeed(0);
		*/
	}
	void initializeAccel()
	{
		resetExitGate();
		// The orientation is as follows: +Z should be down . +X should be straight, +Y should be pointing right when you look at the front of device
		// This function (and class) should be updated to include user input to decided what the G rating is -> [2,4,8,16] Gs
		long long AccelX = 0;
		long long AccelY = 0;
		long long AccelZ = 0;
		const uint8_t ITERATIONS = 100;
		accelOffsets[0] = 0;
		accelOffsets[1] = 0;
		accelOffsets[2] = 0;
		// Start at LSB of X Register and go up to the MSB of Z
		for (int i = 0; i < ITERATIONS; i++)
		{
			readAccel();
			//AccelX += accelData[0];
			//AccelY += accelData[1];  
			AccelZ += accelData[2];
		}

		// X and Y should read zero initially
		//accelOffsets[0] = (int)(AccelX / ITERATIONS);
		//accelOffsets[1] = (int)(AccelY / ITERATIONS);
		// Z should read 16383
		uint16_t  zOffset = (int)(AccelZ / ITERATIONS);
		accelOffsets[2] = zOffset - 16383;
	}
	
	public:
	// Offset Values
	// Tell this class what port your two pins are connected to
	SparkFunIMU_MPU9250(char _PORT, uint8_t pinSuccessOperation, uint8_t pinFailedOperation, DCMotor* _leftMotor, DCMotor* _rightMotor)
	{
		// Initialize Motors
		leftMotor = _leftMotor;
		rightMotor = _rightMotor;
		
		
		// In order to Use PORT C fully, disable the JTD Debug Mode
		MCUCR |= (1<<JTD);
		MCUCR |= (1<<JTD);
		
		if (_PORT== 'A') {PORTController = &PORTA; DDRController = &DDRA; }
		if (_PORT== 'B') {PORTController = &PORTB; DDRController = &DDRB; }
		if (_PORT== 'C') {PORTController = &PORTC; DDRController = &DDRC; }
		if (_PORT== 'D') {PORTController = &PORTD; DDRController = &DDRD; }
		
		*DDRController |= (1 << pinSuccessOperation) | (1 << pinFailedOperation);
		successPin = pinSuccessOperation;
		failedPin = pinFailedOperation;
		initializeTWI_I2C();
	}
	
	int accelData[3] = {0};    // [x, y, z] Accelerometer
	int gyroData[3] = {0};     // [x, y, z] Gyro
	int magnetometer[3] = {0}; // [x, y, z] Magnetometer 
	int gyroOffsets[3] = {0};  // [x, y, z] For Gyro
	int accelOffsets[3] = {0};
	int pitch;
	int roll;
	float Vg;
	float Va;
	float d2r = 3.14/180;
	float r2d = 180/3.14;
	
	void initializeIMU()
	{
		// Initialization: Configure the device for 2g mode
		initializeTWI_I2C();
		resetExitGate();
		transmissionCheck(i2c_start(IMU_WRITE));
		transmissionCheck(i2c_write(0x77));
		transmissionCheck(i2c_write(0x00));
		
		// Reset
		transmissionCheck(i2c_start(IMU_WRITE));
		transmissionCheck(i2c_write(PWR_MGMT_1));
		transmissionCheck(i2c_write(0x80));
		
		// Reset Data Registers
		transmissionCheck(i2c_start(IMU_WRITE));
		transmissionCheck(i2c_write(SIGNAL_PATH_RESET));
		transmissionCheck(i2c_write(0x07));
		
		// Wake
		transmissionCheck(i2c_start(IMU_WRITE));
		transmissionCheck(i2c_write(PWR_MGMT_1));
		transmissionCheck(i2c_write(0x00));
		
		// All Sensors are on
		transmissionCheck(i2c_start(IMU_WRITE));
		transmissionCheck(i2c_write(PWR_MGMT_2)); 
		transmissionCheck(i2c_write(0x00));
		
		// initializeIMU
		initializeGyro();
		initializeAccel();
	}
	
	void readAccel()
	{
		resetExitGate();
		// Start at LSB of X Register and go up to the MSB of Z
		i2c_start(IMU_WRITE);
		i2c_write(ACCEL_XOUT_H);
		i2c_stop();
		
		transmissionCheck(i2c_start(IMU_READ));
		
		int raw_x_high = ((uint8_t)i2c_read_ack());
		int raw_x = ((raw_x_high << 8)| i2c_read_ack());
		
		int raw_y_high = ((uint8_t)i2c_read_ack());
		int raw_y = ((raw_y_high << 8) | i2c_read_ack());
		
		int raw_z_high = ((uint8_t)i2c_read_ack());
		int raw_z = ((raw_z_high << 8) | i2c_read_nack());
		
		i2c_stop();
		
		// Reset Data Registers
		i2c_start(IMU_WRITE);
		i2c_write(USER_CTRL);
		i2c_write(0x01);
		i2c_stop();
		
		if (!exitGate)
		{
			accelData[0] = raw_x; //+ accelOffsets[0];
			accelData[1] = raw_y; //- accelOffsets[1];
			accelData[2] = raw_z; //- accelOffsets[2];
		}
	}
	
	void readGyro()
	{
		resetExitGate();
		// Start at LSB of X Register and go up to the MSB of Z
		i2c_start(IMU_WRITE);
		i2c_write(GYRO_XOUT_H);
		i2c_stop();
		
		transmissionCheck(i2c_start(IMU_READ));
		
		int raw_x_high = ((uint8_t)i2c_read_ack());
		int raw_x = ((raw_x_high << 8)| i2c_read_ack());
		
		int raw_y_high = ((uint8_t)i2c_read_ack());
		int raw_y = ((raw_y_high << 8) | i2c_read_ack());
		
		int raw_z_high = ((uint8_t)i2c_read_ack());
		int raw_z = ((raw_z_high << 8) | i2c_read_nack());
		
		i2c_stop();
		
		// Reset Data Registers
		i2c_start(IMU_WRITE);
		i2c_write(USER_CTRL);
		i2c_write(0x01);
		i2c_stop();
		
		if (!exitGate) // If the exitGate condition is not true...
		{
			gyroData[0] = raw_x - gyroOffsets[0];
			gyroData[1] = raw_y - gyroOffsets[1];
			gyroData[2] = raw_z - gyroOffsets[2];
		}
		
	}
	
	uint16_t whoAMI()
	{
		transmissionCheck(i2c_start(IMU_WRITE));
		transmissionCheck(i2c_write(WHO_AM_I));
		i2c_stop();
		
		transmissionCheck(i2c_start(IMU_READ));
		uint8_t iAm = (uint8_t)i2c_read_nack();
		i2c_stop();
		return iAm;
	}

	float mapVal(float valueToMap, float ilow, float ihigh, float olow, float ohigh)
	{
		float i_range = ihigh - ilow;
		float o_range = ohigh - olow;
		valueToMap -= ilow;
		float mappedValue = ((float) valueToMap / i_range) * o_range;
		// Add offset and return to user
		mappedValue += olow;
		return mappedValue;
	}

	void calculatePitch()
	{
		// Read Values from MPU9250
		this->readAccel();
		
		float XAccel = accelData[0]; // [-32768, 32767]
		float YAccel = accelData[1]; // [-32768, 32767]
		float ZAccel = accelData[2]; // [-32768, 32767]
		
		// Convert from ADC bits to G's -- UPDATE 19.6
		XAccel = this->mapVal(XAccel, -32768, 32767, -19.6, 19.6); // [m/s^2]
		YAccel = this->mapVal(YAccel, -32768, 32767, -19.6, 19.6); // [m/s^2]
		ZAccel = this->mapVal(ZAccel, -32768, 32767, -19.6, 19.6); // [m/s^2]
		
		// Find Pitch
		double pitchDenom  = sqrt(pow(YAccel,2) + pow(ZAccel,2));
		float pitchRadians = atan2(YAccel,pitchDenom);
		float pitchDegrees = pitchRadians * this->r2d;	// Radian to degree
		pitchDegrees *= 2; // Since Accelerometer is out of 2g, we need to adjust by 2
		
		// Update pitch inside class as public member variable
		this->pitch = pitchDegrees;
	}
	
};


// AVR Microcontroller Internal Units
class Atmega324pTimer0
{
	private:
	uint16_t timer0Prescale = 0;
	
	public:
	Atmega324pTimer0()
	{
		
	}
	
	// Stores elapsed time between functions tic and toc
	long _tictoc = 0;
	
	// Timer Counting and Compare Registers
	unsigned char volatile * timer0Count  = & TCNT0;
	unsigned char volatile * timer0_OCR0A = & OCR0A;
	unsigned char volatile * timer0_OCR0B = & OCR0B;
	
	
	// Timer Control Registers
	unsigned char volatile * TimerControl0A  = & TCCR0A;
	unsigned char volatile * TimerControl0B  = & TCCR0B;
	
	// MUST: Set Pre-scale Value
	void setTimer0Prescale(uint16_t prescale)
	{
		if(prescale == 0) TCCR0B &= ~(7<<0);
		if(prescale == 1) TCCR0B |= (1<<CS00);
		if(prescale == 8) TCCR0B |= (1<<CS01);
		if(prescale == 64) TCCR0B |= (1<<CS00) | (1<< CS01);
		if(prescale == 256) TCCR0B |= (1<<CS02);
		if(prescale == 1024) TCCR0B |= (1<<CS00) | (1<<CS02);
		
		timer0Prescale = prescale;
	}
	

	// Fast PWM
	// 0-3: 0 = No Compare, 1 = Toggle on compare, 2 = clear on compare
	void initializeOutputCompareA_PWM(uint8_t config)
	{
		*TimerControl0A |= (config <<6);
	}
	void initializeOutputCompareB_PWM(uint8_t config)
	{
		*TimerControl0A |= (config <<4);
	}
	
	
	// Set Value for Output Compare
	void setOCR0A(uint8_t value)
	{
		*timer0_OCR0A = value;
	}
	void setOCR0B(uint8_t value)
	{
		*timer0_OCR0B = value;
	}
	
	///Enable Interrupts
	
	void enableTimerOverflowInterrupt()
	{
		TIMSK0 |= (1<<TOIE0); // Enable timer interrupt 0
		sei(); // Enable Global Interrupts
	}
	
	void tic()
	{
		cli(); // Clear Interrupts
		// .2550 ms time step with a prescale of 0
		TCNT0 = 0; // Reset the TCNT
		_tictoc = 0; // Reset tictoc
		setTimer0Prescale(8); // Set prescale to 1 for most accurate timing
		TIMSK0 |= (1<<TOIE0); // Enable timer interrupt 0
		
		sei(); // Enable Global Interrupts
		
	}
	float toc()
	{
		// returns the number of cycles tic/toc has completed
		//cli();
		float _time = ((float)_tictoc)*.255; // convert to miliseconds
		uint8_t leftovers = TCNT0;
		float addTime = ((float)leftovers / 255) * .255;
		_time += addTime;
		
		TIMSK0 &= ~(1<<TOIE0); // Disable timer interrupt 0 (until we get back to tic)
		return _time;
	}
	void updateTicTocInsideInterupt()
	{
		_tictoc += 1; // Add ms due to overflow
		TCNT0 = 0;
	}

};
class Atmega324pTimer1
{
	private:
	uint16_t timer1Prescale = 0;
	
	public:
	Atmega324pTimer1()
	{
		
	}
	long _tictoc = 0;
	// Timer Counting and Compare Registers
	unsigned int volatile * timer1Count  = & TCNT1;
	unsigned int volatile * timer1_OCR1A = & OCR1A;
	unsigned int volatile * timer1_OCR1B = & OCR1B;
	
	
	// Timer Control Registers
	unsigned char volatile * TimerControl1A  = & TCCR1A;
	unsigned char volatile * TimerControl1B  = & TCCR1B;
	
	// Set Pre-scale
	void setTimer0Prescale(uint16_t prescale)
	{
		if(prescale == 0) TCCR1B &= ~(7<<0);
		if(prescale == 1) TCCR1B |= (1<<CS10);
		if(prescale == 8) TCCR1B |= (1<<CS11);
		if(prescale == 64) TCCR1B |= (1<<CS10) | (1<< CS11);
		if(prescale == 256) TCCR1B |= (1<<CS12);
		if(prescale == 1024) TCCR1B |= (1<<CS10) | (1<<CS12);
	}
	
	// nonPwm -- Fill in
	// 0-3: 0 = No Compare, 1 = Toggle on compare, 2 = clear on compare, 3 = set on compare match
	void initializeOutputCompareA_Timer1(uint8_t config)
	{
		*TimerControl1A |= (config <<6);
	}
	void initializeOutputCompareB_Timer1(uint8_t config)
	{
		*TimerControl1A |= (config <<4);
	}

	// Fast PWM
	// 0-3: 0 = No Compare, 1 = Toggle on compare, 2 = clear on compare
	void initializeOutputCompareA_PWM_Timer0(uint8_t config)
	{
		*TimerControl1A |= (config <<6);
	}
	void initializeOutputCompareB_PWM_Timer0(uint8_t config)
	{
		*TimerControl1A |= (config <<4);
	}
	
	
	// Set Value for Output Compare
	void setOCR0A(uint8_t value)
	{
		*timer1_OCR1A = value;
	}
	void setOCR0B(uint8_t value)
	{
		*timer1_OCR1B = value;
	}
	
	void tic()
	{
		cli(); // Clear Interrupts
		TCNT1 = 0; // Reset the TCNT
		_tictoc = 0; // Reset tictoc
		setTimer0Prescale(1); // Set prescale to 1 for most accurate timing
		TIMSK1 |= (1<<TOIE0); // Enable timer interrupt 0
		
		sei(); // Enable Global Interrupts
		
	}
	float toc()
	{
		// returns the number of cycles tic/toc has completed
		cli();
		float _time = _tictoc * 65.53; // convert to miliseconds
		float leftovers = TCNT1;
		float addTime = (leftovers / 65535) * 65.535;
		_time += addTime;
		TCNT1 = 0;
		return _time;
	}
	void updateTicTocInsideInterupt()
	{
		_tictoc += 1; // Add ms due to overflow
	}
	
	
};
class Atmega324pTimer2
{
	private:
	uint16_t timer2Prescale = 0;
	
	public:
	Atmega324pTimer2();
	
	// Timer Counting and Compare Registers
	unsigned char volatile * timer2Count  = & TCNT2;
	unsigned char volatile * timer2_OCR2A = & OCR2A;
	unsigned char volatile * timer2_OCR2B = & OCR2B;
	
	
	// Timer Control Registers
	unsigned char volatile * TimerControl2A  = & TCCR2A;
	unsigned char volatile * TimerControl2B  = & TCCR2B;
	
	// Set Pre-scale
	void setTimer0Prescale(uint16_t prescale)
	{
		if(prescale == 0) TCCR2B &= ~(7<<0);
		if(prescale == 1) TCCR2B |= (1<<CS20);
		if(prescale == 8) TCCR2B |= (1<<CS21);
		if(prescale == 64) TCCR2B |= (1<<CS20) | (1<< CS21);
		if(prescale == 256) TCCR2B |= (1<<CS22);
		if(prescale == 1024) TCCR2B |= (1<<CS20) | (1<<CS22);
	}
	
	// nonPwm -- Fill in
	// 0-3: 0 = No Compare, 1 = Toggle on compare, 2 = clear on compare, 3 = set on compare match
	void initializeOutputCompareA_Timer1(uint8_t config)
	{
		*TimerControl2A |= (config <<6);
	}
	void initializeOutputCompareB_Timer1(uint8_t config)
	{
		*TimerControl2A |= (config <<4);
	}

	// Fast PWM
	// 0-3: 0 = No Compare, 1 = Toggle on compare, 2 = clear on compare
	void initializeOutputCompareA_PWM_Timer0(uint8_t config)
	{
		*TimerControl2A |= (config <<6);
	}
	void initializeOutputCompareB_PWM_Timer0(uint8_t config)
	{
		*TimerControl2A |= (config <<4);
	}
	
	
	// Set Value for Output Compare
	void setOCR0A(uint8_t value)
	{
		*timer2_OCR2A = value;
	}
	void setOCR0B(uint8_t value)
	{
		*timer2_OCR2B = value;
	}
};
class Atmega324pExternalInterrupt0
{
	private:
	unsigned char volatile * ExternalInterruptController = &EICRA;
	unsigned char volatile * ExternalInterruptMask = &EIMSK;
	public:
	// Ext Interrupts: [0,1,2] interruptSenseControl = [0,1,2,3]
	// interruptSenseControl: 0 = low level, 1 = Any change, 2 = falling edge, 3 = rising edge
	Atmega324pExternalInterrupt0();

	void enableInterupt()
	{
		sei(); // Turn Global Interrupts on
	}
	// interruptSenseControl: 0 = low level, 1 = Any change, 2 = falling edge, 3 = rising edge
	void disableInterrupts()
	{
		cli(); // Turn Global Interrupts off
	}
	void initializeInterruptEvent(uint8_t config) // Select Which type of logic will execute your interrupt
	{
		*ExternalInterruptMask |= (1<<0); // Set EIMSK Control Mask to INT0
		*ExternalInterruptController |= (config << 0);
	}
	
};
class Atmega324pExternalInterrupt1
{
	private:
	unsigned char volatile * ExternalInterruptController = &EICRA;
	unsigned char volatile * ExternalInterruptMask = &EIMSK;
	public:
	// Ext Interrupts: [0,1,2] interruptSenseControl = [0,1,2,3]
	// interruptSenseControl: 0 = low level, 1 = Any change, 2 = falling edge, 3 = rising edge
	Atmega324pExternalInterrupt1();

	void enableInterupt()
	{
		sei(); // Turn Global Interrupts on
	}
	// interruptSenseControl: 0 = low level, 1 = Any change, 2 = falling edge, 3 = rising edge
	void disableInterrupts()
	{
		cli(); // Turn Global Interrupts off
	}
	void initializeInterruptEvent(uint8_t config) // Select Which type of logic will execute your interrupt
	{
		*ExternalInterruptMask |= (1<<1); // Set EIMSK Control Mask to INT0
		*ExternalInterruptController |= (config << 2);
	}
	
};
class Atmega324pExternalInterrupt2
{
	private:
	unsigned char volatile * ExternalInterruptController = &EICRA;
	unsigned char volatile * ExternalInterruptMask = &EIMSK;
	public:
	// Ext Interrupts: [0,1,2] interruptSenseControl = [0,1,2,3]
	// interruptSenseControl: 0 = low level, 1 = Any change, 2 = falling edge, 3 = rising edge
	Atmega324pExternalInterrupt2();

	void enableInterupt()
	{
		sei(); // Turn Global Interrupts on
	}
	// interruptSenseControl: 0 = low level, 1 = Any change, 2 = falling edge, 3 = rising edge
	void disableInterrupts()
	{
		cli(); // Turn Global Interrupts off
	}
	void initializeInterruptEvent(uint8_t config) // Select Which type of logic will execute your interrupt
	{
		*ExternalInterruptMask |= (1<<2); // Set EIMSK Control Mask to INT0
		*ExternalInterruptController |= (config << 4);
	}
	
};
class Atmega324pUSART
{
	private:
		uint8_t latestDataReceived;
		uint8_t latestDataSent;
		void waitForMasterUSART0_ToBeReady()
		{
			// Wait until the transmitter is ready
			while(!(UCSR0A & (1<<UDRE0))); // Cant Pass until its a one
		}
		void waitForSlaveUSART0_ToBeReady()
		{
			while(!(UCSR0A & (1 << RXC0)));
		}
		void waitForMasterUSART1_ToBeReady()
		{
			// While the UDRE is still processing our request
			while(!(UCSR1A & (1<<UDRE1))); // Cant Pass until its a one
		}
		void waitForSlaveUSART1_ToBeReady()
		{
			while(!(UCSR1A & (1 << RXC1)));
		}
	public:
	// Constructor
	Atmega324pUSART()
	{
				
	}
	
	// Initialize The USARTS with these functions
	void enableUSART0AsMaster(uint16_t baud = 2400, bool AsyncDoubleSpeed = false, bool setParity = true)
	{
		uint16_t UBBRValue = lrint ( (F_CPU / (16L * baud)) - 1 ); // Rounds to nearest integer
		
		// Enable Double Speed
		if(AsyncDoubleSpeed) UCSR0A |= (1<<U2X0); // Sets Double Speed Mode
		
		// Write UBBRValue to the baud rate registers
		UBRR0H = (uint8_t)(UBBRValue >> 8);
		UBRR0L = (uint8_t)(UBBRValue);
		
		// Set bits high for receiving enable and transmit enable
		UCSR0B  = (1<<RXEN0) | (1<<TXEN0);
		
		
		// USBS = Stop Bit
		// Setup 8 bit transmission mode with 2 stop bits - User has no choice here
		UCSR0C |= (1<<USBS0) | (1 << UCSZ00) | (1 << UCSZ01);
		
		// Enable Parity Even or Odd
		if (setParity) UCSR0C |= (1<< UPM01); // EVEN
		else UCSR0C |= (3<<UPM00);  // ODD
	}
	
	void enableUSART1AsMaster(uint16_t baud = 2400, bool AsyncDoubleSpeed = false, bool setParity = true)
	{
		uint16_t UBBRValue = lrint ( (F_CPU / (16L * baud)) - 1 ); // Rounds to nearest integer
		
		// Enable Double Speed
		if(AsyncDoubleSpeed) UCSR1A |= (1<<U2X1); // Sets Double Speed Mode
		
		// Write UBBRValue to the baud rate registers
		UBRR1H = (UBBRValue >> 8);
		UBRR1L = (UBBRValue);
		
		// Set bits high for receiving enable and transmit enable
		UCSR1B  = (1<<RXEN1) | (1<<TXEN1);
		
		
		// USBS = Stop Bit
		// Setup 8 bit transmission mode with 2 stop bits - User has no choice here
		UCSR1C |= (1<<USBS1) | (1 << UCSZ10) | (1 << UCSZ11);
		
		// Enable Parity Even or Odd
		if (setParity) UCSR1C |= (1<< UPM11); // EVEN
		else UCSR1C |= (3<<UPM10);  // ODD
	}
	
	void enableUSART0AsSlave(uint16_t baud = 2400, bool AsyncDoubleSpeed = false, bool setParity = true)
	{
		uint16_t UBBRValue = lrint ( (F_CPU / (16L * baud)) - 1 ); // Rounds to nearest integer
		
		// Enable Double Speed
		if(AsyncDoubleSpeed) UCSR0A |= (1<<U2X0); // Sets Double Speed Mode
		
		// Write UBBRValue to the baud rate registers
		UBRR0H = (UBBRValue >> 8);
		UBRR0L = (UBBRValue);
		
		// Set bits high for receiving enable and transmit enable
		UCSR0B  = (1<<RXEN0) | (1<<TXEN0);
		
		
		// USBS = Stop Bit
		// Setup 8 bit transmission mode with 2 stop bits - User has no choice here
		UCSR0C |= (1<<USBS0) | (1 << UCSZ00) | (1 << UCSZ01);
		
		// Enable Parity Even or Odd
		if (setParity) UCSR0C |= (1<< UPM01); // EVEN
		else UCSR0C |= (3<<UPM00);  // ODD
	}
	
	void enableUSART1AsSlave(uint16_t baud = 2400, bool AsyncDoubleSpeed = false, bool setParity = true)
	{
		uint16_t UBBRValue = lrint ( (F_CPU / (16L * baud)) - 1 ); // Rounds to nearest integer
		
		// Enable Double Speed
		if(AsyncDoubleSpeed) UCSR1A |= (1<<U2X1); // Sets Double Speed Mode
		
		// Write UBBRValue to the baud rate registers
		UBRR1H = (UBBRValue >> 8);
		UBRR1L = (UBBRValue);
		
		// Set bits high for receiving enable and transmit enable
		UCSR1B  = (1<<RXEN1) | (1<<TXEN1);
		
		// USBS = Stop Bit
		// Setup 8 bit transmission mode with 2 stop bits - User has no choice here
		UCSR1C |= (1<<USBS1) | (1 << UCSZ10) | (1 << UCSZ11);
		
		// Enable Parity Even or Odd
		if (setParity) UCSR1C |= (1<< UPM11); // EVEN
		else UCSR1C |= (3<<UPM10);  // ODD
	}
	
	// Send Data to USART 0-1, Master or Slave with these functions
	void sendDataToSlaveUSART0(uint8_t data)
	{
		waitForMasterUSART0_ToBeReady();
		UDR0 = data;
		latestDataSent = data;
	}
	
	void sendDataToSlaveUSART1(uint8_t data)
	{
		waitForMasterUSART1_ToBeReady();
		UDR1 = data;
		latestDataSent = data;
	}
	
	void sendDataToMasterUSART0(uint8_t data)
	{
		waitForSlaveUSART0_ToBeReady();
		UDR0 = data; // Put Data on the line
		latestDataSent = data;
	}
	
	void sendDataToMasterUSART1(uint8_t data)
	{
		waitForSlaveUSART1_ToBeReady();
		UDR1 = data;
		latestDataSent = data;
	}
	
	uint8_t receiveDataFromMasterUSART0()
	{
		waitForSlaveUSART0_ToBeReady();
		uint8_t data = UDR0; 
		latestDataReceived = data; // Sets internally in the class as well
		UDR0 = 0; // Clear Data Line
		return data; // return value back to program
	}
	
	uint8_t receiveDataFromMasterUSART1()
	{
		waitForSlaveUSART1_ToBeReady();
		uint8_t data = UDR1;
		UDR1 = 0; // Clear Data Line
		latestDataReceived = data; // Sets internally in the class as well
		return data; // return value back to program
	}
	
	uint8_t receiveDataFromSlaveUSART0()
	{
		waitForMasterUSART0_ToBeReady();
		uint8_t data = UDR0;
		UDR0 = 0; // CLear Data Line
		latestDataReceived = data; // Sets internally in the class as well
		return data; // return value back to program
	}
	
	uint8_t receiveDataFromSlaveUSART1()
	{
		waitForMasterUSART1_ToBeReady();
		uint8_t data = UDR1;
		UDR1 = 0; // Clear data line
		latestDataReceived = data; // Sets internally in the class as well
		return data; // return value back to program
	}
	
	// Data Confirmation Algorithms
	void waitUntilMasterUSART0ReceivedData()
	{
		while(receiveDataFromMasterUSART0() != DATA_SENT_TO_MASTER_AND_RECEIVED_SUCCESSFULLY);
	}
	
	void waitUntilMasterUSART1ReceivedData()
	{
		while(receiveDataFromMasterUSART1() != DATA_SENT_TO_MASTER_AND_RECEIVED_SUCCESSFULLY);
	}
	
	void waitUntilSlaveUSART0ReceivedData()
	{
		while(receiveDataFromSlaveUSART0() != DATA_SENT_TO_SLAVE_AND_RECEIVED_SUCCESSFULLY);
	}
	
	void waitUntilSlaveUSART1ReceivedData()
	{
		while(receiveDataFromSlaveUSART1() != DATA_SENT_TO_SLAVE_AND_RECEIVED_SUCCESSFULLY);
	}
	
	// Slave Confirmation
	void SlaveUSART0_ConfirmReceivalOfDataFromUSART0Master()
	{
		sendDataToMasterUSART0(DATA_SENT_TO_SLAVE_AND_RECEIVED_SUCCESSFULLY);
	}
	void SlaveUSART0_ConfirmReceivalOfDataFromUSART1Master()
	{
		sendDataToMasterUSART1(DATA_SENT_TO_SLAVE_AND_RECEIVED_SUCCESSFULLY);
	}
	void SlaveUSART1_ConfirmReceivalOfDataFromUSART0Master()
	{
		sendDataToMasterUSART0(DATA_SENT_TO_SLAVE_AND_RECEIVED_SUCCESSFULLY);
	}
	void SlaveUSART1_ConfirmReceivalOfDataFromUSART1Master()
	{
		sendDataToMasterUSART1(DATA_SENT_TO_SLAVE_AND_RECEIVED_SUCCESSFULLY);
	}
	
	// Master Confirmation
	void MasterUSART0_ConfirmReceivalOfDataFromUSART0Slave()
	{
		sendDataToSlaveUSART0(DATA_SENT_TO_MASTER_AND_RECEIVED_SUCCESSFULLY);
	}
	void MasterUSART0_ConfirmReceivalOfDataFromUSART1Slave()
	{
		sendDataToSlaveUSART1(DATA_SENT_TO_MASTER_AND_RECEIVED_SUCCESSFULLY);
	}
	void MasterUSART1_ConfirmReceivalOfDataFromUSART0Slave()
	{
		sendDataToSlaveUSART0(DATA_SENT_TO_MASTER_AND_RECEIVED_SUCCESSFULLY);
	}
	void MasterUSART1_ConfirmReceivalOfDataFromUSART1Slave()
	{
		sendDataToSlaveUSART1(DATA_SENT_TO_MASTER_AND_RECEIVED_SUCCESSFULLY);
	}
	
	// Sending and Receiving Confirmation
	const uint8_t DATA_SENT_TO_SLAVE_AND_RECEIVED_SUCCESSFULLY   = 11;
	const uint8_t DATA_SENT_TO_MASTER_AND_RECEIVED_SUCCESSFULLY  = 12;
	
};

class Atmega324p
{
	private:
	uint8_t ADC_Pointer_Counter = 0;
	
	public:
	
	// Timers
	Atmega324p();
	Atmega324pTimer0* Timer0;
	Atmega324pTimer1* Timer1;
	Atmega324pTimer2* Timer2;
	AnalogToDigitalConverter* ADCController[8];
	
	// Interrupts
	Atmega324pExternalInterrupt0* ExtInt0;
	Atmega324pExternalInterrupt1* ExtInt1;
	Atmega324pExternalInterrupt2* ExtInt2;
	
	void addNewADCPeripheral(uint8_t channel)
	{
		ADCController[ADC_Pointer_Counter] = new AnalogToDigitalConverter(channel);
		ADC_Pointer_Counter++;
		
	}
	void deleteADCPeripheral(uint8_t channel)
	{
		delete ADCController[channel];
	}
	
};



// Prototypes
void TestLCD();
void TestServo();
void controlServoWithPotentiometer();
void TestMotor();
void TestIMU();
void TestAccelerometer();
void TestUSART_Slave();
void TestUSART_Master();
void Quadcopter();
void PIDController();


Atmega324pTimer1 _Timer1;

int main(void)
{	
	PIDController();
}
void TestLCD()
{
	LCD mrLCD('B','D', 0, 1, 2);
	mrLCD.setUpTheLCD();
	int i = 0;
	while(1)
	{
		if(i>255) i = 0;
		//mrLCD.setUpTheLCD();
		mrLCD.Send_A_String_To_Position("ADC Value: ",1,0);
		mrLCD.Send_An_Integer_To_Position_Decimal(i,1,12);
		mrLCD.Send_A_String_To_Position("Binary: ", 2,1);
		mrLCD.Send_An_Integer_To_Position_Binary(i,2,8);
		_delay_ms(1);
		i+=5;
	}
}

void TestServo() 
{
	NonContinousServo servo(1,'A');
	LCD mrLCD('B','D',0,1,2);
	while(1)
	{
		
		for (int i = 0; i <= 180; i++)
		{
			mrLCD.Send_A_String("Angle: ");
			mrLCD.Send_An_Integer_To_Position_Decimal(i,1,8);
			servo.rotateDegrees(i);
			_delay_ms(10);
		}
		for (int i = 180; i >= 0; i--)
		{
			mrLCD.Send_A_String("Angle: ");
			mrLCD.Send_An_Integer_To_Position_Decimal(i,1,8);
			servo.rotateDegrees(i);
			_delay_ms(10);
		}
	}
}

void controlServoWithPotentiometer()
{
	NonContinousServo servo(1,'A');
	LCD mrLCD('B','D',0,1,2); // PortB = Data (), PortD = Command
	AnalogToDigitalConverter potentiometer(0); // ADC0
	potentiometer.switchToMyChannel();
	uint16_t ADCVal;
	float angle;
	while(1)
	{
		ADCVal = potentiometer.runningAverageReadADC(50);
		mrLCD.Send_A_String("ADC0: ");
		mrLCD.Send_A_String_To_Position("Angle: ",2,1);
		mrLCD.Send_An_Integer_To_Position_Decimal(ADCVal,1,8);
		angle = ((float)ADCVal / 1024) * 180;
		mrLCD.Send_An_Integer_To_Position_Decimal(angle,2,9);
		servo.rotateDegrees(angle);
		_delay_ms(10);
	}
	
}

void TestMotor()
{
	DCMotor leftengine('C',2,'A',6,7);
	DCMotor rightEngine('C',2,'B',4,5);
	LCD mrLCD('B','D',0,1,2);
	
	
	while(1)
	{
		for (int i = 0; i <= 100; i++)
		{
			mrLCD.Send_A_String("Speed: ");
			mrLCD.Send_An_Integer_To_Position_Decimal(i,1,8);
			leftengine.pwmSpeed(i);
			rightEngine.pwmSpeed(i);
			_delay_ms(50);
		}
		_delay_ms(1000);
		for (int i = 100; i >= 0; i--)
		{
			mrLCD.Send_A_String("Speed: ");
			mrLCD.Send_An_Integer_To_Position_Decimal(i,1,8);
			leftengine.pwmSpeed(i);
			rightEngine.pwmSpeed(i);
			_delay_ms(50);
		}
		_delay_ms(1000);
		for (int i = 0; i <= 100; i++)
		{
			mrLCD.Send_A_String("Speed: ");
			mrLCD.Send_An_Integer_To_Position_Decimal(i,1,8);
			leftengine.pwmSpeed(i);
			rightEngine.pwmSpeed(i);
			_delay_ms(50);
		}
		
		_delay_ms(1000);
		for (int i = 100; i >= 0; i--)
		{
			mrLCD.Send_A_String("Speed: ");
			mrLCD.Send_An_Integer_To_Position_Decimal(i,1,8);
			leftengine.pwmSpeed(i);
			rightEngine.pwmSpeed(i);
			_delay_ms(50);
		}
		_delay_ms(1000);
	}
}

void TestAccelerometer()
{
	// The Master Test:
	
	// Accel: The Y Position Will Control Motor1
	// Accel: The X Position Will Control Servo1

	// ServoPot: Readout Will Control Servo2
	// MotorPot: ReadOut Will Control Motor2
	
	// Gyro Read-Out Will Control An LED being on or off 
	
	LCD mrLCD('B','D',0,1,2);
	NonContinousServo servo1(1,'A');
	NonContinousServo servo2(1,'B');
	
	DCMotor motor1('C',2,'A',6,7);
	DCMotor motor2('C',2,'B',4,5);
	
	AnalogToDigitalConverter servoPot(0);
	AnalogToDigitalConverter motorPot(1);
	AnalogToDigitalConverter gyro(2);
	
	ADXL345 Accel('C',2,3);
	Accel.initializeADXL();
	
	Button GyroButton('A',5);
	LED GyroLED('A',4);
	
	while(1)
	{
		// Read in Accelerometer and Potentiometers - Must Change channel between reads
		Accel.readData(); // Fill Up public Data Vector with new Data
		servoPot.switchToMyChannel(); 
		servoPot.runningAverageReadADC(25);
		motorPot.switchToMyChannel();
		motorPot.runningAverageReadADC(25);
		gyro.switchToMyChannel();
		gyro.readADC();
		
		// Display Accel Data
		mrLCD.Send_A_String("X: ");
		mrLCD.Send_A_String_To_Position("Y: ",2,1);
		mrLCD.Send_An_Integer_To_Position_Decimal(Accel.data[0],1,3);
		mrLCD.Send_An_Integer_To_Position_Decimal(Accel.data[1],2,4);
		
		// Display Motor1 and Servo1 Headers
		mrLCD.Send_A_String_To_Position("S1: ",1,6);
		mrLCD.Send_A_String_To_Position("M1: ",2,7);
		
		// Calculate Servo and Motor Speeds from Accelerometer Values
		uint8_t servo1Angle = servo1.mapVal(Accel.data[0],-256,256,0,180);
		servo1.rotateDegrees(servo1Angle); // X-Data
		int motor1Speed = motor1.mapVal(Accel.data[1],-256,256,-100,100);
		motor1.pwmSpeed(motor1Speed); // Y-Data
		
		// Play with Non Accelerometer Controlled Devices
		servo2.rotateDegrees(servo2.mapVal(servoPot.currentVal,0,1023,0,180));
		
		// Motor Potentiometer PWM
		int motor2Speed = motor2.mapVal(motorPot.currentVal,0,1023,-100,100); // map val from ADC to speed reading
		motor2.pwmSpeed(motor2Speed);
		
		// Display Motor1 and Servo1 Headers
		mrLCD.Send_An_Integer_To_Position_Decimal(motor1Speed,2,11);
		mrLCD.Send_An_Integer_To_Position_Decimal(servo1Angle,1,10);
		
		// Lets See how many iterations this thing runs at
		GyroLED.toggle();
	}
	
}

void TestIMU()
{
	Button resetButton('D',3); //External Interrupt Button
	//LED interruptLED('C',2); // LED
	DCMotor rightMotor('A',2,'B',0,1); // Timer 2 B: PortB pins 0 and 1
	DCMotor leftMotor('A',2,'A',2,3); // Timer 2 A: PortB pins 0 and 1
	SparkFunIMU_MPU9250 imu('C',2,3, &leftMotor,&rightMotor);
	imu.initializeIMU();
	LCD mrLCD('B','D',0,1,2);
	// Enable External Interrupt 1
	//Atmega324pExternalInterrupt1 ExtInt1;
	//ExtInt1.initializeInterruptEvent(3);
	//ExtInt1.enableInterupt();
		
	while(1)
	{
		imu.readGyro();
		//imu.readAccel();
		mrLCD.Send_A_String("X_Data: ");
		mrLCD.Send_An_Integer_To_Position_Decimal(imu.gyroData[0],1,9);
		mrLCD.Send_A_String_To_Position("Z_Data: ",2,1);
		mrLCD.Send_An_Integer_To_Position_Decimal(imu.gyroData[2],2,10);
		_delay_ms(50);
	}

}

void TestUSART_Master()
{
	// USART Communication Numbers
	
	LED masterLED('D',7);
	LED testLED('D',5);
	Button masterButton('D',6);
	Atmega324pUSART masterUSART0;
	
	masterUSART0.enableUSART0AsMaster(2400,false,true); // 2400 Baud, Double Speed False, Partity is even (true)
	
	while(1)
	{
		if(masterButton.isUp())
		{
			masterLED.toggle();
			masterLED.delay(500);
		}
		
		if(masterButton.isDown())
		{
			testLED.toggle();
			// Send Data and wait for transmission they got it
			masterUSART0.sendDataToSlaveUSART0(69); // he-he
			masterUSART0.waitUntilSlaveUSART0ReceivedData();
			
		}
		if(masterUSART0.receiveDataFromSlaveUSART0() == 69)
		{
			masterUSART0.MasterUSART0_ConfirmReceivalOfDataFromUSART0Slave();
			for (int i = 0; i < 50; i++)
			{
				masterLED.toggle();
				masterLED.delay(50);
			}
		}
	}
}

void TestUSART_Slave()
{
	LED slaveLED('D',7);
	Button slaveButton('D',6);
	Atmega324pUSART slaveUSART0;
		
	slaveUSART0.enableUSART0AsSlave(2400,false,true); // 2400 Baud, Double Speed true, Partity is even (true)
		
	while(1)
	{
		if(slaveButton.isUp())
		{
			slaveLED.toggle();
			slaveLED.delay(500);
		}	
		if(slaveButton.isDown())
		{
			slaveUSART0.sendDataToMasterUSART0(69); // Send Integer Over
			slaveUSART0.waitUntilMasterUSART0ReceivedData();
		}
		if(slaveUSART0.receiveDataFromMasterUSART0() == 69)
		{
			slaveUSART0.SlaveUSART0_ConfirmReceivalOfDataFromUSART0Master();
			for (int i = 0; i < 50; i++)
			{
				slaveLED.toggle();
				slaveLED.delay(50);
			}
		}
	}
}

void Quadcopter()
{
	DCMotor rightMotor('A',1,'B',0,1); // Timer 1 B: PortB pins 0 and 1
	DCMotor leftMotor('A',1,'A',2,3); // Timer 1 A: PortB pins 0 and 1
	SparkFunIMU_MPU9250 imu('C',2,3, &leftMotor,&rightMotor);
	imu.initializeIMU();
	LCD mrLCD('B','D',0,1,2);
	uint16_t elaspedTime;
	while(1)
	{
		// Read Values from MPU9250
		_Timer1.tic();
		
		imu.readAccel();
		_delay_ms(50);
		imu.readGyro();
		
		// LCD Display Values
		//mrLCD.Send_A_String("GX: ");
		//mrLCD.Send_An_Integer_To_Position_Decimal(imu.gyroData[0],1,4);
		//mrLCD.Send_A_String_To_Position("GZ: ",2,1);
		//mrLCD.Send_An_Integer_To_Position_Decimal(imu.gyroData[2],2,5);
		
		float XAccel = imu.accelData[0]; // [-32768, 32767]
		float YAccel = imu.accelData[1]; // [-32768, 32767]
		float ZAccel = imu.accelData[2]; // [-32768, 32767]
		
		// Convert from ADC bits to G's 
		XAccel = imu.mapVal(XAccel, -32768, 32767, -19.6, 19.6); // [m/s^2]
		YAccel = imu.mapVal(YAccel, -32768, 32767, -19.6, 19.6); // [m/s^2]
		ZAccel = imu.mapVal(ZAccel, -32768, 32767, -19.6, 19.6); // [m/s^2]
		
		// Find Pitch
		double pitchDenom  = sqrt(pow(XAccel,2) + pow(YAccel,2));
		float pitchRadians = atan2(YAccel,pitchDenom);
		float pitchDegrees = pitchRadians * imu.r2d;	// Radian to degree
		pitchDegrees *= 2; // Since Accelerometer is out of 2g, we need to adjust by 2
		
		// Find Roll -- ONLY if the pitch angle is not 90 degrees
		float rollRadians;
		float rollDegrees;
		if (pitchDegrees != 90) 
		{
			rollRadians = atan2(-XAccel,ZAccel);
			rollDegrees = rollRadians * imu.r2d;	
		}
		else
		{
			// Something
		}
		
		// Output To LCD
		//mrLCD.Send_A_String_To_Position("P:",1,10);
		elaspedTime = _Timer1.toc();
		mrLCD.Send_A_String("T:");
		//mrLCD.Send_An_Integer_To_Position_Decimal(pitchDegrees,1,13);
		mrLCD.Send_An_Integer_To_Position_Decimal(elaspedTime,1,4);
		
	}
	
}

void PIDController()
{
	DCMotor rightMotor('A',2,'B',0,1); // Timer 2 B: PortB pins 0 and 1
	DCMotor leftMotor('A',2,'A',2,3); // Timer 2 A: PortB pins 0 and 1
	LCD mrLCD('B','D',0,1,2);
	mrLCD.setUpTheLCD();
	
	SparkFunIMU_MPU9250 imu('C',2,3, &leftMotor,&rightMotor);
	imu.initializeIMU();
	
	int pitchReference = 0;
	float Kp_LeftMotor =  .8;
	float Kp_RightMotor = -.8;
	float Kd_LeftMotor =  2;
	float Kd_RightMotor = -2;
	float Ki_LeftMotor =  .00001;
	float Ki_RightMotor = -.00001;
	
	
	//float p00=imu.Va;//error corrections
	//float p01=1;
	//float p10=1;
	//float p11=imu.Vg;
	//double p00n=1;//intermediate p's
	//double p01n=1;
	//double p10n=1;
	//double p11n=1;
	float pitchDot = 1;
	//float pitchHat=1; //current estimate of pitch
	//float pitchDotHat=1; //current estimate of pitchDot
	//double pitchn=0; //new estimate of pitch
	//double pitchDotn=0; //new estimate of pitchDot

	float integral = 0;
	
	float h;
		
	mrLCD.Send_A_String_To_Position("Vg: ",1,0);
	mrLCD.Send_An_Integer_To_Position_Decimal(imu.Vg,1,4);
	mrLCD.Send_A_String_To_Position("Va:",2,1);
	mrLCD.Send_An_Integer_To_Position_Decimal(imu.Va,2,5);
	_delay_ms(1000);
	
	_Timer1.tic(); //initialize timer
	
	while(1)
	{	
		//Read in new pitch
		h = _Timer1.toc(); // Find elapsed time - Dummy Time
		_Timer1.tic(); //reset timer
		imu.calculatePitch(); //find pitch
		_delay_ms(10);
		imu.readGyro(); //update Gyro
		pitchDot=imu.mapVal(imu.gyroData[0],-32768,32767,-250,250);
		
		
		//pitchn = (h*p11*pitchDot*imu.Va + p11*pitchHat*imu.Va + h*p10*imu.pitch*imu.Vg + pow(h,2)*p11*imu.pitch*imu.Vg + h*pitchDotHat*imu.Va*imu.Vg + pitchHat*imu.Va*imu.Vg + p00*imu.pitch*(p11 + imu.Vg) + p01*(-1*p10*imu.pitch + pitchDot*imu.Va - pitchDotHat*imu.Va + h*imu.pitch*imu.Vg)); //*p11*pitchDot*imu.Va;
		//pitchn = (double)(h*p11*pitchDot*imu.Va + p11*pitchHat*imu.Va + h*p10*imu.pitch*imu.Vg + pow(h,2)*p11*imu.pitch*imu.Vg + h*pitchDotHat*imu.Va*imu.Vg + pitchHat*imu.Va*imu.Vg + p00*imu.pitch*(p11 + imu.Vg) + p01*(-1*p10*imu.pitch + pitchDot*imu.Va - pitchDotHat*imu.Va + h*imu.pitch*imu.Vg)) / (double)(p11*imu.Va + h*p10*imu.Vg + pow(h,2)*p11*imu.Vg + imu.Va*imu.Vg + p00*(p11+imu.Vg) + p01*(-p10 + h*imu.Vg));
		//pitchDotn = (double)(-1*(p01*p10*pitchDot) + p00*p11*pitchDot + p11*pitchDot*imu.Va + p10*imu.pitch*imu.Vg + h*p11*imu.pitch*imu.Vg + p00*pitchDotHat*imu.Vg + h*p01*pitchDotHat*imu.Vg - p10*pitchHat*imu.Vg - h*p11*pitchHat*imu.Vg + pitchDotHat*imu.Va*imu.Vg) / (double)(p11*imu.Va + h*p10*imu.Vg + pow(h,2)*p11*imu.Vg + imu.Va*imu.Vg + p00*(p11 + imu.Vg) + p01*(-p10 + h*imu.Vg));
		
		//p00n= (double)(imu.Va*(h*(p10 + h*p11)*imu.Vg + p00*(p11 + imu.Vg) + p01*(-p10 + h*imu.Vg))) /(double)(p11*imu.Va + h*p10*imu.Vg + pow(h,2)*p11*imu.Vg + imu.Va*imu.Vg + p00*(p11 + imu.Vg) + p01*(-p10 + h*imu.Vg));
		//p01n= (double)((p01 + h*p11)*imu.Va*imu.Vg)/(double)(p11*imu.Va + h*p10*imu.Vg + pow(h,2)*p11*imu.Vg + imu.Va*imu.Vg + p00*(p11 + imu.Vg) + p01*(-p10 + h*imu.Vg));
		//p10n= (double)((p10 + h*p11)*imu.Va*imu.Vg)/(double)(p11*imu.Va + h*p10*imu.Vg + pow(h,2)*p11*imu.Vg + imu.Va*imu.Vg + p00*(p11 + imu.Vg) + p01*(-p10 + h*imu.Vg));
		//p11n= (double)((-1*(p01*p10) + p11*(p00 + imu.Va))*imu.Vg)/(double)(p11*imu.Va + h*p10*imu.Vg + pow(h,2)*p11*imu.Vg + imu.Va*imu.Vg + p00*(p11 + imu.Vg) + p01*(-p10 + h*imu.Vg));
		
		//pitchHat=pitchn; // Update Guess
		//pitchDotHat=pitchDotn; // Update Guess
		
		//p00=p00n;
		//p01=p01n;
		//p10=p10n;
		//p11=p11n;
		
		
		
		
		// Find Values
		integral += h*(imu.pitch-pitchReference);
		
		//int leftMotorSpeed  = Kp_LeftMotor*(pitchHat - pitchReference) + Kd_LeftMotor*(pitchDotHat-0) + Ki_LeftMotor*integral;
		//int rightMotorSpeed = Kp_RightMotor*(pitchHat - pitchReference) + Kd_RightMotor*(pitchDotHat-0) + Ki_RightMotor*integral;
		int leftMotorSpeed  = Kp_LeftMotor*(imu.pitch - pitchReference)+Kd_LeftMotor*(pitchDot)+Ki_LeftMotor*integral;
		int rightMotorSpeed = Kp_RightMotor*(imu.pitch - pitchReference)+Kd_RightMotor*(pitchDot)+Ki_RightMotor*integral;
		// Control Motors
		rightMotor.pwmSpeed(rightMotorSpeed,15);
		leftMotor.pwmSpeed(leftMotorSpeed,15);
		
		
		
		// Write To LCD for output
		//mrLCD.Send_A_String_To_Position("PDH: ",1,0); // See Elapsed Time
		//int xgyro = imu.mapVal(imu.gyroData[0],-32768,32767,-250,250);
		//mrLCD.Send_An_Integer_To_Position_Decimal(pitchDot,1,5);
		
		mrLCD.Send_A_String_To_Position("Ph: ",1,0);
		mrLCD.Send_An_Integer_To_Position_Decimal(imu.pitch,1,4);
		
		mrLCD.Send_A_String_To_Position("PDh: ",2,1);
		mrLCD.Send_An_Integer_To_Position_Decimal(h,2,6);
		
		//mrLCD.Send_A_String_To_Position("LM:",2,1);
		//mrLCD.Send_An_Integer_To_Position_Decimal(leftMotorSpeed,2,5);
		
		//mrLCD.Send_A_String_To_Position("RM:",2,8);
		//mrLCD.Send_An_Integer_To_Position_Decimal(rightMotorSpeed,2,12);
		
		
		_delay_ms(1);
	}
}




ISR(TIMER1_OVF_vect)
{
	_Timer1.updateTicTocInsideInterupt();
}