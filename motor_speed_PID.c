/* Includes */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* Defines */
#define F_CPU 16000000
#define BAUD_RATE 9600
#define FOSC 16000000
#define TX_Buffer 255

/* Prototype Function declarations */
void initMotor(void);
void initEncoder(void);
void initTimer(void);
void Motor_Clockwise(float input);
void Motor_CounterClockwise(float input);
void initUSART(void);
void USART_Transmit(unsigned char data);
void StoreSerial(char ToWrite);
void SerialWriteNumber(void);
void PrintDelay (float targetspeed, float currentspeed, float pwmpercent);
void PIDController(float Target_Speed, float Speed_Input);
float Enc_to_Degrees(float raw_count);
float SpeedControl(void);

/* Variable definitions */
static float duty = 0.0;
static float PWM_Percent;

char SerialBuffer[TX_Buffer];
static uint8_t ReadPosition = 0;
static uint8_t WritePosition = 0;
char StringArray[100];
static int chararray = 0;

volatile static float EncoderCounts = 0.0;
volatile static float prev_EncoderCounts = 0.0;
static float degrees = 0.0;

static float control_signal = 0.0;
volatile static float RPM = 0.0;

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Main function
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
int main(void) {
/*	Initialize Encoder, Interrupts, Motor Control, Timer and USART */
	initMotor();
	initEncoder();
	initUSART();
	initTimer();
/*	Enable interrupts */
	sei();

	while(1) {

	}
	return 0;
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			ISR(PCINT2_vect)
	Input:			None
	Description:	Interrupt that fires when change is detected in Pin 3. 
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
ISR(PCINT2_vect) {
	if (PIND & (1<<PD3)) {
	/*	If pin 3 is high and pin 9 is high (CW rotation) add count */
		if (PINB & (1<<PB1)) {
			EncoderCounts++;
	/*	If pin 3 is high and pin 9 is low (CCW rotation) subtract count */
		} else if ((PINB & (1<<PB1)) == 0) {
			EncoderCounts--;
		}
		
	} else if ((PIND & (1<<PD3)) == 0) {
	/*	If pin 3 is low and pin 9 is low (CCW rotation) subtract count */
		if (PINB & (1<<PB1)) {
			EncoderCounts--;
	/*	If pin 3 is low and pin 9 is low (CW rotation) add count */
		} else if ((PINB & (1<<PB1)) == 0) {
			EncoderCounts++;
		}
	}
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			ISR(PCINT0_vect)
	Input:			None
	Description:	Interrupt that fires when change is detected in Pin 9
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
ISR(PCINT0_vect) {
	if (PINB & (1<<PB1)) {
	/*	If pin 9 is high and pin 3 is high (CCW rotation) add count */
		if (PIND & (1<<PD3)) {
			EncoderCounts--;
	/*	If pin 9 is high and pin 3 is low (CW rotation) subtract count */
		} else if ((PIND & (1<<PD3)) == 0) {
			EncoderCounts++;
		}
	} else if ((PINB & (1<<PB1)) == 0) {
	/*	If pin 9 is low and pin 3 is high (CW rotation) add count */
		if (PIND & (1<<PD3)) {
			EncoderCounts++;
	/*	If pin 9 is low and pin 3 is low (CCW rotation) add count */
		} else if ((PIND & (1<<PD3)) == 0) {
			EncoderCounts--;
		}
	}
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			ISR(TIMER2_COMPA_vect)
	Input:			None
	Description:	1 millisecond tick.
					Interrupt that fires when compare match triggers on OCR2A = 250 
					Given 16,000,000 Hz clock speed, pre scaler = 64 and 8 bit max as 255.
					OCR2A = 250 = 1000 microseconds = 1 millisecond
					Timer2Counter = 1 = 1 millisecond.
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
ISR(TIMER2_COMPA_vect) {
/*	Define Variables and counters */
	volatile static int Timer2Counter_A = 0;
	volatile static int Timer2Counter_B = 0;
	volatile static int Timer2Counter_PID = 0;
	volatile static int CounterA = 1;
	
	volatile static int RampUpComplete = 0;
	volatile static float RPM_30 = 30.0;
	volatile static float RPM_60 = 60.0;
	volatile static float RPM_180 = 180.0;
	volatile static float RPM_400 = 400.0;
	volatile static float Target_RPM = 0.0;

/*	Read initial encoder reading and save to variable prev_EncoderCounts */
	if (Timer2Counter_A == CounterA) {
		prev_EncoderCounts = EncoderCounts;
	}
/*	After 50 milliseconds, read new encoder reading. Calculate RPM via SpeedControl(). Reset Timer2Counter_A */
	if (Timer2Counter_A == (CounterA + 50)) {
		SpeedControl();
		Timer2Counter_A = 0;
	}

/*	1000 counts = 1 second. Begin counting to create ramp up and ramp down of Target motor speed
	Giving 7 seconds at each speed allows 2 seconds of time to reach steady state RPM 
	
	Ramp up of target RPM */	
	if (RampUpComplete == 0) {
		if (Timer2Counter_PID <= 7000) {
			Target_RPM = RPM_30;
		} else if (Timer2Counter_PID > 7000 && Timer2Counter_PID <= 14000) {
			Target_RPM = RPM_60;
		} else if (Timer2Counter_PID > 14000 && Timer2Counter_PID <= 21000) {
			Target_RPM = RPM_180;
		} else if (Timer2Counter_PID > 21000 && Timer2Counter_PID <= 28000) {
			Target_RPM = RPM_400;
		} else if (Timer2Counter_PID == 28001) {
			RampUpComplete = 1;
		}
	}

/*	Ramp down of target RPM */
	if (RampUpComplete == 1) {
		if (Timer2Counter_PID <= 28001 && Timer2Counter_PID > 21001) {
			Target_RPM = RPM_400;
		} else if (Timer2Counter_PID <= 21001 && Timer2Counter_PID > 14001) {
			Target_RPM = RPM_180;
		} else if (Timer2Counter_PID <= 14001 && Timer2Counter_PID > 7001) {
			Target_RPM = RPM_60;
		} else if (Timer2Counter_PID <= 7001) {
			Target_RPM = RPM_30;
		} 
		if (Timer2Counter_PID == 0) {
			RampUpComplete = 0;
		}
	}
	
/*	Run PID Control loop every 10 milliseconds */	
	if (Timer2Counter_B % 10 == 0) {
		PIDController(Target_RPM,RPM);
	}

/*	Transmit to USART every 500 milliseconds. Output Target Speed, Current Speed and PWM duty cycle */
	if (Timer2Counter_B % 500 == 0) {
		PrintDelay(Target_RPM,RPM,PWM_Percent);
	}
	
/*	Increment Counters once every time Timer2 Interrupt is fired */
	Timer2Counter_A++;
	Timer2Counter_B++;
	
/*	Increment counter during ramp up */
	if (Timer2Counter_PID <= 28001 && RampUpComplete == 0) {
		Timer2Counter_PID++;
	}
/*	Decrement counter during ramp down */
	if (Timer2Counter_PID >= 0 && RampUpComplete == 1) {
		Timer2Counter_PID--;
	}
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			initMotor()
	Input:			void
	Description:	Initialize Power Control registers to allow for CTC timer and use of Compare and Match to generate PWM 
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void initMotor(void) {
	
/*	Set to fast PWM mode with OCR0A or OCR0B as TOP*/
	TCCR0A |= (1<<COM0A1)|(1<<WGM01)|(1<<WGM00);
	
/*	Set prescaler = 0*/
	TCCR0B |= (1<<CS00);
	
/*	Init counter. Start at 0 for Compare and Match */
	TCNT0 = 0;
	OCR0A = 0;

/*	Set PORTD Pin 6 and Pin 7 as output for motor control */
	DDRD |= (1 << PD6);
	DDRD |= (1 << PD7);
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			initEncoder()
	Input:			void
	Description:	Initialize Interrupt pins for Encoder phase A and B
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void initEncoder(void) {
	
/*	Set PD3 and PB1 as input pins */
	DDRD &= ~(1<<PD3); 
	DDRB &= ~(1<<PB1);
	
/*	Enable pullup for PD3 and PB1 */
	PORTD |= (1<<PD3);
	PORTB |= (1<<PB1);
	
/*	Enable PCIE2 and PCIE0 interrupts */ 
	PCICR |= (1 << PCIE2)|(1 << PCIE0);
	
/*	Enable interrupt on Pin PCINT1 = PB1 = Digital Pin 9 */
	PCMSK0 |= (1 << PCINT1);
/*	Enable interrupt on Pin PCINT19 = PD3 = Digital Pin 3 */
	PCMSK2 |= (1 << PCINT19);
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			initTimer()
	Input:			void
	Description:	Initialize Timer registers 
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void initTimer(void) {
	
/*	Set OCR2A as Top and Clear OCR2A on Compare Match, Non PWM MODE - CTC */
	TCCR2A |= (1<<WGM21);
	
/*	Set prescaler = 64 and enable Timer*/
	TCCR2B |= (1<<CS22);

/*	Enable Timer 2 Phase A */	
	TIMSK2 |= (1<<OCIE2A);
	
/*	Init counter. Start at 0 for Compare and Match */
	TCNT2 = 0;
	OCR2A = 250;

}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			Enc_to_Degrees(float raw_count)			//	Encoder Count		Degrees
	Input:			Raw encoder count						//--------------------------------
	Description:	Convert encoder count to degrees.		//		480				90
															//		960				180
															//		1440			270
															//		1920			360
															//		5.3333			1
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
float Enc_to_Degrees(float raw_count) {
	const float deg_conversion = 5.3333;
	degrees = EncoderCounts/deg_conversion;
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			Motor_Clockwise(float input)
	Input:			Integer value of percentage of duty cycle
	Description:	Set PD7 LOW to dictate Clockwise rotation and set OCR0A to input duty cycle
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void Motor_Clockwise(float input) {
/*	Take input from 0% to 100% as integer 
	Calculate double value of duty cycle based on integer input
	and round double down to nearest integer */
	PORTD &= ~(1 << PD7);
	const float maxdutycycle = 255.0;
	duty = floor((double)(maxdutycycle*input*0.01));
/*	Calculate percentage duty cycle to be used to transmit to USART */
	PWM_Percent = (float)duty / (float)maxdutycycle * 100;

/*	Maximum OCR0A is 255
	If duty cycle exceeds hardware limitations, set to 255 or 0 with respect to input */	
	if (duty > 255) {
		duty = 255;
	} else if (duty < 0) {
		duty = 0;
	} else if (duty < 1) {
		duty = 1;
	}
	OCR0A = duty;
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			Motor_CounterClockwise(float input)
	Input:			Integer value of percentage of duty cycle
	Description:	Set PD7 HIGH to dictate Counter-Clockwise rotation and set OCR0A to input duty cycle
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void Motor_CounterClockwise(float input) {
/*	Take input from 0% to 100% as integer 
	Calculate double value of duty cycle based on integer input
	and round double down to nearest integer */
	PORTD |= (1 << PD7);
	const float maxdutycycle = 255.0;
	duty = floor((double)(maxdutycycle*input*0.01));
/*	Calculate percentage duty cycle to be used to transmit to USART */
	PWM_Percent = (float)duty / (float)255 * 100;

/*	Maximum OCR0A is 255
	If duty cycle exceeds hardware limitations, set to 255 or 0 with respect to input */	
	if (duty > 255) {
		duty = 255;
	} else if (duty < 0) {
		duty = 0;
	} else if (duty < 1) {
		duty = 1;
	}
	OCR0A = duty;
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			initUSART(void)
	Input:			None
	Description:	Initialize USART registers, set baud rate, and enable serial communication
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void initUSART(void) {
/*	Baud Rate = 9600, Clock speed is 16 MHz, Baud = 103 for 0.2% error
	Baud = ((ClockSpeed/BaudRate/16) - 1)
	Set baud */
	UBRR0H = (unsigned char)(103>>8);
	UBRR0L = (unsigned char)(103);
/*	Enable transmitter and receiver */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); //|(1<<TXCIE0)
/*	Set frame format: 8 bit data, 1 stop bit */
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);

}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			USART_Transmit(unsigned char data)
	Input:			unsigned character to be transmitted	
	Description:	Waits for empty transmission buffer and places data into UDR0 buffer to be transmitted
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void USART_Transmit(unsigned char data) {
	
	if (chararray == 0) {
/*		Do nothing and wait for empty transmit buffer */
		while ( !( UCSR0A & (1<<UDRE0)) ) {
		}
/*		Put data into buffer, sends the data */
		UDR0 = data;
		
	} else if (chararray == 1) {
		
		if (WritePosition < sizeof(StringArray)) {
			WritePosition++;
		}
		
		if (ReadPosition != WritePosition) {
			UDR0 = SerialBuffer[ReadPosition];
			ReadPosition++;
				
			if (ReadPosition >= TX_Buffer) {
				ReadPosition = 0;
			}
		} else if (ReadPosition == WritePosition) {
			ReadPosition = 0;
			WritePosition = 0;
		}
		chararray = 0;
	}

}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			StoreSerial(char ToWrite)
	Input:			Character elements to be written
	Description:	Writes bit to buffer array. Resets array position once buffer limit is reached/exceeded
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void StoreSerial(char ToWrite) {
	SerialBuffer[WritePosition] = ToWrite;
	USART_Transmit(SerialBuffer[WritePosition]);
	
	if (WritePosition >= TX_Buffer) {
		WritePosition = 0;
	}
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			SerialWrite(void)
	Input:			None	
	Description:	Sends each element of StringArray to be stored and transmitted
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void SerialWrite(void) {
	chararray = 1;
	/*	Increment through character area so all elements are sent to StoreSerial(); */
	for (uint8_t n = 0; n < strlen(StringArray); n++) {
		StoreSerial(StringArray[n]);
	}

	/*	Send empty bit until bit is set */
	if (UCSR0A & (1<<UDRE0)) {
		UDR0 = 0;
	}
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			PrintDelay (float targetspeed, float currentspeed, float pwmpercent)
	Input:			Integer value to be printed 
	Description:	Provides float value within sprintf function to assign to StringArray 
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void PrintDelay (float targetspeed, float currentspeed, float pwmpercent) {
	sprintf(StringArray,"\n\nTarget Speed (RPM) = %0.1f\n\n\rCurrent Speed (RPM) = %0.1f\n\n\rPWM = %0.1f %%\n\n\n\n\r",targetspeed,currentspeed,pwmpercent);
	SerialWrite();
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			SpeedControl(void);
	Input:			None
	Description:	Calculate RPM based on change to encoder reading after a 50 millisecond time frame
							  (EncoderCount - prev_EncoderCounts)
					RPM = 	  -----------------------------------	*	1 ms * 1000 * 60 seconds			(For 1 millisecond time frame)
									   64 PPR * 18.75 GR 
										
					Motor count 1200 per 1 rotation of motor shaft. 60,000 milliseconds in 1 minute.
					Therefore, for 50 millisecond time frame, 1200 segments of 50 milliseconds in 1 minute. 
					
					50 millisecond * 20 increments in 1 second * 60 seconds / 64 PPR / 18.75 Gear Ratio
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
float SpeedControl(void) {
/*	Since encoder reading changes every 50 millisecond within Interrupt, equation is updated to the following */
	RPM = ((EncoderCounts - prev_EncoderCounts)/1200.0)*1200.0;
}

/*	------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Name:			PIDController(float Target_Speed, float Speed_Input)
	Input:			Target_Speed = Desired motor speed in RPM
					Speed_Input = Current motor speed in RPM based on calculations performed in SpeedControl() function
	Description:	PID Controller to control output to motor to reach desired motor speed based on current motor speed in RPM. 
	------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/
void PIDController(float Target_Speed, float Speed_Input) {
/*	Variable setup */
	volatile static float Kp;
	volatile static float Kd;	
	volatile static float Ki;	
	
	volatile static float derivative = 0.0;
	volatile static float integral = 0.0;
	volatile static float error_signal = 0.0;
	volatile static float prev_error = 0.0;		
	
/*	Individual Kp, Kd, and Ki gain values depending on target speed to ensure that controller performance is better optimized */
	if (Target_Speed == 30.0) {	
		Kp = 0.06; // 0.05
		Kd = 0.5; // 1.25
		Ki = 0.001; // 0.0005
	} else if (Target_Speed == 60.0) {
		Kp = 0.025;
		Kd = 0.15; // 0.1
		Ki = 0.0005; // 0.0001
	} else if (Target_Speed == 180.0 || Target_Speed == 400.0) {
		Kp = 0.2;
		Kd = 0.5;
		Ki = 0.005;
	}	
	
/*	PID Calculations for Proportional, Derivative, and Integral gains */
	error_signal = Target_Speed - Speed_Input;	
	derivative = (error_signal - prev_error);
	integral = (integral + error_signal);
	control_signal = (Kp*error_signal) + (Ki*integral) + (Kd*derivative);
	
/*	If control signal is positive, set PWM to clockwise, if negative set PWM to counter-clockwise */
	if (control_signal > 0.0) {
		Motor_Clockwise(control_signal);
	} else if (control_signal < 0.0) {
		Motor_CounterClockwise(-control_signal);
	}	
	prev_error = error_signal;

}

