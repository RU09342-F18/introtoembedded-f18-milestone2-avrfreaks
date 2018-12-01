#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>

#define F_CPU 1000000L // Clock Speed
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1


void USART_Init( unsigned int ubrr){
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;//setup usart bits
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);//tx rx enable, RX interrupt enable
}
void Timer_Init(void){
	TCCR0A = (1<<COM0A1)|(1<<WGM01)|(1<<WGM00);//fast pwm on timer0
	TCCR0B = (1<<CS01);//3.9 KHz overflow on timer 0
	OCR0A = 0;//Timer0 PWM value init

	TCCR1B = (1<<CS11)|(1<<WGM12); //timer 1 clockdiv 
	OCR1AH = (1<<6)|(1<<1);//timer 1 overflow rate high byte
	OCR1AL = 0;//low byte
	TIMSK1 = (1<<OCIE1A);//timer 1 interrupt enable
	
}
void ADC_init(void){
	ADMUX = (1<<MUX0)|(1<<ADLAR);//ADC pin 24
	ADCSRA = (1<<ADEN)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADATE);//125 KHz ADC
	ADCSRB = (1<<ADTS2)|(1<<ADTS1);//manual interrupt
}
uint8_t REF = 30;
const uint8_t 	kp 	= 128,
				ki 	= 1,
				kd 	= 40;
int16_t p,i=0,d;//Several methods for tuning the PID loop exist. The choice of method will depend largely on whether the process can be taken off-line for tuning or not. Ziegler-Nichols method is a well-known online tuning strategy. The first step in this method is setting the I and D gains to zero, increasing the P gain until a sustained and stable oscillation (as close as possible) is obtained on the output. Then the critical gain Kc and the oscillation period Pc is recorded and the P, I, and D values adjusted accordingly using the table below.
//Kc = critical gain
//Pc = oscillation period (~10s)
//		Kp		Ti		Td
//P		0.5*Kc	/		/
//PD	0.5*Kc	/		0.12Pd
//PI	0.45Kc	0.85Pc	/
//PID	0.65Kc	0.5Pc	0.12Pd	
//It was discovered that these dit not work, and these values had to be found out manually.	
uint8_t ADCtoPWM(int8_t temp, int8_t error[2]){
	uint16_t total;//made 16 bit so there's no overflows
	
	p =  kp*error[0];//P term
	if(p >= 255)
		p = 255;//no overflows or underflows
	if(p < 0)
		p=0;
		
	i = i + ki*error[0];//I term
	if(i >= 255)
		i = 255;//no overflows or underflows
	else if(i < 0)
		i=0;
		
	d =  kd * (error[1]-error[0]);
	if(p+i == 0)
		d=0;
	total = p+i+d;
	if(total >= 255)//no overflows
		return 255;
	return (uint8_t)total;//mask top 8 bits
}


uint8_t ADCtoTEMP(uint8_t ADCval){
	if(ADCval < 189)
		return (uint8_t)(((uint16_t)105*ADCval)/256 - 17);//let the comiler decide on whether to shift, swap nibbles, or something crazy, etc. 105/256 was picked because 0.41 is needed, and division should be divisible by 
	return (uint8_t)(((uint16_t)225*ADCval)/256 - 106);//same here
}
ISR(TIMER1_COMPA_vect){
//used to wake up CPU

}

ISR(ADC_vect){

}
void USART_Transmit( uint8_t data ){
	while ( !( UCSR0A & (1<<UDRE0)) );//wait until buffer is empty
	UDR0 = data;
}
void sendint(uint8_t temp){//converts binary to ascii
	uint8_t tempbytes[] = {0,0,0};
	while(temp >= 100){
		tempbytes[2]++;
		temp-=100;
	}while(temp >= 10){
		tempbytes[1]++;
		temp-=10;
	}while(temp != 0){
		tempbytes[0]++;
		temp-=1;
	}
	USART_Transmit(tempbytes[2]+48);//transmit the result over UART
	USART_Transmit(tempbytes[1]+48);
	USART_Transmit(tempbytes[0]+48);

}
uint8_t rec[] = {0,0,0,0};//received UART bits
uint8_t ctr=0;
ISR(USART_RX_vect){
	uint8_t temp = UDR0;
	rec[ctr] = temp;
	if(ctr < 4)
		ctr++;
	if(temp == '\r'){//if enter, set temp
		if(ctr == 3)
			REF = 10*(rec[0]-112)+(rec[1]-112);
		rec[0]=0;
		rec[1]=0;
		rec[2]=0;
		ctr = 0;
	}	
	else if(temp == '\b'){//if backspace, remove chars
		rec[0]=0;
		rec[1]=0;
		rec[2]=0;
		ctr = 0;
	}
	
	
}
int main(void){

	DDRB = 0xFF;
	DDRC = 0xFD;
	DDRD = 0xFF;//set all to output except used bits
	USART_Init(MYUBRR);//init uart
	Timer_Init();//init timer
	ADC_init();//init ADC
	uint8_t temp=0,ADCval = 0;
	int8_t error[] = {0,0};
	while(1){
		ADCSRA |= (1<<ADSC);//enable conversion
		asm("SEI");//set up interrupts
		SMCR = (1<<SE);//sleep enable
		asm("sleep");//sleep
		
		ADCval = ADCH;//read adc
		temp = ADCtoTEMP(ADCval);//convert to temperature
		error[0] = temp - REF;//save error
		OCR0A = ADCtoPWM(temp,error);//convert to PWM value using PID
		
		USART_Transmit('T');
		USART_Transmit(':');
		sendint(temp);//send temp
		USART_Transmit(0x09);
		USART_Transmit('R');
		USART_Transmit(':');
		sendint(REF);//send ref
		error[1] = error[0];
		//sendint(OCR0A);//pwm value
		USART_Transmit('\r');
		USART_Transmit('\n');//return, new line
	}
	return 0;
}

