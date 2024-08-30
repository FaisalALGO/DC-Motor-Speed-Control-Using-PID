/*
Connections

Encoder output A --> PD3     
Encoder output B --> PD2  
-----------------------------
PD6 --> L298n Driver Input 1 (IN1)
PD5 --> L298n Driver Input 2 (IN2)
-----------------------------------
L298n Driver 5V output --> Vcc encoder
L298n Driver output 1 (OUT1) --> motor power terminal (RED)
L298n Driver output 2 (OUT2) --> motor power terminal (BLACK)

*/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

volatile signed int count = 0; 
volatile unsigned int state_time = 0;
volatile unsigned int print_time = 0;
volatile unsigned char phase = 1; // Current phase
volatile unsigned char previous_phase = 1; // Previous phase
volatile unsigned long timer1_count = 0; // timer overflow count
volatile float rpm = 0.0;
volatile float target_rpm = 30.0;
volatile float error, derivative_error;//variables to store error and derivative of error
volatile float integral_error = 0.0;
volatile float previous_error = 0.0;

volatile int16_t pwm_value = 0;
volatile float control_signal = 0;

volatile float  Kp = 0.35;
volatile float  Kd = 0.05;
volatile float  Ki = 0.2;
volatile short current_state = 1;

void handle_motor_state(short state) {
    switch (state) {

        case 1:
            target_rpm = 30.0;
             break;
        case 2:
            target_rpm = 60.0;
            break;
        case 3:
            target_rpm = 180.0;
            break;
        case 4:
            target_rpm = 400.0;
            break;    
        case 5:
            target_rpm = 180.0;
            break;
        case 6:
            target_rpm = 60.0;
            break;
        default:
            target_rpm = 30.0;
            break; 
    }
}

void initInterrupt(void) {
    cli();
    EIMSK |= (1 << INT0) | (1 << INT1); // Enable interrupt 0 and 1
    // Interrupt on logical change for INT0 and INT1
    // EICRA &= ~((1 << ISC01) | (1 << ISC11));
    EICRA |= (1 << ISC00) | (1 << ISC10);
    sei();
}


void initUART(unsigned int baud_rate) {
    unsigned int ubrr = 16000000 / 16 / baud_rate - 1;
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0); // Enable transmit and receive
    UCSR0C = (1 << UCSZ00) | (1 << UCSZ01); // 8-bit character
}


void transmit_string(char *s) {
    // While loop stops when it reaches null terminator
    while (*s) {
        while (!(UCSR0A & (1 << UDRE0))); // Wait until buffer is empty
        UDR0 = *s; // Put data into the buffer, which sends data
        s++; // Move to next character in the string
    }
}

void uart_print_float(float num){
    char buffer[20];
    dtostrf(num, 4, 2, buffer);
    transmit_string(buffer);

}

void transmitByte(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0))); // Wait for empty transmit buffer
    UDR0 = data; // Start transmission by writing to UDR0 register
}

/*checkphase function finds what phase the encoder is at
    PD2 --> INTO //PD2 is connected to external interrupt 0
    PD3 --> INT1 //PD3 is connected to external interrupt 1
    if INT0 is 0 and INT1 is 0  --> phase 1
    if INT0 is 0 and INT1 is 1  --> phase 2
    if INT0 is 1 and INT1 is 1  --> phase 3
    if INT0 is 1 and INT1 is 0  --> phase 4
*/
void check_phase(void) {
    if ((PIND & (1 << PIND2)) == 0 && (PIND & (1 << PIND3)) == 0) {
        phase = 1;
    } else if ((PIND & (1 << PIND2)) == 0 && (PIND & (1 << PIND3)) != 0) {
        phase = 2;
    } else if ((PIND & (1 << PIND2)) != 0 && (PIND & (1 << PIND3)) != 0) {
        phase = 3;
    } else if ((PIND & (1 << PIND2)) != 0 && (PIND & (1 << PIND3)) == 0) {
        phase = 4;
    }
}

 /*detects clockwise or counterclockwise based on phase trasnsition 
 Clockwise rotation is encoded as going from phase 1 to 2, 2 to 3, 3 to 4 and 4 back to 1. 
 The counter-clockwise transitions would be 4 to 3, 3 to 2, 2 to 1 and 1 to 4.*/
ISR(INT0_vect) {
    check_phase();
    if ((previous_phase == 1 && phase == 2) || (previous_phase == 2 && phase == 3) ||
        (previous_phase == 3 && phase == 4) || (previous_phase == 4 && phase == 1)) {
        count++;
    } else if ((previous_phase == 1 && phase == 4) || (previous_phase == 2 && phase == 1) ||
               (previous_phase == 3 && phase == 2) || (previous_phase == 4 && phase == 3)) {
        count--;
    }
    previous_phase = phase;
}


ISR(INT1_vect) {
    check_phase();
    if ((previous_phase == 1 && phase == 2) || (previous_phase == 2 && phase == 3) ||
        (previous_phase == 3 && phase == 4) || (previous_phase == 4 && phase == 1)) {
        count++;
    } else if ((previous_phase == 1 && phase == 4) || (previous_phase == 2 && phase == 1) ||
               (previous_phase == 3 && phase == 2) || (previous_phase == 4 && phase == 3)) {
        count--;
    }
    previous_phase = phase;
}

void initMotors(void){
   TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0A1) | (1 << COM0B1); // Fast PWM
   TCCR0B = (1 << CS02) | (1 << CS00); // Prescaler of 1024 -- (16000000/(256*1024) = 61Hz)
   DDRD |= (1 << PD5) | (1 << PD6); // Set PD6 as output (OC0A pin) and PD5 as output (OC0B pin)
}

//Setting PWM for Motor direction and speed control
void PWM(int16_t M_Power){
   if (M_Power > 0){
       OCR0A = M_Power;// Set the PWM duty cycle - forward
       OCR0B = 0;}
   else if (M_Power < 0){
       OCR0A = 0;
       OCR0B = -1 * M_Power;}// Set the PWM duty cycle - reverse
   else {
       OCR0A = M_Power;//simulates a brake when PWM is 0
       OCR0B = -1 * M_Power;//simulates brake when PWM is 0
   }
}

void initTimer1(void) {
    // Set Timer1 to CTC mode
    TCCR1B |= (1 << WGM12);
    // Set the compare match value for 10 ms interval
    OCR1A = 2500 - 1;  // Calculate and set OCR1A for 10 ms interrupt
    // Enable the Timer1 compare match interrupt
    TIMSK1 |= (1 << OCIE1A);
    // Set the prescaler to 64 and start Timer1
    TCCR1B |= (1 << CS11) | (1 << CS10);
}

//executes every 10ms
ISR(TIMER1_COMPA_vect) {
    timer1_count++;
    if (timer1_count >= 10){
    //previous_state_time = timer1_count;    
    rpm = ((count / 1200.0) * 600.0);
    error =  target_rpm - rpm;
    derivative_error = error - previous_error;
    control_signal = Kp*error + Kd*derivative_error + Ki*integral_error;   
    previous_error = error;
    integral_error += error; 
    if (control_signal < -255.0) {
    control_signal = -255.0;
    } else if (control_signal > 255.0) {
    control_signal = 255.0;
    }
    pwm_value = (int16_t) control_signal;//casting control signal from float to 16-bit integer
    PWM(pwm_value);
    count = 0;

    state_time += timer1_count;
    print_time +=timer1_count;
    timer1_count = 0;
    }
    
    //print to terminal every half a second
    if (print_time >= 50){
     transmit_string("Targeted Speed (RPM): ");
     uart_print_float(target_rpm);
     transmit_string("\n\r");

     transmit_string("Current Speed (RPM): ");
     uart_print_float(rpm);
     transmit_string("\n\r");

     transmit_string("PWM: ");
     uart_print_float(pwm_value);
     transmit_string("\n\r");
     print_time = 0;

    }
    //update state every 5 seconds
    if (state_time >= 500){
        //previous_state_time = timer1_count;  
        current_state += 1;
        handle_motor_state(current_state);
        state_time = 0;
        if (current_state == 7)
            current_state = 1;
    }
}


int main(void) {
    initUART(19200);
    initInterrupt();
    initMotors();
    initTimer1();
  
    while (1) {
 
    }
return 0;
}
