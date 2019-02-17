/* Name: main.c
 * Author: Yuanyuan Wang,Victor Janniaud   
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include "teensy_general.h"  // includes the resources included in the teensy_general.h file
#include "t_usb.h"

void setup1stPWM(){
    // set prescaler = /64
    clear(TCCR1B,CS12);
    set(TCCR1B,CS11);
    set(TCCR1B,CS10);

    // set PWM mode 14
    set(TCCR1B,WGM13);
    set(TCCR1B,WGM12);
    set(TCCR1A,WGM11);
    clear(TCCR1A,WGM10);
}

void setup2ndPWM(){
    // set prescaler = /1
    clear(TCCR3B,CS32);
    clear(TCCR3B,CS31);
    set(TCCR3B,CS30);

    // (mode 14) UP to ICR3, PWM mode
    set(TCCR3B,WGM33);
    set(TCCR3B,WGM32);
    set(TCCR3A,WGM31);
    clear(TCCR3A,WGM30);
}

void setup1stPort(int duty_cycle){
    set(DDRB,6);
    // set(PORTB,6);

    // set output OC1B B6
    set(TCCR1A,COM1B1);
    clear(TCCR1A,COM1B0);

    // set freq
    ICR1 = 5000; 
    OCR1B = 50*duty_cycle;
}

void setup2ndPort(int duty_cycle){
    set(DDRC,6);

    // set output
    set(TCCR3A,COM3A1);
    clear(TCCR3A,COM3A0);

    // set freq
    ICR3 = 5000;
    OCR3A = 50 * duty_cycle;
}

void set1AHigh(){
    set(PORTD,2);
}

void set1ALow(){
    clear(PORTD,2);
}

void set2AHigh(){
    set(PORTD,1);
}

void set2ALow(){
    clear(PORTD,1);
}

void set3AHigh(){
    set(PORTB,5);
}

void set3ALow(){
    clear(PORTB,5);
}

void set4AHigh(){
    set(PORTB,4);
}

void set4ALow(){
    clear(PORTB,4);
}

void motor1clockwise(){
    set1AHigh();
    set2ALow();
}

void motor1anticlockwise(){
    set1ALow();
    set2AHigh();
}

void motor2clockwise(){
    set3AHigh();
    set4ALow();
}

void motor2anticlockwise(){
    set3ALow();
    set4AHigh();
}

int main(void){
    // set system clock 
    teensy_clockdivide(0); //16MHz 
    
    // turn on 1,2EN
    setup1stPWM();
    // turn on 3,4EN 
    setup2ndPWM();

    // set output port for DIR
    set(DDRD,1);
    set(DDRD,2);
    set(DDRB,4);
    set(DDRB,5);

    for(;;){
        // enable 1,2 EN side motor
        // setupEN();
        int i;
        // 1st direction
        // ramp up 
        
        for(i=0;i<=100;i++){
            setup1stPort(i);
            setup2ndPort(i);
            motor1clockwise();
            motor2clockwise();
            teensy_wait(15);
        }
        // ramp down 
        for(i=100;i>=0;i--){
            setup1stPort(i);
            setup2ndPort(i);
            motor1clockwise();
            motor2clockwise();
            teensy_wait(15);
        }
        
        // 2nd direction
        for(i=0;i<=100;i++){
            setup1stPort(i);
            setup2ndPort(i);
            motor1anticlockwise();
            motor2anticlockwise();
            teensy_wait(15);
        }
        // ramp down 
        for(i=100;i>=0;i--){
            setup1stPort(i);
            setup2ndPort(i);
            motor1anticlockwise();
            motor2anticlockwise();
            teensy_wait(15);
        }
    }
    return 0; /* never reached */
}