#include <Arduino.h>
#include "esp32-I2C-LCD.h"
#include "esp32-recieve-plus-send.h"
/*
main esp should just receive whatever message earth sends to it
and just print it out on the LCD screen...
*/
uint8_t empty_mac[6] ={0};
recieve_plus_send main_esp = recieve_plus_send(0,empty_mac);
LCD score_tracker = LCD(GPIO_NUM_21,GPIO_NUM_22,0x27);
unsigned long previousmicrosecond = 0;
bool received_or_not =false;
void setup(){
delay(100);
Serial.begin(115200);
received_or_not = main_esp.Recieve();
score_tracker.i2cinit();
}
void loop(){
unsigned long currentmicrosecond = micros();
unsigned long microinterval =currentmicrosecond-previousmicrosecond;
if(microinterval>=60){
    if(received_or_not==true){
        score_tracker.clear();
        Serial.printf("We just scored %d\n",main_esp.getMessage());
        static int score =0;
         score+=main_esp.getMessage();
        score_tracker.print("Score: "+ std::to_string(score));
    }else{
        Serial.printf("No incoming points....");
    }
}else{

}
}