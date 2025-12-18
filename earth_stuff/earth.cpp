/*
Messages via ESP_NOW
+10---Leaving  starting area
+10---Flag outside area
+5----for each astro-duck that ends the round within any part of the luna area
+20--robot sucessfully enters and exits the crater(literally any portion of the
drive mechanism) +35--full lap around crater +30--Laucnh of the UAV, must be
"successful" launch--basically needs to lift itself and be away from the robot
+50--retreival of UAV(must land back on robot does not matter where)
+15--robot auto starts using a white LED start bar and does not require a human
to start the robot at the beginning of the competition -3--for each
unintentional collision from either UAV or robot with antennas. 
Messages via IR Transmission 
+15--each antenna that is turned on 
+20---1st connection to earth---basically first message 
+30---points for antenna identification--includes color--via robot 
+60---points for antenna identification--includes color--via robot AND transmission to earth
+15---robot ends the round in the starting area 
-15--for each improper Antennta LED
identification sent to earth

TIEBREAKERS----
1.# of correct antennas/colors reported to earth
2.# of correct antennas/colors displayed on the robot
3.# of antennas turned on
*/
#include "esp32-irNEC.hpp"
#include <Arduino.h>
#include "esp32-recieve-plus-send.h"
IRReceiver *earthir = NULL;
uint8_t mac_address_main_esp[6] = {0};
recieve_plus_send earthesp = recieve_plus_send(0,mac_address_main_esp);
unsigned long previousmicrosecond = 0;



void setup() {
  Serial.begin(115200);
  earthir = new IRReceiver(GPIO_NUM_4);
  earthesp.prereqSend_and_Receive();
}




void loop() {
  // first four are atennas, last four are colors
  static uint16_t array_of_constants[8] = {0};
  // clock
  unsigned long currentmicrosecond = micros();
  unsigned long microinterval = currentmicrosecond - previousmicrosecond;
  //
  uint16_t address = 0;
  uint16_t command = 0;
  if(microinterval>=60){
    if(earthir->checkMessage(address,command)){
        previousmicrosecond = currentmicrosecond;
        uint16_t last_eight_bits_address = address & 0xFF;
        uint16_t last_eight_bits_command = command & 0xFF;
        earthesp.sendMessage(score(last_eight_bits_address,last_eight_bits_command,array_of_constants));
      }
  }else{
    
  }
}

//functions
uint16_t score(uint16_t address, uint16_t command, uint16_t array[]){
int keepscore = 0;
static uint8_t placement=0;
    if(check(address, array)==false){
        keepscore+=15; 
        array[placement]+=address;
        placement++;
    }
    if(check(command,array)==false){
        keepscore+=15;
        array[placement]+=command;
        placement++;
    }
    if(keepscore<=0){
        return 0;
    }else{
        return keepscore;
    }
}
//is the address/command in the array of antennas and colors...if not lets add score!
bool check(uint16_t value, uint16_t array[]){
    for(int i=0; i < 8; i++){
        if(array[i]==value){
            return true;       
        }
    }
    return false;
}