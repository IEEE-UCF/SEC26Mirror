#include "photodiode.h"
namespace RawDrivers{
photodiode *photodiode::s_instance = nullptr;
bool photodiode::init(){
s_instance =this;
s_instance->photodiode_pinn = _setup.photodiode_pin;
pinMode(s_instance->photodiode_pinn, INPUT_DISABLE);    
s_instance->adc->adc0->setReference(ADC_REFERENCE::REF_3V3);
s_instance->adc->adc0->setAveraging(4);
s_instance->adc->adc0->setResolution(12);
s_instance->adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
s_instance->adc->adc0->enableInterrupts(s_instance->interrupt_service_routine);
s_instance->adc->adc0->startSingleRead(s_instance->photodiode_pinn);
s_instance->photo_diode_state = NOTREADY;
return true;
}
void photodiode::interrupt_service_routine(){
    if(s_instance!=nullptr){
        s_instance->photo_diode_reading = s_instance->adc->adc0->readSingle();
        s_instance->adc->adc0->startSingleRead(s_instance->photodiode_pinn);
    }
}
void photodiode::update(){
    unsigned long currentmicrosecond = micros();
    unsigned long microinterval = currentmicrosecond-previousmicrosecond;
    static int count=0;
    if(microinterval>=20){
        previousmicrosecond = currentmicrosecond;
        if(s_instance->photo_diode_state == NOTREADY){
            count++;
            s_instance->ambient_light_threshold+=s_instance->photo_diode_reading;
            if(count==10){
                s_instance->ambient_light_threshold = s_instance->ambient_light_threshold/count;
                count=0;
                s_instance->photo_diode_state = AWAITING;    
            }
        }else if(s_instance->photo_diode_state == AWAITING){//the 1500 can be changed currently don't have any teensys to mess with...
            if(s_instance->photo_diode_reading >=(s_instance->ambient_light_threshold+1500)){
                s_instance->photo_diode_state = READY;
            }
        }
    }else{

    }
}
bool photodiode::commence(){
    if(s_instance->photo_diode_state == READY){
        return true;
    }else{
        return false;
    }
}
}
