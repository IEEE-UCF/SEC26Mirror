/*
@photodiode-light-dectector
@date 12/20/2025
@author Jonathan Hernandez-Morales
*/
#include <ADC.h>
#include <ADC_util.h>
#include <Arduino.h>
#include "BaseDriver.h"
namespace RawDrivers{
    //@breif configuration class for photo-diode
    class photodiodeSetup:public Classes::BaseSetup{
        public:
        photodiodeSetup(const char*_id, const uint8_t adc_pin)
            :Classes::BaseSetup(_id){
                photodiode_pin = adc_pin;
            }
            uint8_t photodiode_pin;
        };
    //@brief main implementation for light readings
    class photodiode:public Classes::BaseDriver{
        public:
        ~photodiode() override =default;
        photodiode(const photodiodeSetup&setup):Classes::BaseDriver(setup), _setup(setup){}
        //@brief allow interrupts and configure ADC channel for teensy
        bool init() override;
        //@brief checks whether or not the "starting" light is detected
        bool commence();
        //@brief first 10 readings are averaged--this will be our light threshold
        void update() override;
        private:
        uint16_t ambient_light_threshold =0;
        uint8_t photodiode_pinn;
        volatile uint16_t photo_diode_reading =0;
        unsigned long previousmicrosecond =0;
        static void interrupt_service_routine();
        enum states{
            AWAITING,
            READY,
            NOTREADY
        }photo_diode_state;
        ADC *adc = new ADC();
        static photodiode * s_instance;
        const photodiodeSetup &_setup; 
    };
}
