#pragma once

//kontrola koncovych dojezdu
void check_conclusion(){

//koncovy bod zdvihu celeho ramene
    printf("kontrola koncaku \n");
    if (gpio_get_level(opto2) == 1){
        conclusion=1;

        //return;
        }
//koncovy bod pro optozavoru otaceni podstavy
    if (gpio_get_level(opto1) == 1){
        conclusion=1;

        //return;
        }
//koncovy bod stisk klepet
    if (gpio_get_level(opto4) == 1){
        conclusion=1;

       // return;
        }
//koncovy bod ramene s klepety
    if (gpio_get_level(opto3) == 1){    
        conclusion=1;

        //return;
        }
}

void pushBack(int optol, int pushTime){
}

void synchronizeMotor(Driver driver, gpio_num_t opto, int direction){
    driver.set_speed(motor_speed * direction);
    while(1){
        if(opto == 1){
            driver.set_speed(0);
            return;
        }
    }
}
void synchronizeAllMotors(Driver driver0, Driver driver1, Driver driver2, Driver driver3, gpio_num_t opto0, gpio_num_t opro1, gpio_num_t opro2, gpio_num_t opro3){
    synchronizeMotor(driver0, opto0, 1);
    synchronizeMotor(driver1, opto1, 1);
    synchronizeMotor(driver2, opto2, -1);
    synchronizeMotor(driver3, opto3, -1);
}