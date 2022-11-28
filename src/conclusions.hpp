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