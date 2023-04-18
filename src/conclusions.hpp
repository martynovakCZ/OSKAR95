#pragma once

//kontrola koncovych dojezdu
void check_conclusion(){

//koncovy bod zdvihu celeho ramene
    printf("kontrola koncaku \n");
    if (gpio_get_level(opto1) == 1){
        conclusion=1;

        //return;
        }
//koncovy bod pro optozavoru otaceni podstavy
    if (gpio_get_level(opto0) == 1){
        conclusion=1;

        //return;
        }
//koncovy bod stisk klepet
    if (gpio_get_level(opto3) == 1){
        conclusion=1;

       // return;
        }
//koncovy bod ramene s klepety
    if (gpio_get_level(opto2) == 1){    
        conclusion=1;

        //return;
        }
}






void checkOptos(){
        printf("kontrola koncaku \n");
  while(1){
//koncovy bod stisk klepet
    if (gpio_get_level(opto1) == 1){
        conclusion=1;
        printf("opto1 \n"); 
        //return;
        }
//koncovy bod pro optozavoru otaceni podstavy
    if (gpio_get_level(opto0) == 1){
        conclusion=1;
        printf("opto0 \n"); 
        //return;
        }
//koncovy bod zdvihu celeho ramene
    if (gpio_get_level(opto3) == 1){
        conclusion=1;
        printf("opto3 \n"); 
        //return;
        }
//koncovy bod ramene s klepety
    if (gpio_get_level(opto2) == 1){    
        conclusion=1;
        printf("opto2 \n"); 
        //return;
        }
  }
}