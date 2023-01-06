#define GRIDUI_LAYOUT_DEFINITION
#include "vector"
#include "layout.hpp"           //  pro grafické rozhraní
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "soc/gpio_sig_map.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "defines.hpp"
#include "utils.hpp"
#include "init.hpp"
#include "uart.hpp"
#include "driver.hpp"
#include "conclusions.hpp"
#include "pcnt.hpp"
#include "stepmoving.hpp"
#include "gridui.h"         //  pro grafické rozhraní          
#include "rbprotocol.h"     //  pro grafické rozhraní
#include "rbwebserver.h"    //  pro grafické rozhraní
#include "rbwifi.h"         //  pro grafické rozhraní

using namespace rb;




static void initGridUi() {
    using namespace gridui;
    // Initialize WiFi
    WiFi::startAp("Oskar95Marty","oskaroskar");     //esp vytvoří wifi sít
    // WiFi::connect("Jmeno wifi", "Heslo");    //připojení do místní sítě
    
    // Initialize RBProtocol
    auto *protocol = new Protocol("burda", "Oskar", "Compiled at " __DATE__ " " __TIME__, [](const std::string& cmd, rbjson::Object* pkt) {
        UI.handleRbPacket(cmd, pkt);
    });
    protocol->start();
    // Start serving the web page
    rb_web_start(80);
    // Initialize the UI builder
    UI.begin(protocol);
    // Build the UI widgets. Positions/props are set in the layout, so most of the time,
    // you should only set the event handlers here.
    auto builder = gridui::Layout.begin();
    //builder.MotorSpeed.min(MOTOR_SPEED_MIN);
    //builder.MotorSpeed.max(MOTOR_SPEED_MAX);
   /* builder.StartStopButton.onPress([](Button &s){
        start_stop = true;
        printf("cau\n");
    });



    builder.StartStopButton.onRelease([](Button &s){
        start_stop = false;
        printf("ahoj\n");
    });
    builder.MotorSpeed.onChanged([](Slider &s) {
        motor_speed = int(MOTOR_SPEED_COEFICIENT * s.value());
        printf("motor_speed: %f -> %d\n", s.value(), motor_speed);   
    });

     builder.StopSensitivity.onChanged([](Slider &s) {
        motor_stop_sensitivity = int(s.value());
        printf("stop sensitivity:%f -> %d\n",s.value(), motor_stop_sensitivity);
    });

    builder.IRun.onChanged([](Slider &s) {
        i_run = int(s.value());
        printf("I_Run / 32:%f -> %d\n",s.value(), i_run);
    });

        builder.Slider1.onChanged([](Slider &s) {
        motor_speed1 = MOTOR_SPEED_COEFICIENT * s.value();
        printf("svalule:%f ; motorspeed1:%d",s.value(), motor_speed1);
    });
        builder.Slider2.onChanged([](Slider &s) {
        motor_speed2 = MOTOR_SPEED_COEFICIENT * s.value();
        printf("svalule:%f ; motorspeed2:%d",s.value(), motor_speed2);
    });
        builder.Slider3.onChanged([](Slider &s) {
        motor_speed3 = MOTOR_SPEED_COEFICIENT * s.value();
        printf("svalule:%f ; motorspeed3:%d",s.value(), motor_speed3);
    });
        builder.Slider4.onChanged([](Slider &s) {
        motor_speed4 = MOTOR_SPEED_COEFICIENT * s.value();
        printf("svalule:%f ; motorspeed4:%d",s.value(), motor_speed4);
    });*/

        builder.RucniRizeni.onRelease([](Button &s){   
            printf("RucniRizeni.onRelease \n");
            rucniRizeni_onRelease = true;

    });
        builder.ZadavaniTrasy.onRelease([](Button &s){
            printf("ZadavaniTrasy.onRelease \n");
            zadavaniTrasy_onRelease = true;
    });
        builder.PridatBod.onRelease([](Button &s){
            printf("PridatBod.onRelease \n");
            pridatBod_onRelease = true;
    });
        builder.SpustitTrasu.onRelease([](Button &s){
            printf("SpustitTrasu.onRelease \n");
            spustitTrasu_onRelease = true;
    });
        builder.CyklovatTrasu.onRelease([](Button &s){
            printf("CyklovatTrasu.onRelease \n");
            cyklovatTrasu_onRelease = true;
    });
        builder.Synchronize.onRelease([](Button &s){
            printf("Synchronize.onRelease \n");
            synchronize_onRelease = true;
    }); 


        builder.KlestePlus.onPress([](Button &s){
            printf("KlestePlus.onPress \n");
            klestePlus_onPress = true;
    });
        builder.KlestePlus.onRelease([](Button &s){
            printf("KlestePlus.onRelease \n");
            klestePlus_onRelease = true;
    });
        builder.KlesteMinus.onPress([](Button &s){
            printf("KlesteMinus.onPress \n");
            klesteMinus_onPress = true;
    });
        builder.KlesteMinus.onRelease([](Button &s){
            printf("RucniRizeni.onRelease \n");
            klesteMinus_onRelease = true;
    });
       builder.LoketPlus.onPress([](Button &s){
            printf("KlesteMinus.onRelease \n");
            loketPlus_onPress = true;
    });
        builder.LoketPlus.onRelease([](Button &s){
            printf("LoketPlus.onRelease \n");
            loketPlus_onRelease = true;
    });
        builder.LoketMinus.onPress([](Button &s){
            printf("LoketMinus.onPress \n");
            loketMinus_onPress = true;
    });
        builder.LoketMinus.onRelease([](Button &s){
            printf("LoketMinus.onRelease \n");
            loketMinus_onRelease = true;
    });
        builder.RamenoPlus.onPress([](Button &s){
            printf("RamenoPlus.onPress \n");
            ramenoPlus_onPress = true;
    });
        builder.RamenoPlus.onRelease([](Button &s){
            printf("RucniRizeni.onRelease \n");
            ramenoPlus_onRelease = true;
    });
        builder.RamenoMinus.onPress([](Button &s){
            printf("RamenoMinus.onPress \n");
            ramenoMinus_onPress = true;
    });
        builder.RamenoMinus.onRelease([](Button &s){
            printf("RamenoMinus.onRelease \n");
            ramenoMinus_onRelease = true;
    });
        builder.PodstavaPlus.onPress([](Button &s){
            printf("PodstavaPlus.onPress \n");
            podstavaPlus_onPress = true;
    });
        builder.PodstavaPlus.onRelease([](Button &s){
            printf("PodstavaPlus.onRelease \n");
            podstavaPlus_onRelease = true;
    });
        builder.PodstavaMinus.onPress([](Button &s){
            printf("PodstavaMinus.onPress \n");
            podstavaMinus_onPress = true;
    });
        builder.PodstavaMinus.onRelease([](Button &s){
            printf("PodstavaMinus.onRelease \n");
            podstavaMinus_onRelease = true;
    });


    // Commit the layout. Beyond this point, calling any builder methods on the UI is invalid.
    builder.commit();
}

static void initDriver(Driver& driver, const int iRun, const int iHold) {
    driver.init();
    vTaskDelay(100 / portTICK_PERIOD_MS);   
    uint32_t data =0;
    int result = driver.get_PWMCONF(data);
    if (result != 0){
        printf("PWMCONF driveru %d : ERROR  %d\n", driver.address(), result);}
    else{
        printf("PWMCONF driveru %d =  %08X\n", driver.address(), data);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        result = driver.get_DRV_STATUS(data);}
    if (result != 0)
        printf("DRV_STATUS driveru %d : ERROR  %d\n", driver.address(), result);
    else
        printf("DRV_STATUS driveru  %d =  %08X\n", driver.address(), data);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    driver.set_speed(0);                      // otáčení motoru se nastavuje zápisem rychlosti do driveru přes Uart
    driver.set_IHOLD_IRUN (16, 32);             // proud IHOLD (při stání) =8/32, IRUN (při běhu)= 8/32 (8/32 je minimum, 16/32 je maximum pro dluhodobější provoz)
    driver.enable();                          //zapnutí mptoru
    vTaskDelay(300 / portTICK_PERIOD_MS);     //doba stání pro nastavení automatiky driveru
    driver.set_IHOLD_IRUN (iRun, iHold);             //proud IHOLD =0, IRUN = 8/32 (při stání je motor volně otočný)
}

    void movePosition(int driver0Steps, int driver1Steps, int driver2Steps, int driver3Steps, Driver driver0, Driver driver1, Driver driver2, Driver driver3){
            //dir is expressed by mathematical sign by driverXSteps
            //PCNT UNIT //NELZE DAT DO PCNT.HPP
        
        int dir0=0; 
        int dir1=0; 
        int dir2=0; 
        int dir3=0;

        if (driver0Steps<0) {dir0=(-1); driver0Steps=driver0Steps*(-1);} else if (driver0Steps>0) {dir0=1;}
        if (driver1Steps<0) {dir1=(-1); driver1Steps=driver1Steps*(-1);} else if (driver1Steps>0) {dir1=1;}
        if (driver2Steps<0) {dir2=(-1); driver2Steps=driver2Steps*(-1);} else if (driver2Steps>0) {dir2=1;}
        if (driver3Steps<0) {dir3=(-1); driver3Steps=driver3Steps*(-1);} else if (driver3Steps>0) {dir3=1;}

        pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
        pcnt_evt_t evt;
        portBASE_TYPE res;

        int startSteps0 = FinalStep0;
        int startSteps1 = FinalStep1;
        int startSteps2 = FinalStep2;
        int startSteps3 = FinalStep3;
        driver0.set_speed(motor_speed*dir0);
        //vTaskDelay(motor_delay/portTICK_PERIOD_MS);
        driver1.set_speed(motor_speed*dir1);
        //vTaskDelay(motor_delay/portTICK_PERIOD_MS);
        driver2.set_speed(motor_speed*dir2);
        //vTaskDelay(motor_delay/portTICK_PERIOD_MS);
        driver3.set_speed(motor_speed*dir3);
        //vTaskDelay(motor_delay/portTICK_PERIOD_MS);

        while(1){
        res = xQueueReceive(pcnt_evt_queue, &evt, 0 / portTICK_PERIOD_MS);
        if (res == pdTRUE) {
           /* pcnt_get_counter_value(PCNT_UNIT_0, &pcnt0_count);
            pcnt_get_counter_value(PCNT_UNIT_1, &pcnt1_count);
            pcnt_get_counter_value(PCNT_UNIT_2, &pcnt2_count);
            pcnt_get_counter_value(PCNT_UNIT_3, &pcnt3_count);

            printf("Event PCNT unit[%d]; cnt0: %d; cnt1: %d; cnt2: %d; cnt3: %d\n", evt.unit, pcnt0_count, pcnt1_count, pcnt2_count, pcnt3_count);*/

            if (evt.status & PCNT_STATUS_H_LIM_M) {
                //printf("H_LIM EVT\n");
                switch(evt.unit) {
                    case 0:
                        FinalStep0 ++;
                        break;
                    case 1:
                        FinalStep1 ++;
                        break;
                    case 2:
                        FinalStep2 ++;
                        break;
                    case 3:
                        FinalStep3 ++;
                        break;
                    default:
                        break;
                }
            }
        } else {
                /*printf("FINALSTEP0:%d\n", FinalStep0);
                printf("FINALSTEP1:%d\n", FinalStep1);
                printf("FINALSTEP2:%d\n", FinalStep2);
                printf("FINALSTEP3:%d\n", FinalStep3);
                printf("startSteps0:%d\n", startSteps0);
                printf("startSteps1:%d\n", startSteps1);
                printf("startSteps2:%d\n", startSteps2);
                printf("startSteps3:%d\n", startSteps3);
                printf("driver0Steps:%d\n", driver0Steps);
                printf("driver1Steps:%d\n", driver1Steps);
                printf("driver2Steps:%d\n", driver2Steps);
                printf("driver3Steps:%d\n", driver3Steps);*/
            if (driver0Steps + startSteps0 <= FinalStep0){
                driver0.set_speed(0);
                vTaskDelay(motor_delay/portTICK_PERIOD_MS);
            }
            if (driver1Steps + startSteps1 <= FinalStep1){
                driver1.set_speed(0);
                vTaskDelay(motor_delay/portTICK_PERIOD_MS);
            }
            if (driver2Steps + startSteps2 <= FinalStep2){
                driver2.set_speed(0);
                vTaskDelay(motor_delay/portTICK_PERIOD_MS);
            }
            if (driver3Steps + startSteps3 <= FinalStep3){
                driver3.set_speed(0);
                vTaskDelay(motor_delay/portTICK_PERIOD_MS);
            }

            if((driver0Steps + startSteps0 <= FinalStep0) && (driver1Steps +  startSteps1 <= FinalStep1) && (driver2Steps +  startSteps2 <= FinalStep2) && (driver3Steps +  startSteps3 <= FinalStep3)) 
            {   
                driver0.set_speed(0);
                vTaskDelay(motor_delay/portTICK_PERIOD_MS);
                driver1.set_speed(0);
                vTaskDelay(motor_delay/portTICK_PERIOD_MS);
                driver2.set_speed(0);
                vTaskDelay(motor_delay/portTICK_PERIOD_MS);
                driver3.set_speed(0);
                vTaskDelay(motor_delay/portTICK_PERIOD_MS);
               /* printf("FINALSTEP0:%d\n", FinalStep0);
                printf("FINALSTEP1:%d\n", FinalStep1);
                printf("FINALSTEP2:%d\n", FinalStep2);
                printf("FINALSTEP3:%d\n", FinalStep3);*/
                return;
            }         
        }
        }
    }

    void testsynchro(Driver driver0, Driver driver1, Driver driver2, Driver driver3, gpio_num_t opto0, gpio_num_t opto1, gpio_num_t opto2, gpio_num_t opto3){
            driver0.set_speed(motor_speed1);
            driver1.set_speed(motor_speed2);
            driver2.set_speed(motor_speed*(-1));
            driver3.set_speed(motor_speed*(-1));
            while(1){

            
            if (gpio_get_level(opto0) == 1){
                driver0.set_speed(0);
                vTaskDelay(motor_delay/portTICK_PERIOD_MS);
                printf("opto opto0Stop\n");
            }
            if (gpio_get_level(opto1) == 1){
                driver1.set_speed(0);
                vTaskDelay(motor_delay/portTICK_PERIOD_MS);
                printf("opto opto1Stop\n");
            }
            if (gpio_get_level(opto2) == 1){
                driver2.set_speed(0);
                vTaskDelay(motor_delay/portTICK_PERIOD_MS);
                printf("opto opto2Stop\n");
            }
            if (gpio_get_level(opto3) == 1){
                driver3.set_speed(0);
                vTaskDelay(motor_delay/portTICK_PERIOD_MS);
                printf("opto opto3Stop\n");
            }

            if((gpio_get_level(opto0) == 1) && (gpio_get_level(opto1) == 1) && (gpio_get_level(opto2) == 1) && (gpio_get_level(opto3) == 1)) 
            {   
                driver0.set_speed(0);
                vTaskDelay(motor_delay/portTICK_PERIOD_MS);
                driver1.set_speed(0);
                vTaskDelay(motor_delay/portTICK_PERIOD_MS);
                driver2.set_speed(0);
                vTaskDelay(motor_delay/portTICK_PERIOD_MS);
                driver3.set_speed(0);
                vTaskDelay(motor_delay/portTICK_PERIOD_MS);
                printf("return optoStop\n");
                //movePosition(360, 360, 360, 360, -1, -1, 1, 1, driver0, driver1, driver2, driver3);
                return;
            }
            }
    }
    
    void count_positions_from_synchro(Driver driver0, Driver driver1, Driver driver2, Driver driver3, gpio_num_t opto0, gpio_num_t opto1, gpio_num_t opto2, gpio_num_t opto3){
            
            pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
            pcnt_evt_t evt;
            portBASE_TYPE res;

            driver0.set_speed(motor_speed1);
            driver1.set_speed(motor_speed2);
            driver2.set_speed(motor_speed*(-1));
            driver3.set_speed(motor_speed*(-1));

           /* int motor0=0;
            int motor1=0;
            int motor2=0;
            int motor3=0;*/
            
            while(1){

                res = xQueueReceive(pcnt_evt_queue, &evt, 0 / portTICK_PERIOD_MS);
                if (res == pdTRUE) {
                /* pcnt_get_counter_value(PCNT_UNIT_0, &pcnt0_count);
                    pcnt_get_counter_value(PCNT_UNIT_1, &pcnt1_count);
                    pcnt_get_counter_value(PCNT_UNIT_2, &pcnt2_count);
                    pcnt_get_counter_value(PCNT_UNIT_3, &pcnt3_count);

                    printf("Event PCNT unit[%d]; cnt0: %d; cnt1: %d; cnt2: %d; cnt3: %d\n", evt.unit, pcnt0_count, pcnt1_count, pcnt2_count, pcnt3_count);*/

                    if (evt.status & PCNT_STATUS_H_LIM_M) {
                        //printf("H_LIM EVT\n");
                        switch(evt.unit) {
                            case 0:
                                motor0 ++;
                                break;
                            case 1:
                                motor1 ++;
                                break;
                            case 2:
                                motor2 ++;
                                break;
                            case 3:
                                motor3 ++;
                                break;
                            default:
                                break;
                        }
                    }
                } else {
                        
                        if (gpio_get_level(opto0) == 1){
                            driver0.set_speed(0);
                            //printf("opto opto0Stop\n");
                        }
                        if (gpio_get_level(opto1) == 1){
                            driver1.set_speed(0);
                            //printf("opto opto1Stop\n");
                        }
                        if (gpio_get_level(opto2) == 1){
                            driver2.set_speed(0);
                            //printf("opto opto2Stop\n");
                        }
                        if (gpio_get_level(opto3) == 1){
                            driver3.set_speed(0);
                            //printf("opto opto3Stop\n");
                        }

                        if((gpio_get_level(opto0) == 1) && (gpio_get_level(opto1) == 1) && (gpio_get_level(opto2) == 1) && (gpio_get_level(opto3) == 1)) 
                        {   
                            driver0.set_speed(0);
                            driver1.set_speed(0);
                            driver2.set_speed(0);
                            driver3.set_speed(0);
                            printf("return optoStop\n");                            
                            printf("motor0: %d\n", motor0);                            
                            printf("motor1: %d\n", motor1);                            
                            printf("motor2: %d\n", motor2);                            
                            printf("motor3: %d\n", motor3);                            
                           

                            //movePosition(360, 360, 360, 360, -1, -1, 1, 1, driver0, driver1, driver2, driver3);
                            return;
                        }
                    }
    }
    }
    void count_position_from_synchro(Driver driver, gpio_num_t opto){
            
            pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
            pcnt_evt_t evt;
            portBASE_TYPE res;
            switch (opto){
                case opto0:
                    driver.set_speed(motor_speed1);
                    break;
                case opto1:
                    driver.set_speed(motor_speed2);
                    break;
                case opto2:
                    driver.set_speed(motor_speed*(-1));
                    break;
                case opto3:
                    driver.set_speed(motor_speed*(-1));
                    break;
                default:
                    break;
            }

            int motor=0;

           /* int motor0=0;
            int motor1=0;
            int motor2=0;
            int motor3=0;*/
            
            while(1){

                res = xQueueReceive(pcnt_evt_queue, &evt, 0 / portTICK_PERIOD_MS);
                if (res == pdTRUE) {
                /* pcnt_get_counter_value(PCNT_UNIT_0, &pcnt0_count);
                    pcnt_get_counter_value(PCNT_UNIT_1, &pcnt1_count);
                    pcnt_get_counter_value(PCNT_UNIT_2, &pcnt2_count);
                    pcnt_get_counter_value(PCNT_UNIT_3, &pcnt3_count);

                    printf("Event PCNT unit[%d]; cnt0: %d; cnt1: %d; cnt2: %d; cnt3: %d\n", evt.unit, pcnt0_count, pcnt1_count, pcnt2_count, pcnt3_count);*/

                    if (evt.status & PCNT_STATUS_H_LIM_M) {
                        //printf("H_LIM EVT\n");

                        motor++;
                    }
                } else {
                        
                        if (gpio_get_level(opto) == 1){
                            driver.set_speed(0);
                            printf("opto optoStop\n");
                            printf("motor: %d\n", motor);                            
                        }
                    }
    }
    }

    void moveMotor(int steps, int dir, Driver driver){   
         //dir can be just 1 or -1 to express direction of moving
        //PCNT UNIT //NELZE DAT DO PCNT.HPP
        pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
        pcnt_evt_t evt;
        portBASE_TYPE res;

        int currentSteps=0;
        driver.set_speed(motor_speed*dir);
        vTaskDelay(motor_delay/portTICK_PERIOD_MS);

        while(1){
        res = xQueueReceive(pcnt_evt_queue, &evt, 0 / portTICK_PERIOD_MS);
        if (res == pdTRUE) {
           /* pcnt_get_counter_value(PCNT_UNIT_0, &pcnt0_count);
            pcnt_get_counter_value(PCNT_UNIT_1, &pcnt1_count);
            pcnt_get_counter_value(PCNT_UNIT_2, &pcnt2_count);
            pcnt_get_counter_value(PCNT_UNIT_3, &pcnt3_count);

            printf("Event PCNT unit[%d]; cnt0: %d; cnt1: %d; cnt2: %d; cnt3: %d\n", evt.unit, pcnt0_count, pcnt1_count, pcnt2_count, pcnt3_count);*/

            if (evt.status & PCNT_STATUS_H_LIM_M) {
                //printf("H_LIM EVT\n");
                switch(evt.unit) {
                    case 0:
                        currentSteps ++;
                        break;
                    case 1:
                        currentSteps ++;
                        break;
                    case 2:
                        currentSteps ++;
                        break;
                    case 3:
                        currentSteps ++;
                        break;
                    default:
                        break;
                }
            }
        } else {
                /*printf("FINALSTEP0:%d\n", FinalStep0);
                printf("FINALSTEP1:%d\n", FinalStep1);
                printf("FINALSTEP2:%d\n", FinalStep2);
                printf("FINALSTEP3:%d\n", FinalStep3);
                printf("startSteps0:%d\n", startSteps0);
                printf("startSteps1:%d\n", startSteps1);
                printf("startSteps2:%d\n", startSteps2);
                printf("startSteps3:%d\n", startSteps3);
                printf("driver0Steps:%d\n", driver0Steps);
                printf("driver1Steps:%d\n", driver1Steps);
                printf("driver2Steps:%d\n", driver2Steps);
                printf("driver3Steps:%d\n", driver3Steps);*/
            if ( steps <= currentSteps){
                driver.set_speed(0);
                vTaskDelay(motor_delay/portTICK_PERIOD_MS);
                return;
            }
        }
        }        
    }

    void pushBack(Driver driver, int dir, int pushSteps){
    moveMotor(50, -1, driver);
    }
   
    void synchronizeMotor(Driver driver, gpio_num_t opto, int direction){
    driver.set_speed(motor_speed * direction);
        printf("opto level:2\n");
        while(gpio_get_level(opto)!=1){
            vTaskDelay(5/portTICK_PERIOD_MS);

            }
            printf("opto level:%d\n",gpio_get_level(opto));
            if(gpio_get_level(opto) == 1){
                driver.set_speed(0);
                //pushBack(driver, direction*(-1), 50);
                return; //break
        }

    }
    void synchronizeAllMotors(Driver driver0, Driver driver1, Driver driver2, Driver driver3, gpio_num_t opto0, gpio_num_t opro1, gpio_num_t opro2, gpio_num_t opro3){
        synchronizeMotor(driver0, opto0, 1);
        synchronizeMotor(driver1, opto1, 1);
        synchronizeMotor(driver2, opto2, -1);
        synchronizeMotor(driver3, opto3, -1);
    }
   
   void realTimeControl(Driver driver0, Driver driver1, Driver driver2, Driver driver3, gpio_num_t opto0, gpio_num_t opto1, gpio_num_t opto2, gpio_num_t opto3){
        pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    while(1){
        if(podstavaPlus_onPress == true){  driver0.set_speed(motor_speed0); podstavaPlus_onPress = false;}
        if(podstavaPlus_onRelease == true){    driver0.set_speed(0); podstavaPlus_onRelease = false;}
        if(podstavaMinus_onPress == true){  driver0.set_speed(motor_speed0*(-1)); podstavaMinus_onPress = false;}
        if(podstavaMinus_onRelease == true){    driver0.set_speed(0); podstavaMinus_onRelease  = false;}

        if(klestePlus_onPress == true){     driver1.set_speed(motor_speed1); klestePlus_onPress = false;}
        if(klestePlus_onRelease == true){    driver1.set_speed(0); klestePlus_onRelease = false;}
        if(klesteMinus_onPress == true){  driver1.set_speed(motor_speed1*(-1)); klesteMinus_onPress = false;}
        if(klesteMinus_onRelease == true){    driver1.set_speed(0); klesteMinus_onRelease = false;}

        if(loketPlus_onPress == true){  driver2.set_speed(motor_speed2); loketPlus_onPress = false;}
        if(loketPlus_onRelease == true){    driver2.set_speed(0); loketPlus_onRelease = false;}
        if(loketMinus_onPress == true){  driver2.set_speed(motor_speed2*(-1)); loketMinus_onPress = false;}
        if(loketMinus_onRelease == true){    driver2.set_speed(0); loketMinus_onRelease = false;}

        if(ramenoPlus_onPress == true){  driver3.set_speed(motor_speed3); ramenoPlus_onPress = false;}
        if(ramenoPlus_onRelease == true){    driver3.set_speed(0); ramenoPlus_onRelease = false;}
        if(ramenoMinus_onPress == true){  driver3.set_speed(motor_speed3*(-1)); ramenoMinus_onPress = false;}
        if(ramenoMinus_onRelease == true){    driver3.set_speed(0); ramenoMinus_onRelease = false;}

        if(zadavaniTrasy_onRelease==1){ zadavaniTrasy_onRelease = false; return;}

        vTaskDelay(50/portTICK_PERIOD_MS);
    }    
   }


   void readPoints(){
        printf("readPoints function\n");
        for(int a = 0; a<=Vdriver0.size()-1; a++){
            printf("%d. - %d    ", a, Vdriver0[a]);
            vTaskDelay(50/portTICK_PERIOD_MS);
        }
        printf("\n");

        for(int a = 0; a<=Vdriver1.size()-1; a++){
            printf("%d. - %d    ", a, Vdriver1[a]);
            vTaskDelay(50/portTICK_PERIOD_MS);
        }
        printf("\n");

        for(int a = 0; a<=Vdriver2.size()-1; a++){
            printf("%d. - %d    ", a, Vdriver2[a]);
            vTaskDelay(50/portTICK_PERIOD_MS);
        }
        printf("\n");

        for(int a = 0; a<=Vdriver3.size()-1; a++){
            printf("%d. - %d    ", a, Vdriver3[a]);
            vTaskDelay(50/portTICK_PERIOD_MS);
        }
        printf("\n");
   }
   
   void makePoints(Driver driver0, Driver driver1, Driver driver2, Driver driver3, gpio_num_t opto0, gpio_num_t opto1, gpio_num_t opto2, gpio_num_t opto3){
        testsynchro( driver0,  driver1,  driver2,  driver3,  opto0,  opto1,  opto2,  opto3);
        vTaskDelay(250/portTICK_PERIOD_MS);
        int stepsDriver0=0;
        int stepsDriver1=0;
        int stepsDriver2=0;
        int stepsDriver3=0;

        bool reverseDriver0=0;
        bool reverseDriver1=0;
        bool reverseDriver2=0;
        bool reverseDriver3=0;

        pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
        pcnt_evt_t evt;
        portBASE_TYPE res;

        while(1){

                res = xQueueReceive(pcnt_evt_queue, &evt, 0 / portTICK_PERIOD_MS);
                if (res == pdTRUE) {

                    if (evt.status & PCNT_STATUS_H_LIM_M) {
                        switch(evt.unit) {
                            case 0:
                                if(reverseDriver0 == false){
                                    stepsDriver0++;
                                }
                                else{
                                    stepsDriver0--;
                                }
                                break;
                            case 1:
                                if(reverseDriver1 == false){
                                    stepsDriver1++;
                                }
                                else{
                                    stepsDriver1--;
                                }
                                break;
                            case 2:
                                if(reverseDriver2 == false){
                                    stepsDriver2++;
                                }
                                else{
                                    stepsDriver2--;
                                }
                                break;
                            case 3:
                                if(reverseDriver3 == false){
                                    stepsDriver3++;
                                }
                                else{
                                    stepsDriver3--;
                                }
                                break;
                            default:
                                break;
                        }
                    }
                } else {
                        if(podstavaPlus_onPress == true){  driver0.set_speed(motor_speed0); reverseDriver0 = false; podstavaPlus_onPress = false;}
                        if(podstavaPlus_onRelease == true){    driver0.set_speed(0); podstavaPlus_onRelease = false;}
                        if(podstavaMinus_onPress == true){  driver0.set_speed(motor_speed0*(-1)); reverseDriver0 = true; podstavaMinus_onPress = false;}
                        if(podstavaMinus_onRelease == true){    driver0.set_speed(0); podstavaMinus_onRelease  = false;}

                        if(klestePlus_onPress == true){  driver1.set_speed(motor_speed1); reverseDriver1 = true; klestePlus_onPress = false;}
                        if(klestePlus_onRelease == true){    driver1.set_speed(0); klestePlus_onRelease = false;}
                        if(klesteMinus_onPress == true){  driver1.set_speed(motor_speed1*(-1)); reverseDriver1 = false; klesteMinus_onPress = false;}
                        if(klesteMinus_onRelease == true){    driver1.set_speed(0); klesteMinus_onRelease = false;}

                        if(loketPlus_onPress == true){  driver2.set_speed(motor_speed2); reverseDriver2 = false; loketPlus_onPress = false;}
                        if(loketPlus_onRelease == true){    driver2.set_speed(0); loketPlus_onRelease = false;}
                        if(loketMinus_onPress == true){  driver2.set_speed(motor_speed2*(-1)); reverseDriver2 = true; loketMinus_onPress = false;}
                        if(loketMinus_onRelease == true){    driver2.set_speed(0); loketMinus_onRelease = false;}

                        if(ramenoPlus_onPress == true){  driver3.set_speed(motor_speed3); reverseDriver3 = false; ramenoPlus_onPress = false;}
                        if(ramenoPlus_onRelease == true){    driver3.set_speed(0); ramenoPlus_onRelease = false;}
                        if(ramenoMinus_onPress == true){  driver3.set_speed(motor_speed3*(-1)); reverseDriver3 = true; ramenoMinus_onPress = false;}
                        if(ramenoMinus_onRelease == true){    driver3.set_speed(0); ramenoMinus_onRelease = false;}

                        if (pridatBod_onRelease == true){

                            Vdriver0.push_back(stepsDriver0);
                            Vdriver1.push_back(stepsDriver1);
                            Vdriver2.push_back(stepsDriver2);
                            Vdriver3.push_back(stepsDriver3);

                            stepsDriver0=0;
                            stepsDriver1=0;
                            stepsDriver2=0;
                            stepsDriver3=0;

                            pridatBod_onRelease = false;
                        }


                        if(spustitTrasu_onRelease==1){return;}
                        if(cyklovatTrasu_onRelease==1){return;}

                        if(rucniRizeni_onRelease==1){return;}

                        //vTaskDelay(50/portTICK_PERIOD_MS); // must be commented to accuracy

                    }
    }

   }

   void drivePointsOnce(Driver driver0, Driver driver1, Driver driver2, Driver driver3, gpio_num_t opto0, gpio_num_t opto1, gpio_num_t opto2, gpio_num_t opto3){
    readPoints();
    testsynchro(driver0, driver1, driver2, driver3, opto0, opto1, opto2, opto3);
    for (int i=0; i<= Vdriver0.size()-1; i++){
        printf("Vdriver0:   %d\n", Vdriver0[i]);
        printf("Vdriver1:   %d\n", Vdriver1[i]);
        printf("Vdriver2:   %d\n", Vdriver2[i]);
        printf("Vdriver3:   %d\n", Vdriver3[i]);
        movePosition(Vdriver0[i], Vdriver1[i], Vdriver2[i], Vdriver3[i] ,driver0, driver1, driver2, driver3);
    }
   }

   void drivePointsCycle(Driver driver0, Driver driver1, Driver driver2, Driver driver3, gpio_num_t opto0, gpio_num_t opto1, gpio_num_t opto2, gpio_num_t opto3){} 
  
extern "C" void app_main(void)
{   
    gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = GPIO_BIT_MASK;
	gpio_config(&io_conf);

    gpio_set_level(VCC_IO_0, 1); // zapnuti napajeni do driveru0 
    gpio_set_level(VCC_IO_1, 1); // zapnuti napajeni do driveru1
    gpio_set_level(VCC_IO_2, 1); // zapnuti napajeni do driveru2
    gpio_set_level(VCC_IO_3, 1); // zapnuti napajeni do driveru3

    gpio_set_level(GPIO_NUM_32, 1); // zapnuti siloveho napajeni do driveru 
    printf("Simple Motor \n\tbuild %s %s\n", __DATE__, __TIME__);
    check_reset();
    iopins_init();
    gpio_set_level(VCC_IO_0, 0);              // reset driveru
    gpio_set_level(VCC_IO_1, 0);
    gpio_set_level(VCC_IO_2, 0);
    gpio_set_level(VCC_IO_3, 0);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(VCC_IO_0, 1);              //zapíná VCCIO driveru
    gpio_set_level(VCC_IO_1, 1);
    gpio_set_level(VCC_IO_2, 1);
    gpio_set_level(VCC_IO_3, 1);
    nvs_init();                             //inicializace pro zápis do flash paměti
    Uart drivers_uart {
        DRIVERS_UART,
        Uart::config_t {
            .baud_rate = 750000,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
            .use_ref_tick = false
        },
        Uart::pins_t {
            .pin_txd = DRIVERS_UART_TXD,
            .pin_rxd = DRIVERS_UART_RXD,
            .pin_rts = UART_PIN_NO_CHANGE,
            .pin_cts = UART_PIN_NO_CHANGE
        },
        Uart::buffers_t {
            .rx_buffer_size = DRIVERS_UART_BUF_SIZE,
            .tx_buffer_size = 0,
            .event_queue_size = 0
        }
    };
    Driver driver0 { drivers_uart, DRIVER_0_ADDRES, DRIVER_0_ENABLE };
    initDriver(driver0, 16, 32);

    Driver driver1 { drivers_uart, DRIVER_1_ADDRES, DRIVER_1_ENABLE };
    initDriver(driver1, 16, 32);

    Driver driver2 { drivers_uart, DRIVER_2_ADDRES, DRIVER_2_ENABLE };
    initDriver(driver2, 16, 32);

    Driver driver3 { drivers_uart, DRIVER_3_ADDRES, DRIVER_3_ENABLE };
    initDriver(driver3, 16, 32);

    




    initGridUi();





    
    printf("opto level begin\n");


    

    testsynchro(driver0, driver1, driver2, driver3, opto0, opto1, opto2, opto3);

    IndexStepCounter_init(PCNT_UNIT_0, GPIO_NUM_12, GPIO_NUM_0); //testsynchro must be before this init
    IndexStepCounter_init(PCNT_UNIT_1, GPIO_NUM_18, GPIO_NUM_0);
    IndexStepCounter_init(PCNT_UNIT_2, GPIO_NUM_15, GPIO_NUM_0);
    IndexStepCounter_init(PCNT_UNIT_3, GPIO_NUM_13, GPIO_NUM_0);

    //count_positions_from_synchro(driver0, driver1, driver2, driver3, opto0, opto1, opto2, opto3);
    //movePosition(motor0, motor1, motor2, motor3, driver0, driver1, driver2, driver3);
    

    while(1){
        if (synchronize_onRelease == 1) { testsynchro(driver0, driver1, driver2, driver3, opto0, opto1, opto2, opto3); synchronize_onRelease = false;}

        if (rucniRizeni_onRelease == 1) {realTimeControl(driver0, driver1, driver2, driver3, opto0, opto1, opto2, opto3); rucniRizeni_onRelease = false;}

        if (zadavaniTrasy_onRelease == 1) {makePoints(driver0, driver1, driver2, driver3, opto0, opto1, opto2, opto3); zadavaniTrasy_onRelease = false;}

        if (spustitTrasu_onRelease == 1) {drivePointsOnce(driver0, driver1, driver2, driver3, opto0, opto1, opto2, opto3); spustitTrasu_onRelease = false;}

        vTaskDelay(50/portTICK_PERIOD_MS);

    }

 
   // movePosition(360, 360, 360, 360, -1, -1, 1, 1, driver0, driver1, driver2, driver3);
   // vTaskDelay(500/portTICK_PERIOD_MS);

    /*movePosition(360, 360, 360, 360, -1, -1, 1, 1, driver0, driver1, driver2, driver3);
    vTaskDelay(500/portTICK_PERIOD_MS);*/

    //synchronizeMotor(driver0, opto0, 1);
/*while(1){

    vTaskDelay(500/portTICK_PERIOD_MS);
    movePosition(27*360, 2*360, 36*0, 6*360, -1, -1, 1, 1, driver0, driver1, driver2, driver3); // maximalni stupne: 40, 5, 36, 12 // podstava, klepeta, loket, rameno // -1 od opto, -1 od opto, 1 od opto, 1 od opto
    vTaskDelay(500/portTICK_PERIOD_MS);
    movePosition(10*360, 1*360, 10*360, 3*360, 1, -1, 1, -1, driver0, driver1, driver2, driver3); // maximalni stupne: 40, 5, 36, 12 // podstava, klepeta, loket, rameno // -1 od opto, -1 od opto, 1 od opto, 1 od opto
    vTaskDelay(500/portTICK_PERIOD_MS);
    movePosition(10*360, 3*360, 5*360, 5*360, 1, 1, -1, 1, driver0, driver1, driver2, driver3); // maximalni stupne: 40, 5, 36, 12 // podstava, klepeta, loket, rameno // -1 od opto, -1 od opto, 1 od opto, 1 od opto
    vTaskDelay(500/portTICK_PERIOD_MS);
    movePosition(7*360, 5*360, 5*360, 8*360, 1, -1, 1, -1, driver0, driver1, driver2, driver3); // maximalni stupne: 40, 5, 36, 12 // podstava, klepeta, loket, rameno // -1 od opto, -1 od opto, 1 od opto, 1 od opto
}
    return;*/
   
}