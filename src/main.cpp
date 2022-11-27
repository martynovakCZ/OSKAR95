#define GRIDUI_LAYOUT_DEFINITION
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
#include "gridui.h"         //  pro grafické rozhraní          
#include "rbprotocol.h"     //  pro grafické rozhraní
#include "rbwebserver.h"    //  pro grafické rozhraní
#include "rbwifi.h"         //  pro grafické rozhraní

using namespace rb;


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
    });*/

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
    initGridUi();
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
    initDriver(driver0, 16, 31);

    Driver driver1 { drivers_uart, DRIVER_1_ADDRES, DRIVER_1_ENABLE };
    initDriver(driver1, 16, 31);

    Driver driver2 { drivers_uart, DRIVER_2_ADDRES, DRIVER_2_ENABLE };
    initDriver(driver2, 16, 31);

    Driver driver3 { drivers_uart, DRIVER_3_ADDRES, DRIVER_3_ENABLE };
    initDriver(driver3, 16, 31);

    









    /* Initialize PCNT event queue and PCNT functions */
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    pcnt_evt_t evt;
    portBASE_TYPE res;

    IndexStepCounter_init(PCNT_UNIT_0, GPIO_NUM_12, GPIO_NUM_0);
    IndexStepCounter_init(PCNT_UNIT_1, GPIO_NUM_18, GPIO_NUM_0);
    IndexStepCounter_init(PCNT_UNIT_2, GPIO_NUM_15, GPIO_NUM_0);
    IndexStepCounter_init(PCNT_UNIT_3, GPIO_NUM_13, GPIO_NUM_0);



    while(1){
        res = xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);
        if (res == pdTRUE) {
            pcnt_get_counter_value(PCNT_UNIT_0, &pcnt0_count);
            pcnt_get_counter_value(PCNT_UNIT_1, &pcnt1_count);
            pcnt_get_counter_value(PCNT_UNIT_2, &pcnt2_count);
            pcnt_get_counter_value(PCNT_UNIT_3, &pcnt3_count);

            printf("Event PCNT unit[%d]; cnt0: %d; cnt1: %d; cnt2: %d; cnt3: %d\n", evt.unit, pcnt0_count, pcnt1_count, pcnt2_count, pcnt3_count);

            if (evt.status & PCNT_STATUS_H_LIM_M) {
                printf("H_LIM EVT\n");
            }
        } else {
            pcnt_get_counter_value(PCNT_UNIT_0, &pcnt0_count);
            pcnt_get_counter_value(PCNT_UNIT_1, &pcnt1_count);
            pcnt_get_counter_value(PCNT_UNIT_2, &pcnt2_count);
            pcnt_get_counter_value(PCNT_UNIT_3, &pcnt3_count);
            printf("Current counter value :%d; cnt0: %d; cnt1: %d; cnt2: %d; cnt3: %d\n", evt.unit, pcnt0_count, pcnt1_count, pcnt2_count, pcnt3_count);
        /*driver0.set_speed(motor_speed1/3);
        vTaskDelay(200/portTICK_PERIOD_MS);
        driver1.set_speed(motor_speed2/3);
        vTaskDelay(200/portTICK_PERIOD_MS);
        driver2.set_speed(motor_speed3/3);
        vTaskDelay(200/portTICK_PERIOD_MS);
        driver3.set_speed(motor_speed4/3);
        vTaskDelay(200/portTICK_PERIOD_MS);
        check_conclusion();
        if(conclusion==1){
            driver0.set_speed((motor_speed1/3)*(-1));
            vTaskDelay(600/portTICK_PERIOD_MS);
            driver1.set_speed((motor_speed2/3)*(-1));
            vTaskDelay(600/portTICK_PERIOD_MS);
            driver2.set_speed((motor_speed3/3)*(-1));
            vTaskDelay(600/portTICK_PERIOD_MS);
            driver3.set_speed((motor_speed4/3)*(-1));
            vTaskDelay(600/portTICK_PERIOD_MS);
            
            driver0.set_speed(0);
            vTaskDelay(200/portTICK_PERIOD_MS);
            driver1.set_speed(0);
            vTaskDelay(200/portTICK_PERIOD_MS);
            driver2.set_speed(0);
            vTaskDelay(200/portTICK_PERIOD_MS);
            driver3.set_speed(0);
            vTaskDelay(200/portTICK_PERIOD_MS);
            conclusion=0;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);*/


        /*driver1.set_speed(motor_speed/3);
        vTaskDelay(200/portTICK_PERIOD_MS);
        driver1.set_speed(0);*/

        driver0.set_speed(motor_speed/3);
        vTaskDelay(200/portTICK_PERIOD_MS);
        driver1.set_speed(motor_speed/3);
        vTaskDelay(200/portTICK_PERIOD_MS);
        driver2.set_speed(motor_speed/3);
        vTaskDelay(200/portTICK_PERIOD_MS);
        driver3.set_speed(motor_speed/3);
        vTaskDelay(200/portTICK_PERIOD_MS);
        /*pcnt_get_counter_value(PCNT_UNIT_0, &pcnt0_count);
        pcnt_get_counter_value(PCNT_UNIT_1, &pcnt1_count);
        pcnt_get_counter_value(PCNT_UNIT_2, &pcnt2_count);
        pcnt_get_counter_value(PCNT_UNIT_3, &pcnt3_count);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("Current counter 0 value :%d\n", pcnt0_count);
        printf("Current counter 1 value :%d\n", pcnt1_count);
        printf("Current counter 2 value :%d\n", pcnt2_count);
        printf("Current counter 3 value :%d\n", pcnt3_count);*/


        }
        
    }
        //driver1.set_speed(motor_speed);
        //vTaskDelay(2000/portTICK_PERIOD_MS);

        
}