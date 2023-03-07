#pragma once

#define DRIVER_0_ADDRES           0  //podstava kladný doprava kladný k závoře
#define DRIVER_1_ADDRES           1  //klepeta kladný roztahuje kladný k závoře
#define DRIVER_2_ADDRES           2  //část s klepety (loket ruky) kladný dolů od optozávory
#define DRIVER_3_ADDRES           3  //cele rameno dolu kladny dolu kladny od zavor
#define DRIVER_0_ENABLE           GPIO_NUM_33   // H= disable motor output
#define DRIVER_1_ENABLE           GPIO_NUM_25   // H= disable motor output
#define DRIVER_2_ENABLE           GPIO_NUM_26   // H= disable motor output
#define DRIVER_3_ENABLE           GPIO_NUM_27   // H= disable motor output

#define opto3                     GPIO_NUM_34   // optozavora podstavy
#define opto0                     GPIO_NUM_35   // zdvih celeho ramene  
#define opto1                     GPIO_NUM_36   // stisk klepet
#define opto2                     GPIO_NUM_39   // rameno s klepety

/*#define VCC_IO_0                  GPIO_NUM_33  // L = reset driver 0, H = driver0 on
#define VCC_IO_1                  GPIO_NUM_25  // L = reset driver 0, H = driver0 on
#define VCC_IO_2                  GPIO_NUM_26  // L = reset driver 0, H = driver0 on
#define VCC_IO_3                  GPIO_NUM_27  // L = reset driver 0, H = driver0 on*/

#define SW_CTRL                   GPIO_NUM_32  // L = transistor Q3 off -> motor power off, H = all drivers on

#define DRIVERS_UART              UART_NUM_1
#define DRIVERS_UART_TXD          GPIO_NUM_17  // doma 27
#define DRIVERS_UART_RXD          GPIO_NUM_16  // doma 26
#define DRIVERS_UART_BUF_SIZE     256
#define DRIVERS_RX_TIMEOUT        (20 / portTICK_RATE_MS)
#define DRIVERS_UART_START_BYTE   0x05

#define GPIO_OUTPUT_PIN_SEL ((1ULL<<DRIVER_0_ENABLE) | (1ULL<<SW_CTRL) | (1ULL<<VCC_IO_0) )

#define GPIO_BIT_MASK  ((1ULL<<GPIO_NUM_35) | (1ULL<<GPIO_NUM_36) | (1ULL<<GPIO_NUM_39) | (1ULL<<GPIO_NUM_34))

#define MOTOR_SPEED_COEFICIENT    71608     // 71608 = 1RPS

#define ENCODER_H_LIM_VAL         1000
#define ENCODER_L_LIM_VAL        -1000


// globální proměnné pro pokusy s grafickým rozhraním
volatile int motor_speed0 = MOTOR_SPEED_COEFICIENT/3;      //svirani klepete
volatile int motor_speed1 = MOTOR_SPEED_COEFICIENT/3;      //rotace zakladny
volatile int motor_speed2 = MOTOR_SPEED_COEFICIENT/3;      //zdvih ramene s klepety
volatile int motor_speed3 = MOTOR_SPEED_COEFICIENT/3;      //zdvih celeho ramene
volatile int motor_speed = MOTOR_SPEED_COEFICIENT/3;       //testovaci motorspeed
volatile int motor_load = 0;
volatile int motor_stop_sensitivity = 100;
volatile int potenciometr = 0;
volatile int i_run = 8;
volatile int i_hold = 0;
volatile bool start_stop = false;
volatile uint mot_load[2048];
volatile uint mot_pos[2048];

volatile int motor_delay=5;

volatile bool conclusion;

int16_t pcnt0_count = 0;
int16_t pcnt1_count = 0;
int16_t pcnt2_count = 0;
int16_t pcnt3_count = 0;



#define PCNT_H_LIM_VAL            34
#define PCNT_L_LIM_VAL            0

volatile int FinalStep0=0; 
volatile int FinalStep1=0; 
volatile int FinalStep2=0; 
volatile int FinalStep3=0; 

            int motor0=0;
            int motor1=0;
            int motor2=0;
            int motor3=0;

volatile bool rucniRizeni_onRelease = false;
volatile bool zadavaniTrasy_onRelease = false;
volatile bool pridatBod_onRelease = false;
volatile bool spustitTrasu_onRelease = false;
volatile bool cyklovatTrasu_onRelease = false;
volatile bool synchronize_onRelease = false;
volatile bool reset_onRelease = false;

volatile bool klestePlus_onPress = false;
volatile bool klestePlus_onRelease = false;
volatile bool klesteMinus_onPress = false;
volatile bool klesteMinus_onRelease = false;
volatile bool loketPlus_onPress = false;
volatile bool loketPlus_onRelease = false;
volatile bool loketMinus_onPress = false;
volatile bool loketMinus_onRelease = false;
volatile bool ramenoPlus_onPress = false;
volatile bool ramenoPlus_onRelease = false;
volatile bool ramenoMinus_onPress = false;
volatile bool ramenoMinus_onRelease = false;
volatile bool podstavaPlus_onPress = false;
volatile bool podstavaPlus_onRelease = false;
volatile bool podstavaMinus_onPress = false;
volatile bool podstavaMinus_onRelease = false;
volatile bool Vlajka_onRelease = false;
volatile bool Dopravnik_onRelease = false;

std::vector<int> Vdriver0 = {};
std::vector<int> Vdriver1 = {};
std::vector<int> Vdriver2 = {};
std::vector<int> Vdriver3 = {};