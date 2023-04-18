#pragma once

  xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events

    /* A sample structure to pass events from the PCNT
     * interrupt handler to the main program.
    */
    typedef struct {
        int unit;  // the PCNT unit that originated an interrupt
        uint32_t status; // information on the event type that caused the interrupt
    } pcnt_evt_t;
    /* Decode what PCNT's unit originated an interrupt
    * and pass this information together with the event type
    * the main program using a queue.
    */
static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
    {   
    uint32_t intr_status = PCNT.int_st.val;
    int i;
    pcnt_evt_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;

    for (i = 0; i < PCNT_UNIT_MAX; i++) {
        if (intr_status & (BIT(i))) {
            evt.unit = i;
            /* Save the PCNT event type that caused an interrupt
               to pass it to the main program */
            evt.status = PCNT.status_unit[i].val;
            PCNT.int_clr.val = BIT(i);
            xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE) {
                portYIELD_FROM_ISR();
                }
            }
        }
    }

/* Initialize PCNT event queue and PCNT functions */
void PCNT_event_init(){

};

void IndexStepCounter_init(const pcnt_unit_t unit, const gpio_num_t pinA, const gpio_num_t pinB){

    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = pinA,
        .ctrl_gpio_num = pinB,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_REVERSE,
        .pos_mode = PCNT_COUNT_DIS,
        .neg_mode = PCNT_COUNT_DEC,
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
        .unit = unit,
        .channel = PCNT_CHANNEL_0,
        };
    

    pcnt_unit_config(&pcnt_config);
//---------------------------------------------------------------------------------------
    //ISR
    pcnt_event_enable(unit, PCNT_EVT_H_LIM); 
   
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);

    pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, NULL);
    pcnt_intr_enable(unit);

//---------------------------------------------------------------------------------------
    
    pcnt_counter_resume(unit);


    }
