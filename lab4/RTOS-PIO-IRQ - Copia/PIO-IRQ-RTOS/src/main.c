#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#define LED1_PIO   PIOA
#define LED1_PIO_ID ID_PIOA
#define LED1_PIO_IDX     0
#define LED1_PIO_IDX_MASK  (1<<LED1_PIO_IDX)

#define LED2_PIO   PIOC
#define LED2_PIO_ID ID_PIOC
#define LED2_PIO_IDX     30
#define LED2_PIO_IDX_MASK  (1<<LED2_PIO_IDX)

#define LED3_PIO   PIOB
#define LED3_PIO_ID ID_PIOB
#define LED3_PIO_IDX     2
#define LED3_PIO_IDX_MASK  (1<<LED3_PIO_IDX)


#define BUT1_PIO PIOD
#define BUT1_PIO_ID ID_PIOD
#define BUT1_PIO_IDX 28
#define BUT1_PIO_IDX_MASK (1u<< BUT1_PIO_IDX)

#define BUT2_PIO PIOC
#define BUT2_PIO_ID ID_PIOC
#define BUT2_PIO_IDX 31
#define BUT2_PIO_IDX_MASK (1u<< BUT2_PIO_IDX)

#define BUT3_PIO PIOA
#define BUT3_PIO_ID ID_PIOA
#define BUT3_PIO_IDX 19
#define BUT3_PIO_IDX_MASK (1u<< BUT3_PIO_IDX)

#define TASK_LED_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_BUT_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_BUT_STACK_PRIORITY (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);


extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName) {
  printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
  /* If the parameters have been corrupted then inspect pxCurrentTCB to
   * identify which task has overflowed its stack.
   */
  for (;;) {
  }
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void) { pmc_sleep(SAM_PM_SMODE_SLEEP_WFI); }

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void) {
  /* Called if a call to pvPortMalloc() fails because there is insufficient
  free memory available in the FreeRTOS heap.  pvPortMalloc() is called
  internally by FreeRTOS API functions that create tasks, queues, software
  timers, and semaphores.  The size of the FreeRTOS heap is set by the
  configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

  /* Force an assert. */
  configASSERT((volatile void *)NULL);
}

QueueHandle_t xQueueSenha;

volatile char but_flag; // (1)
volatile char but2_flag; // (1)
volatile char but3_flag; // (1)
void but_callback (void) {
	int digito = 1;
	xQueueSendFromISR(xQueueSenha,(void*)&digito,10);


}
void but2_callback(void){
	int digito = 2;
	xQueueSendFromISR(xQueueSenha,(void*)&digito,10);


}
void but3_callback(void){
	int digito = 2;
	xQueueSendFromISR(xQueueSenha,(void*)&digito,10);
}


void init(void)
{
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	pmc_enable_periph_clk(LED1_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(LED3_PIO_ID);
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);

	pmc_enable_periph_clk(BUT3_PIO_ID);

	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK,0,0,0);
	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK,0,0,0);
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK,0,0,0);
	
	
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_PIO_IDX_MASK, PIO_DEFAULT);

	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK,  PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
	pio_handler_set(BUT1_PIO,BUT1_PIO_ID,BUT1_PIO_IDX_MASK,	PIO_IT_FALL_EDGE,but_callback);
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4);
	
	
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK,  PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, 60);
	pio_handler_set(BUT2_PIO,BUT2_PIO_ID,BUT2_PIO_IDX_MASK,	PIO_IT_FALL_EDGE,but2_callback);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT2_PIO);
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 4);
	
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK,  PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 60);
	pio_handler_set(BUT3_PIO,BUT3_PIO_ID,BUT3_PIO_IDX_MASK,	PIO_IT_FALL_EDGE,but3_callback);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT3_PIO);
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 4);
	
}
static void task_but(void *pvParameters) {
	int senha[4] = {1,1,2,3};

	uint32_t msg = 0;
	uint32_t delayTicks = 2000;
	int tentativa[4];
	int digito;
	int i =0;
	int errou =0;
	gfx_mono_draw_string("oi2i", 50,16, &sysfont);

	for (;;) {
			gfx_mono_draw_string("oi2i", 50,16, &sysfont);

		if (xQueueReceive(xQueueSenha, &digito, 0)){
			tentativa[i] = digito;
			i++;
			if (i>3){
				for (int j =0 ; j<4; j++){
					if (tentativa[j]!= senha[j]){
						errou = 1;
					}
				}
				if (errou ==1){
					gfx_mono_draw_string("trouxa", 50,16, &sysfont);

				}
				i==0;
			}
		}
		
			
		

	}
}
int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	init();
		gfx_mono_ssd1306_init();


  // Init OLED
	
   xQueueSenha = xQueueCreate(32, sizeof(uint32_t));
	gfx_mono_draw_string("oii", 50,16, &sysfont);

 if (xTaskCreate(task_but, "BUT", TASK_BUT_STACK_SIZE, NULL,TASK_BUT_STACK_PRIORITY, NULL) != pdPASS) {
					  gfx_mono_draw_string("naos", 50,16, &sysfont);
	}
	else{
							  gfx_mono_draw_string("foii", 50,16, &sysfont);

	}

  /* Start the scheduler. */
  vTaskStartScheduler();



  /* Insert application code here, after the board has been initialized. */
	while(1) {

			
			
	}
}
