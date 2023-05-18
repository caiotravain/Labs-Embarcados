#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} calendar;
volatile flag_led0;
#define LED_PIO_ID ID_PIOC
#define LED_PIO PIOC
#define LED_PIN 8
#define LED_PIN_MASK (1 << LED_PIN)

#define LED4_PIO   PIOA
#define LED4_PIO_ID ID_PIOA
#define LED4_PIO_IDX     0
#define LED4_PIO_IDX_MASK  (1<<LED4_PIO_IDX)

#define LED2_PIO   PIOC
#define LED2_PIO_ID ID_PIOC
#define LED2_PIO_IDX     30
#define LED2_PIO_IDX_MASK  (1<<LED2_PIO_IDX)

#define LED3_PIO   PIOB
#define LED3_PIO_ID ID_PIOB
#define LED3_PIO_IDX     2
#define LED3_PIO_IDX_MASK  (1<<LED3_PIO_IDX)

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
static void BUT_init(void);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);
void pin_toggle(Pio *pio, uint32_t mask);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/
SemaphoreHandle_t xSemaphoreLED3;
SemaphoreHandle_t xSemaphoreLED2;
SemaphoreHandle_t xSemaphoreclock;
SemaphoreHandle_t xSemaphorepisca;
void but_callback(void) {
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(xSemaphoreLED3, &xHigherPriorityTaskWoken);
}


void TC1_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 1);

	/** Muda o estado do LED (pisca) **/
	pin_toggle(LED_PIO, LED_PIN_MASK);  
}
void TC4_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC1, 1);

	/** Muda o estado do LED (pisca) **/

	pin_toggle(LED4_PIO, LED4_PIO_IDX_MASK);  
}
void TC7_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC2, 1);

	/** Muda o estado do LED (pisca) **/

	pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);  
}

void RTC_Handler(void){
	uint32_t ul_status = rtc_get_status(RTC);

	/* Sec IRQ */
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC){
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphoreclock, &xHigherPriorityTaskWoken);
		
	}

	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM){
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphorepisca, &xHigherPriorityTaskWoken);

		

	}
	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}
void LED_init(int estado){
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_PIN_MASK, estado, 0, 0);
		pmc_enable_periph_clk(LED4_PIO_ID);
		pmc_enable_periph_clk(LED2_PIO_ID);
		pmc_enable_periph_clk(LED3_PIO_ID);


		pio_set_output(LED4_PIO, LED4_PIO_IDX_MASK,estado,0,0);
		pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK,estado,0,0);
		pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK,estado,0,0);
		
	  pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK,  PIO_PULLUP);
	  pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	  pio_handler_set(BUT_PIO,BUT_PIO_ID,BUT_PIO_PIN_MASK,	PIO_IT_FALL_EDGE,but_callback);
	  pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	  pio_get_interrupt_status(BUT_PIO);
	  NVIC_EnableIRQ(BUT_PIO_ID);
	  NVIC_SetPriority(BUT_PIO_ID, 4);
};
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc, irq_type);
}
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}
void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphoreLED2, &xHigherPriorityTaskWoken);

	}
}
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
uint32_t ul_div;
uint32_t ul_tcclks;
uint32_t ul_sysclk = sysclk_get_cpu_hz();

/* Configura o PMC */
pmc_enable_periph_clk(ID_TC);

/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);

/** ATIVA PMC PCK6 TIMER_CLOCK1  */
if(ul_tcclks == 0 )
pmc_enable_pck(PMC_PCK_6);

tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

/* Configura NVIC*/
NVIC_SetPriority(ID_TC, 4);
NVIC_EnableIRQ((IRQn_Type) ID_TC);
tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}
void pin_toggle(Pio *pio, uint32_t mask){
	if (pio_get_output_data_status(pio, mask))
		pio_clear(pio, mask);
	else
		pio_set(pio, mask);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();

	calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);
	TC_init(TC0, ID_TC1, 1, 4);
	tc_start(TC0, 1);
	
	TC_init(TC1, ID_TC4, 1, 5);
	tc_start(TC1, 1);
	
	    /* Leitura do valor atual do RTC */
	uint32_t current_hour, current_min, current_sec;
	uint32_t current_year, current_month, current_day, current_week;
	
  
	/* configura alarme do RTC para daqui 20 segundos */
	for (;;)  {
		
		if (xSemaphoreTake(xSemaphoreLED3,10)){
					rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
					rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
					rtc_set_date_alarm(RTC, 1, current_month, 1, current_day);
					rtc_set_time_alarm(RTC, 1, current_hour, 1, current_min, 1, current_sec + 10);
		}
		
	}
}
static void task_RTT(void *pvParameters) {
	gfx_mono_ssd1306_init();
	
	RTT_init(4, 16, RTT_MR_ALMIEN);

	/* Leitura do valor atual do RTC */
	uint32_t current_hour, current_min, current_sec;
	uint32_t current_year, current_month, current_day, current_week;
	
	
	/* configura alarme do RTC para daqui 20 segundos */
	for (;;)  {
		
		if (xSemaphoreTake(xSemaphoreLED2,10)){
				RTT_init(4, 16, RTT_MR_ALMIEN);
				pin_toggle(LED2_PIO,LED2_PIO_IDX_MASK);

		}
		
	}
}
static void task_clock(void *pvParameters) {
	gfx_mono_ssd1306_init();
	
	RTT_init(4, 16, RTT_MR_ALMIEN);

	/* Leitura do valor atual do RTC */
	uint32_t current_hour, current_min, current_sec;
	uint32_t current_year, current_month, current_day, current_week;
	
	char tudo [20];
	/* configura alarme do RTC para daqui 20 segundos */
	for (;;)  {
		
		if (xSemaphoreTake(xSemaphoreclock,10)){
			rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
			rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
			sprintf(tudo, "%d:%d:%d ", current_hour,current_min,current_sec);
			gfx_mono_draw_string(tudo, 10, 10, &sysfont);
		}
		
	}
}

static void task_pisca(void *pvParameters) {
	gfx_mono_ssd1306_init();
	
	RTT_init(4, 16, RTT_MR_ALMIEN);

	/* Leitura do valor atual do RTC */
	uint32_t current_hour, current_min, current_sec;
	uint32_t current_year, current_month, current_day, current_week;
	
	char tudo [20];
	/* configura alarme do RTC para daqui 20 segundos */
	for (;;)  {
		
		if (xSemaphoreTake(xSemaphorepisca,10)){
			TC_init(TC2, ID_TC7, 1, 3);
			tc_start(TC2, 1);
			
		}
		
	}
}
/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void BUT_init(void) {
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

	/* conf botão como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK,PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 10);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	LED_init(1);

	/* Initialize the console uart */
	configure_console();
	
	xSemaphoreLED3 = xSemaphoreCreateBinary();
	xSemaphoreLED2 = xSemaphoreCreateBinary();
	xSemaphoreclock = xSemaphoreCreateBinary();
	xSemaphorepisca = xSemaphoreCreateBinary();

	if (xSemaphoreLED3 == NULL)
	printf("falha em criar o semaforo \n");
	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}
	if (xTaskCreate(task_RTT, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled task\r\n");
	}
	if (xTaskCreate(task_clock, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled task\r\n");
	}

	if (xTaskCreate(task_pisca, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled task\r\n");
	}
	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
