/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include <asf.h>
#include <string.h>
#include "ili9341.h"
#include "lvgl.h"
#include "touch/touch.h"
LV_FONT_DECLARE(dseg30);
LV_FONT_DECLARE(dseg15);
LV_FONT_DECLARE(dseg70);
LV_FONT_DECLARE(dseg40);
LV_FONT_DECLARE(haha);
lv_obj_t * labelBtn1;
lv_obj_t * labelBtn2;
lv_obj_t * labelBtn3;
lv_obj_t * labelBtn4;
lv_obj_t * labelBtn5;
lv_obj_t * labelFloor;
lv_obj_t * labelFloor2;

lv_obj_t * labelTemp;
lv_obj_t * labelTime;

lv_obj_t * btn1;
lv_obj_t * btn2;
lv_obj_t * btn3;
lv_obj_t * btn4;
lv_obj_t * btn5;
#define SYMBOL "\xEF\x80\x97"

typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t seccond;
} calendar;

calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};

/************************************************************************/
/* LCD / LVGL                                                           */
/************************************************************************/

#define LV_HOR_RES_MAX          (320)
#define LV_VER_RES_MAX          (240)

/*A static or global variable to store the buffers*/
static lv_disp_draw_buf_t disp_buf;

/*Static or global buffer(s). The second buffer is optional*/
static lv_color_t buf_1[LV_HOR_RES_MAX * LV_VER_RES_MAX];
static lv_disp_drv_t disp_drv;          /*A variable to hold the drivers. Must be static or global.*/
static lv_indev_drv_t indev_drv;

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_LCD_STACK_SIZE                (1024*12/sizeof(portSTACK_TYPE))
#define TASK_ALT_STACK_SIZE                (1024*2/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}
SemaphoreHandle_t xSemaphoretime;
/************************************************************************/
/* lvgl                                                                 */
/************************************************************************/
SemaphoreHandle_t xMutexLVGL ;
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.seccond);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}


void RTC_Handler(void) {
	uint32_t ul_status = rtc_get_status(RTC);
	
	/* seccond tick */
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphoretime, &xHigherPriorityTaskWoken);
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		// o c�digo para irq de alame vem aqui
		
	}

	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}
static void desl_handler(lv_event_t * e);
static void event_handler(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_CLICKED) {
		LV_LOG_USER("Clicked");
		static lv_style_t style;
				lv_obj_clean(lv_scr_act());

		
		
		btn1 = lv_btn_create(lv_scr_act());
		lv_obj_add_event_cb(btn1, desl_handler, LV_EVENT_ALL, NULL);
		lv_obj_align(btn1, LV_ALIGN_BOTTOM_LEFT, 0,0);
		lv_obj_add_style(btn1, &style, 0);

		labelBtn1 = lv_label_create(btn1);
		lv_label_set_text(labelBtn1, "[ " LV_SYMBOL_POWER);
		lv_obj_center(labelBtn1);
		lv_obj_set_width(btn1, 60);
		lv_obj_set_height(btn1, 60);
		
	}

	
		
	else if(code == LV_EVENT_VALUE_CHANGED) {
		LV_LOG_USER("Toggled");
	}
}

static void menu_handler(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_CLICKED) {
		LV_LOG_USER("Clicked");
	}
	else if(code == LV_EVENT_VALUE_CHANGED) {
		LV_LOG_USER("Toggled");
	}
}

static void Settings_handler(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_CLICKED) {
		LV_LOG_USER("Clicked");
	}
	else if(code == LV_EVENT_VALUE_CHANGED) {
		LV_LOG_USER("Toggled");
	}
}

static void up_handler(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);
	char *c;
	int temp;

	if(code == LV_EVENT_CLICKED) {
		c = lv_label_get_text(labelTemp);
		temp = atoi(c);
		lv_label_set_text_fmt(labelTemp, "%02d", temp + 1);
	}
}

static void down_handler(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);
	char *c;
	int temp;
	if(code == LV_EVENT_CLICKED) {
		c = lv_label_get_text(labelTemp);
		temp = atoi(c);
		lv_label_set_text_fmt(labelTemp, "%02d", temp - 1);
	}
}




void lv_termostato(void) {
		static lv_style_t style;
		lv_style_init(&style);
		lv_style_set_bg_color(&style, lv_color_black());
		lv_style_set_border_color(&style, lv_color_black());
		lv_style_set_border_width(&style, 5);
	
		
	    btn1 = lv_btn_create(lv_scr_act());
	    lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
		lv_obj_align(btn1, LV_ALIGN_BOTTOM_LEFT, 0,0);
		lv_obj_add_style(btn1, &style, 0);

	    labelBtn1 = lv_label_create(btn1);
	    lv_label_set_text(labelBtn1, "[ " LV_SYMBOL_POWER);
	    lv_obj_center(labelBtn1);
		lv_obj_set_width(btn1, 60);
		lv_obj_set_height(btn1, 60);
		
		labelFloor = lv_label_create(lv_scr_act());
		lv_obj_align(labelFloor, LV_ALIGN_LEFT_MID, 35 , -15);
		lv_obj_set_style_text_font(labelFloor, &dseg70, LV_STATE_DEFAULT);
		lv_obj_set_style_text_color(labelFloor, lv_color_white(), LV_STATE_DEFAULT);
		lv_label_set_text_fmt(labelFloor, "%02d", 23);
		
		labelFloor2 = lv_label_create(lv_scr_act());
		lv_obj_align_to(labelFloor2, labelFloor, LV_ALIGN_OUT_RIGHT_MID, 0,0);
		lv_obj_set_style_text_font(labelFloor2, &dseg40, LV_STATE_DEFAULT);
		lv_obj_set_style_text_color(labelFloor2, lv_color_white(), LV_STATE_DEFAULT);
		lv_label_set_text_fmt(labelFloor2, ".%01d", 4);
		
		
		labelTemp = lv_label_create(lv_scr_act());
		lv_obj_align(labelTemp, LV_ALIGN_RIGHT_MID, -20 , -15);
		lv_obj_set_style_text_font(labelTemp, &dseg40, LV_STATE_DEFAULT);
		lv_obj_set_style_text_color(labelTemp, lv_color_white(), LV_STATE_DEFAULT);
		lv_label_set_text_fmt(labelTemp, "%02d", 21);
		
		labelTime = lv_label_create(lv_scr_act());
		lv_obj_align(labelTime, LV_ALIGN_RIGHT_MID, -20 , -90);
		lv_obj_set_style_text_font(labelTime, &dseg30, LV_STATE_DEFAULT);
		lv_obj_set_style_text_color(labelTime, lv_color_white(), LV_STATE_DEFAULT);
		lv_label_set_text_fmt(labelTime, "%02d", 13);
		
	
}
void btnMenu(void) {
	static lv_style_t style;
	lv_style_init(&style);
	lv_style_set_bg_color(&style, lv_color_black());
	lv_style_set_border_color(&style, lv_color_black());
	lv_style_set_border_width(&style, 5);
	
	
	btn2 = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btn2, menu_handler, LV_EVENT_ALL, NULL);
	lv_obj_set_width(btn2, 60);
	lv_obj_set_height(btn2, 60);
	lv_obj_align_to(btn2, btn1, LV_ALIGN_OUT_RIGHT_TOP, 0,0);
	lv_obj_add_style(btn2, &style, 0);

	labelBtn2 = lv_label_create(btn2);
	lv_label_set_text(labelBtn2, "| M |");
	lv_obj_center(labelBtn2);
	
	
}
void Settings(void) {
	static lv_style_t style;
	lv_style_init(&style);
	lv_style_set_bg_color(&style, lv_color_black());
	lv_style_set_border_color(&style, lv_color_black());
	lv_style_set_border_width(&style, 5);
	
	
	btn3 = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btn3, Settings_handler, LV_EVENT_ALL, NULL);
	lv_obj_set_width(btn3, 60);
	lv_obj_set_height(btn3, 60);
	lv_obj_align_to(btn3, btn2, LV_ALIGN_OUT_RIGHT_MID, 0,0);
	lv_obj_add_style(btn3, &style, 0);

	labelBtn3 = lv_label_create(btn3);
	lv_obj_set_style_text_font(labelBtn3, &haha, LV_STATE_DEFAULT);

	lv_label_set_text(labelBtn3, SYMBOL);
	lv_obj_center(labelBtn3);
	labelBtn3 = lv_label_create(btn3);
	lv_obj_set_style_text_font(labelBtn3, &haha, LV_STATE_DEFAULT);

	lv_label_set_text(labelBtn3, SYMBOL);
	lv_obj_center(labelBtn3);

	
}
void btnUp(void) {
	static lv_style_t style;
	lv_style_init(&style);
	lv_style_set_bg_color(&style, lv_color_black());
	lv_style_set_border_color(&style, lv_color_black());
	lv_style_set_border_width(&style, 5);
	
	
	btn4 = lv_btn_create(lv_scr_act());
	lv_obj_set_width(btn4, 60);
	lv_obj_set_height(btn4, 60);
	lv_obj_add_event_cb(btn4, up_handler, LV_EVENT_ALL, NULL);
	lv_obj_align_to(btn4, btn5, LV_ALIGN_OUT_LEFT_TOP, 0,0);
	lv_obj_add_style(btn4, &style, 0);

	labelBtn4 = lv_label_create(btn4);
	lv_label_set_text(labelBtn4, "[ " LV_SYMBOL_UP);
	lv_obj_center(labelBtn4);
	
	
}
void btnDown(void) {
	static lv_style_t style;
	lv_style_init(&style);
	lv_style_set_bg_color(&style, lv_color_black());
	lv_style_set_border_color(&style, lv_color_black());
	lv_style_set_border_width(&style, 5);
	
	
	btn5 = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btn5, down_handler, LV_EVENT_ALL, NULL);
	lv_obj_align(btn5, LV_ALIGN_BOTTOM_RIGHT, 0,0);
	lv_obj_add_style(btn5, &style, 0);

	labelBtn5 = lv_label_create(btn5);
	lv_label_set_text(labelBtn5, LV_SYMBOL_DOWN " ] ");
	lv_obj_center(labelBtn5);
	lv_obj_set_width(btn5, 60);
	lv_obj_set_height(btn5, 60);
	
}
static void desl_handler(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_CLICKED) {
		lv_obj_clean(lv_scr_act());
		LV_LOG_USER("Clicked");
		int px, py;
		lv_termostato();
		btnMenu();
		Settings();
		btnDown();
		btnUp();
		
	}
	else if(code == LV_EVENT_VALUE_CHANGED) {
		LV_LOG_USER("Toggled");
	}
}
/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_lcd(void *pvParameters) {
	int px, py;

			lv_termostato();
			btnMenu();
			Settings();
			btnDown();
			btnUp();

	for (;;)  {
		xSemaphoreTake( xMutexLVGL , portMAX_DELAY );

		lv_tick_inc(50);
		lv_task_handler();
		 xSemaphoreGive( xMutexLVGL  );

		vTaskDelay(50);
	}
}
static void task_rtc(void *pvParameters) {

	        /** Configura RTC */
	        RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN);
	        
	        /* Leitura do valor atual do RTC */
	        uint32_t current_hour, current_min, current_sec;
	        uint32_t current_year, current_month, current_day, current_week;
	        rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
	        rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
	        
	        /* configura alarme do RTC para daqui 20 segundos */
	        rtc_set_date_alarm(RTC, 1, current_month, 1, current_day);
	        rtc_set_time_alarm(RTC, 1, current_hour, 1, current_min, 1, current_sec + 1);
	        
	        while (1) {
				if (xSemaphoreTake(xSemaphoretime,0)){
					rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
					 rtc_set_date_alarm(RTC, 1, current_month, 1, current_day);
					 rtc_set_time_alarm(RTC, 1, current_hour, 1, current_min, 1, current_sec + 1);
					lv_label_set_text_fmt(labelTime, "%02d:%02d:%02d", current_hour, current_min, current_sec);
					if (current_sec == 59){
						rtc_set_date_alarm(RTC, 1, current_month, 1, current_day);
						rtc_set_time_alarm(RTC, 1, current_hour, 1, current_min+1, 1, 0);
					}

					}
				}
			}
			
			

/************************************************************************/
/* configs                                                              */
/************************************************************************/

static void configure_lcd(void) {
	/**LCD pin configure on SPI*/
	pio_configure_pin(LCD_SPI_MISO_PIO, LCD_SPI_MISO_FLAGS);  //
	pio_configure_pin(LCD_SPI_MOSI_PIO, LCD_SPI_MOSI_FLAGS);
	pio_configure_pin(LCD_SPI_SPCK_PIO, LCD_SPI_SPCK_FLAGS);
	pio_configure_pin(LCD_SPI_NPCS_PIO, LCD_SPI_NPCS_FLAGS);
	pio_configure_pin(LCD_SPI_RESET_PIO, LCD_SPI_RESET_FLAGS);
	pio_configure_pin(LCD_SPI_CDS_PIO, LCD_SPI_CDS_FLAGS);
	
	ili9341_init();
	ili9341_backlight_on();
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT,
	};

	/* Configure console UART. */
	stdio_serial_init(CONSOLE_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

/************************************************************************/
/* port lvgl                                                            */
/************************************************************************/

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
	ili9341_set_top_left_limit(area->x1, area->y1);   ili9341_set_bottom_right_limit(area->x2, area->y2);
	ili9341_copy_pixels_to_screen(color_p,  (area->x2 + 1 - area->x1) * (area->y2 + 1 - area->y1));
	
	/* IMPORTANT!!!
	* Inform the graphics library that you are ready with the flushing*/
	lv_disp_flush_ready(disp_drv);
}

void my_input_read(lv_indev_drv_t * drv, lv_indev_data_t*data) {
	int px, py, pressed;
	
	if (readPoint(&px, &py))
		data->state = LV_INDEV_STATE_PRESSED;
	else
		data->state = LV_INDEV_STATE_RELEASED; 
	
	data->point.x = px;
	data->point.y = py;
}

void configure_lvgl(void) {
	lv_init();
	lv_disp_draw_buf_init(&disp_buf, buf_1, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX);
	
	lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
	disp_drv.draw_buf = &disp_buf;          /*Set an initialized buffer*/
	disp_drv.flush_cb = my_flush_cb;        /*Set a flush callback to draw to the display*/
	disp_drv.hor_res = LV_HOR_RES_MAX;      /*Set the horizontal resolution in pixels*/
	disp_drv.ver_res = LV_VER_RES_MAX;      /*Set the vertical resolution in pixels*/

	lv_disp_t * disp;
	disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/
	
	/* Init input on LVGL */
	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = my_input_read;
	lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv);
}



/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	/* board and sys init */
	board_init();
	sysclk_init();
	configure_console();

	/* LCd, touch and lvgl init*/
	configure_lcd();
	configure_touch();
	configure_lvgl();
	xSemaphoretime = xSemaphoreCreateBinary();
	xMutexLVGL = xSemaphoreCreateMutex();
	if (xSemaphoretime == NULL)
	printf("falha em criar o semaforo \n");
	
	/* Create task to control oled */
	if (xTaskCreate(task_lcd, "LCD", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create lcd task\r\n");
	}

	
	if (xTaskCreate(task_rtc, "rtc", TASK_ALT_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create rtc task\r\n");
	}
	/* Create task to control oled */
	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){ }
}
