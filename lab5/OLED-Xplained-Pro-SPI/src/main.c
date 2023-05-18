#include <asf.h>
#include "conf_board.h"
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#define OLED_WIDTH 128
#define OLED_HEIGHT 32

#define USART_COM_ID ID_USART1
#define USART_COM USART1
static void USART1_init(void);

#define BUT1_PIO PIOD
#define BUT1_PIO_ID ID_PIOD
#define BUT1_PIO_IDX 28
#define BUT1_PIO_IDX_MASK (1u<< BUT1_PIO_IDX)

#define TRIG_PIO PIOD
#define TRIG_PIO_ID ID_PIOD
#define TRIG_PIO_IDX 30
#define TRIG_PIO_ID_MASK (1u<< TRIG_PIO_IDX)

#define ECHO_PIO PIOA
#define ECHO_PIO_ID ID_PIOA
#define ECHO_PIO_IDX 6
#define ECHO_PIO_ID_MASK (1u<< ECHO_PIO_IDX)



volatile int but_flag; // (1)
volatile int echo_flag; // (1)
volatile int flag_rtt;




int tempo = 0;
char escreve[20];




void but_callback (void) {
	but_flag = 1;
}
void echo_callback(void){
	if (echo_flag == 0){
		echo_flag = 1;
		rtt_init(RTT,1);

	}
	else{
		echo_flag = 0;
	}
}



void init(void)
{
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;

	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK,  PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
	pio_handler_set(BUT1_PIO,BUT1_PIO_ID,BUT1_PIO_IDX_MASK,	PIO_IT_FALL_EDGE,but_callback);
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4);
	
	pmc_enable_periph_clk(TRIG_PIO_ID);
	pio_configure(TRIG_PIO, PIO_OUTPUT_0, TRIG_PIO_ID_MASK, PIO_DEFAULT);
	
	pmc_enable_periph_clk(ECHO_PIO_ID);
	pio_configure(ECHO_PIO, PIO_INPUT, ECHO_PIO_ID_MASK, PIO_DEFAULT);
	pio_handler_set(ECHO_PIO,ECHO_PIO_ID,ECHO_PIO_ID_MASK, PIO_IT_EDGE, echo_callback);
	pio_enable_interrupt(ECHO_PIO, ECHO_PIO_ID_MASK);
	pio_get_interrupt_status(ECHO_PIO);
	NVIC_EnableIRQ(ECHO_PIO_ID);
	NVIC_SetPriority(ECHO_PIO_ID, 4);



	
}




int main (void)
{
	sysclk_init();
	board_init();

	init();
  // Init OLED
	gfx_mono_ssd1306_init();

  

	flag_rtt = 1;
	int contagem;
	int freq = 32768;
	int vetor[]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	int i = 0;
  /* Insert application code here, after the board has been initialized. */
	while(1) {
		delay_ms(50);

		
		//triggers the sensor
		pio_set(TRIG_PIO,TRIG_PIO_ID_MASK);
		delay_us(10);
		pio_clear(TRIG_PIO,TRIG_PIO_ID_MASK);

		//waits for the echo
		while(echo_flag == 0);
		rtt_init(RTT,1);

		while(echo_flag == 1);

		contagem = rtt_read_timer_value(RTT);
		tempo = (contagem*1000000)/freq;
		
		//get distance in cm
		tempo = tempo * 340 / 2 / 10000;
		gfx_mono_draw_filled_rect(0,0,OLED_WIDTH,OLED_HEIGHT,GFX_PIXEL_CLR);
		if (tempo < 1 || tempo > 400)
		{
			sprintf(escreve,"Invalido",tempo);

		}
		else{
			sprintf(escreve,"%d cm",tempo);
			//de 60 a 80 em x e 20 a 0 em y
			// 2 --- > 30 400 --> 0
			vetor[i] = tempo;
			i++;
			if (i>15){
				i=0;
			}
			int g = 0;
			int j =0;
			int razao ;
			g= i;
			while (j< 16)
			{
				
				
				if (g> 15){
					g =0;
				}
				if (vetor[g] <12 ){
					razao = 30;
				}
				else{
					razao = -(vetor[g]/12) + 30;
				}
				if (razao<0 ){
					razao = 0;
				}
				else if (razao> 30){
					razao =0;
				}
				gfx_mono_draw_filled_rect((60+j*3),razao, 2 ,2,GFX_PIXEL_SET);
				j++;
				g++;
			}
			

		}
		
		gfx_mono_draw_string(escreve,0,0,&sysfont);
		
		
		

		delay_ms(1000);
				
	}
	
	
}
