#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"


#define LED1_PIO   PIOA
#define LED1_PIO_ID ID_PIOA
#define LED1_PIO_IDX     0
#define LED1_PIO_IDX_MASK  (1<<LED1_PIO_IDX)


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
volatile char but_flag; // (1)
volatile char but2_flag; // (1)
volatile char but3_flag; // (1)


int tempo = 0;
char escreve[20];


void pisca_led(int n, int t){
	int j = 70;
	for (int i=0;i<n;i++){
		
		if (but2_flag){
			but2_flag=0;
			break;
		}
		if (but3_flag){
			t += 100;
			tempo+=100;
			sprintf(escreve, "%f",(double)500/tempo);
			gfx_mono_draw_string(escreve, 50,16, &sysfont);
			but3_flag= 0;
		}
		for(int g=j; g<(j+10); g++){
			gfx_mono_draw_rect(g, 5, 2, 10, GFX_PIXEL_SET);

		}

		pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
		delay_ms(t);
		pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
		delay_ms(t);
		j+=10;
	}
}

void but_callback (void) {
	but_flag = 1;
}
void but2_callback(void){
	but2_flag = 1;
}
void but3_callback(void){
	but3_flag = 1;
}
void init(void)
{
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	pmc_enable_periph_clk(LED1_PIO_ID);
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




int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	init();

  // Init OLED
	gfx_mono_ssd1306_init();
  
  
	gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
	sprintf(escreve, "%f",(double)500/tempo ); //na funciona abaixo de 0
  	gfx_mono_draw_string(escreve, 50,16, &sysfont);

  

  /* Insert application code here, after the board has been initialized. */
	while(1) {
			
			

			// Escreve na tela um circulo e um texto
			
		for(int i=70;i<=120;i+=2){
				
				gfx_mono_draw_rect(i, 5, 2, 10, GFX_PIXEL_CLR);
				delay_ms(10);
			
		}
			
		if (but3_flag){
			tempo +=100;
			sprintf(escreve, "%f",(double)500/tempo);
			gfx_mono_draw_string(escreve, 50,16, &sysfont);//na funciona abaixo de 0
			pisca_led(5,tempo);
			but3_flag=0;
			
			
		}	
		if (but_flag) {  // (2)
			delay_ms(100);
			but2_flag =0;
			but3_flag = 0;
			if (pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK)) {
				if (tempo>100){
					tempo-=100;
				}
				
				} 
			else {
				tempo+=100;
			}
			
				
			sprintf(escreve, "%f",(double)500/tempo);
			gfx_mono_draw_string(escreve, 50,16, &sysfont);//na funciona abaixo de 0
			pisca_led(5,tempo);

			but_flag = 0;
		}

			
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI); // (1)

	}
}
