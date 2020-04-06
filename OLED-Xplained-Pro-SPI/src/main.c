#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#define LED1_PIO			PIOA
#define LED1_PIO_ID			ID_PIOA
#define LED1_PIO_IDX		0
#define LED1_PIO_IDX_MASK	(1 << LED1_PIO_IDX)

#define LED2_PIO			PIOC
#define LED2_PIO_ID			ID_PIOC
#define LED2_PIO_IDX		30
#define LED2_PIO_IDX_MASK	(1 << LED2_PIO_IDX)

#define LED3_PIO			PIOB
#define LED3_PIO_ID			ID_PIOB
#define LED3_PIO_IDX		2
#define LED3_PIO_IDX_MASK	(1 << LED3_PIO_IDX)

#define LED_PIO			     PIOC
#define LED_PIO_ID			ID_PIOC
#define LED_PIO_IDX		8
#define LED_PIO_IDX_MASK	(1 << LED_PIO_IDX)


// Configuracoes do botoes

#define BUT_PIO				PIOA
#define BUT_PIO_ID			ID_PIOA
#define BUT_PIO_IDX			11
#define BUT_PIO_IDX_MASK (1u << BUT_PIO_IDX)

#define BUT1_PIO			PIOD
#define BUT1_PIO_ID			ID_PIOD
#define BUT1_PIO_IDX		28
#define BUT1_PIO_IDX_MASK	(1u << BUT1_PIO_IDX)

#define BUT3_PIO			PIOA
#define BUT3_PIO_ID			ID_PIOA
#define BUT3_PIO_IDX		19
#define BUT3_PIO_IDX_MASK	(1u << BUT3_PIO_IDX)

#define BUT2_PIO			PIOC
#define BUT2_PIO_ID			ID_PIOC
#define BUT2_PIO_IDX		31
#define BUT2_PIO_IDX_MASK	(1u << BUT2_PIO_IDX)


void init(void);
void but_callback(void);
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);

volatile char but_flag = 0;
volatile char but1_flag = 0;
volatile char but2_flag = 0;
volatile char but3_flag = 0;

volatile char flag_led = 0;
volatile char flag_led1 = 0;
volatile char flag_led2 = 0;
volatile char flag_led3 = 0;

volatile Bool f_rtt_alarme = false;

int freq1 = 5;
int freq2 = 10;
int freq3 = 1;

void but_callback(void){
	but_flag = !but_flag;
};
void but1_callback(void){
	but1_flag = !but1_flag;
};
void but2_callback(void){
	but2_flag = !but2_flag;
};
void but3_callback(void){
	but3_flag = !but3_flag;
};

void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void init(void){
	// Initialize the board clock
	sysclk_init();

	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	// Ativa o PIO na qual o LED foi conectado
	// para que possamos controlar o LED.
	pmc_enable_periph_clk(LED_PIO_ID);
	pmc_enable_periph_clk(LED1_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(LED3_PIO_ID);
	
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED_PIO , PIO_OUTPUT_0, LED_PIO_IDX_MASK, PIO_DEFAULT);
	
	
	// Inicializa PIO do botao
	pmc_enable_periph_clk(BUT_PIO_ID);
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);
	
	pio_configure(BUT_PIO , PIO_INPUT, BUT_PIO_IDX_MASK, PIO_PULLUP);
	pio_configure(BUT1_PIO, PIO_INPUT, BUT_PIO_IDX_MASK, PIO_PULLUP);
	pio_configure(BUT2_PIO, PIO_INPUT, BUT_PIO_IDX_MASK, PIO_PULLUP);
	pio_configure(BUT3_PIO, PIO_INPUT, BUT_PIO_IDX_MASK, PIO_PULLUP);
	
	
	pio_handler_set(BUT_PIO , BUT_PIO_ID, BUT_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but_callback);
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_RISE_EDGE, but1_callback);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but2_callback);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_RISE_EDGE, but3_callback);
	
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);
	
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4);
	
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 4);
	
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 4);
	
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses){
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	/* Configura o PMC */
	/* O TimerCounter é meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_led1 = 1;
}

void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_led2 = 1;
}

void TC2_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 2);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_led3 = 1;
}

void RTT_Handler(void){
	uint32_t ul_status;

	/* Get RTT status */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {  }

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		pin_toggle(LED1_PIO, LED1_PIO_IDX_MASK);
		pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK);
		pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);    // BLINK Led
		f_rtt_alarme = true;                  // flag RTT alarme
	}
}

void pisca_led1(){
	pin_toggle(LED1_PIO, LED1_PIO_IDX_MASK);
}

void pisca_led2(){
	pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK);
}

void pisca_led3(){
	pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);
}

int main (void){
	
	board_init();
	sysclk_init();
	delay_init();
	init();


	TC_init(TC0, ID_TC0, 0, freq1);
	TC_init(TC0, ID_TC1, 1, freq2);
	TC_init(TC0, ID_TC2, 2, freq3);

  // Init OLED
	gfx_mono_ssd1306_init();
  
  // Escreve na tela um circulo e um texto
	char string1[10]; 
	char string2[10]; 
	char string3[10];
	char stringtotal[30]; 
	itoa(freq1,string1,10);
	itoa(freq2,string2,10);
	itoa(freq3,string3,10); 
	snprintf(stringtotal, sizeof stringtotal, "%s%s%s%s%s", string1," ", string2," ", string3);
// 	gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
    gfx_mono_draw_string(stringtotal, 50,16, &sysfont);


	pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
	pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
	pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
  /* Insert application code here, after the board has been initialized. */
	while(1) {
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		
		if(but1_flag){
			if(flag_led1){
				pisca_led1();
				flag_led1 = 0;
			}
		}
		
		if(but2_flag){
			if(flag_led2){
				pisca_led2();
				flag_led2 = 0;
			}
		}
		if(but3_flag){
			if(flag_led3){
				pisca_led3();
				flag_led3 = 0;
				}
		}
		
		if (f_rtt_alarme){
			
			uint16_t pllPreScale = (int) (((float) 32768) / 2.0);
			uint32_t irqRTTvalue  = 5;
			
			RTT_init(pllPreScale, irqRTTvalue);
			
			f_rtt_alarme = false;
		}
	}
}
