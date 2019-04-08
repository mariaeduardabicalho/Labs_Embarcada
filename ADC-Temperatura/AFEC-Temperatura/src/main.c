/**
 * ADC
 * Rafael Corsi @ insper.edu.br
 * Abril 2017
 *
 * Configura o ADC do SAME70 para fazer leitura
 * do sensor de temperatura interno
 */

/************************************************************************/
/* Includes                                                             */
/************************************************************************/

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "asf.h"

/************************************************************************/
/* Defines                                                              */
/************************************************************************/

/** Header printf */
#define STRING_EOL    "\r"
#define STRING_HEADER "-- AFEC Temperature Sensor Example --\r\n" \
"-- "BOARD_NAME" --\r\n" \
"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)


/** The maximal digital value */
/** 2^12 - 1                  */
#define MAX_DIGITAL     (4095)


/**
* potenciometrO
*/

#define ANALOG_CHANNEL				 2




/************************************************************************/
/* Globals                                                              */
/************************************************************************/


/** The conversion data is done flag */
volatile bool g_is_conversion_done = false;
volatile bool g_is_conversion_done_analog = false;
/** The conversion data value */
volatile uint32_t g_ul_value = 0;
volatile uint32_t g_ul_value_analog = 0;

/* Canal do sensor de temperatura */
#define AFEC_CHANNEL_TEMP_SENSOR 11




/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
//void pin_toggle(Pio *pio, uint32_t mask);

/************************************************************************/
/* Callbacks: / Handler                                                 */
/************************************************************************/

/**
*  Interrupt handler for TC1 interrupt.
*/

void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	afec_channel_enable(AFEC0, ANALOG_CHANNEL);

	afec_start_software_conversion(AFEC0);
		
}

/**
 * \brief AFEC interrupt callback function.
 */

static void AFEC_Temp_callback(void)
{
	g_ul_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
	g_is_conversion_done = true;
}

static void AFEC_Pot_callback(void)
{
	g_ul_value_analog = afec_channel_get_value(AFEC0, ANALOG_CHANNEL);
	g_is_conversion_done_analog = true;
}

/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

/**
 * \brief Configure UART console.
 * BaudRate : 115200
 * 8 bits
 * 1 stop bit
 * sem paridade
 */
/**
void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}
*/
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

static void configure_console(void)
{

	const usart_serial_options_t uart_serial_options = {
		.baudrate   = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits   = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/**
 * converte valor lido do ADC para temperatura em graus celsius
 * input : ADC reg value
 * output: Temperature in celsius
 */


static int32_t convert_adc_to_volume(int32_t ADC_value){

  int32_t ul_vol;
  int32_t ul_pot;
  int32_t ul_volume;
  

  /*
   * converte bits -> tensão (Volts)
   */
	ul_vol = ADC_value * VOLT_REF / (float) MAX_DIGITAL;

  /*
   * According to datasheet, The output voltage VT = 0.72V at 27C
   * and the temperature slope dVT/dT = 2.33 mV/C
   */
  //ul_pot = (ul_vol - 720)  * 100 / 233 + 27;
  ul_volume = (16*(ul_vol)-300)/3270;
  printf("C: %d",ul_vol);
  printf("VOUMW: %d",ul_volume);
  
  return(ul_volume);
}

static void config_ADC_TEMP(void){
/*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
	afec_enable(AFEC0);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(AFEC0, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(AFEC0, AFEC_TRIG_SW);

	/* configura call back */
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_11,	AFEC_Temp_callback, 1);
	
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_2,	AFEC_Pot_callback, 1);

	/*** Configuracao específica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_TEMP_SENSOR, &afec_ch_cfg);
	afec_ch_set_config(AFEC0, ANALOG_CHANNEL, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	 down to 0.
	 */
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_TEMP_SENSOR, 0x200);
	
	afec_channel_set_analog_offset(AFEC0, ANALOG_CHANNEL, 0x200);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);
	

	/* Selecina canal e inicializa conversão */
	afec_channel_enable(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
	
	afec_channel_enable(AFEC0, ANALOG_CHANNEL);
}


/************************************************************************/
/* Main                                                                 */
/************************************************************************/
/**
 * \brief Application entry point.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{

	/* Initialize the SAM system. */
	sysclk_init();
	ioport_init();
	board_init();
	
	TC_init(TC0, ID_TC1, 1, 10);

	/* inicializa delay */
	delay_init(sysclk_get_cpu_hz());

	/* inicializa console (printf) */
	configure_console();

	/* inicializa e configura adc */
	config_ADC_TEMP();

	/* Output example information. */
	puts(STRING_HEADER);
	
	/* incializa conversão ADC */
	afec_start_software_conversion(AFEC0);
	while (1) {
		if(g_is_conversion_done == true) {
			g_is_conversion_done = false;
			printf("");		
		}
		if(g_is_conversion_done_analog == true) {
			g_is_conversion_done_analog = false;
			printf("Pot : %d \r\n", (g_ul_value_analog));
			convert_adc_to_volume(g_ul_value_analog);
		}
		
	}
}
