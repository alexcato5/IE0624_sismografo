/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2014 Chuck McManis <cmcmanis@mcmanis.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "clock.h"
#include "console.h"
#include "sdram.h"
#include "lcd-spi.h"
#include "gfx.h"

// Giroscopio
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h> // Librería para el uso de USART
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>

// Biblioteca ADC
#include <libopencm3/stm32/adc.h>

// +++++++++++++++++++++++++++++++++ BATERÍA ++++++++++++++++++++++++++++++++++++++++++++++
// // Declaración de variables globales relacionadas con la batería
uint16_t battery; // Almacena el valor de la medición de la batería
uint8_t batt_alarm; // Utilizada para indicar una alarma de batería
// ++++++++++++++++++++++++++++++++++ FIN BATERÍA +++++++++++++++++++++++++++++++++++++++++++++


/* CODIGO PARA GIROSCOPIO */
static void spi_setup(void)
{
	rcc_periph_clock_enable(RCC_SPI5);
	/* For spi signal pins */
	rcc_periph_clock_enable(RCC_GPIOC);
	/* For spi mode select on the l3gd20 */
	rcc_periph_clock_enable(RCC_GPIOF);
	/* Setup GPIOE3 pin for spi mode l3gd20 select. */
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
	/* Start with spi communication disabled */
	gpio_set(GPIOC, GPIO1);
//

	/* Setup GPIO pins for AF5 for SPI5 signals. */
	gpio_mode_setup(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO7 | GPIO8 | GPIO9);
	gpio_set_af(GPIOF, GPIO_AF5, GPIO7 | GPIO8 | GPIO9);

	//spi initialization;
	spi_set_master_mode(SPI5);
	spi_set_baudrate_prescaler(SPI5, SPI_CR1_BR_FPCLK_DIV_64);
	spi_set_clock_polarity_0(SPI5);
	spi_set_clock_phase_0(SPI5);
	spi_set_full_duplex_mode(SPI5);
	spi_set_unidirectional_mode(SPI5); /* bidirectional but in 3-wire */
	//spi_set_data_size(SPI5, SPI_CR2_DS_8BIT);
	spi_enable_software_slave_management(SPI5);
	spi_send_msb_first(SPI5);
	spi_set_nss_high(SPI5);
	//spi_enable_ss_output(SPI1);
	//spi_fifo_reception_threshold_8bit(SPI5);
	SPI_I2SCFGR(SPI5) &= ~SPI_I2SCFGR_I2SMOD;
	spi_enable(SPI5);

// +++++++++++++++++++++++++++++++++ BATERÍA ++++++++++++++++++++++++++++++++++++++++++++++
// Activo los pines necesarios, puertos G, 13 y 14
	/* Enable GPIOG clock. */
	rcc_periph_clock_enable(RCC_GPIOG);
	/* Set GPIO13 (in GPIO port G) to 'output push-pull'. */
	gpio_mode_setup(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13 | GPIO14);
// ++++++++++++++++++++++++++++++++++ FIN BATERÍA +++++++++++++++++++++++++++++++++++++++++++++

}//fin del void set up


// ++++++++++++++++++++++++++  COMUNICACIÓN USART +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void usart_setup(void)
{
    // Habilitar los clocks para el puerto GPIO A (para GPIO_USART2_TX) y USART2.
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_GPIOA);

	// Configurar el pin GPIO_USART2_TX/GPIO9 en el puerto GPIO A para transmitir.
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2| GPIO3);

	// Configurar los parámetros de UART
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	// Finalmente, habilitar el USART.
	usart_enable(USART2);
}

// Esta función se encarga de enviar un entero a través de USART. Utiliza la función usart_send_blocking() para enviar los caracteres uno por uno.
static void my_usart_print_int(uint32_t usart, int32_t value)
{
	int8_t i;
	int8_t nr_digits = 0;
	char buffer[25];

	if (value < 0) {
        // Si el valor es negativo, enviar el signo '-' por el USART y hacer que el valor sea positivo.
		usart_send_blocking(usart, '-'); // líneas envían caracteres a través de USART para imprimir un entero en formato decima
		value = value * -1;
	}

	if (value == 0) {
        // Si el valor es cero, enviar el carácter '0' por el USART.
		usart_send_blocking(usart, '0');
	}

	while (value > 0) {
        // Convertir el valor entero en una representación de caracteres decimal.
		buffer[nr_digits++] = "0123456789"[value % 10];
		value /= 10;
	}

	for (i = nr_digits-1; i >= 0; i--) {
        // Enviar cada carácter del buffer por el USART en orden inverso para mantener el valor correcto.
		usart_send_blocking(usart, buffer[i]);
	}

    // Enviar un salto de línea ('\r') y un nueva línea ('\n') para mover el cursor al inicio de la siguiente línea después de imprimir el valor entero.
	//  imprimir las coordenadas X, Y y Z
    usart_send_blocking(usart, '\r');
	usart_send_blocking(usart, '\n');
}

// +++++++++++++++++++++++++++++++++++  FIN COMUNICACIÓN USART  +++++++++++++++++++++++++++++++++++++++++++++++++++


// +++++++++++++++++++++++++++++++++ BATERÍA ++++++++++++++++++++++++++++++++++++++++++++++
// Configuración del ADC (Convertidor Analógico-Digital)
static void adc_setup(void)
{
	rcc_periph_clock_enable(RCC_ADC1);  // Habilitar el reloj del ADC
  	rcc_periph_clock_enable(RCC_GPIOA); // Habilitar el reloj del puerto GPIOA
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1); // Configurar el pin GPIOA1 como entrada analógica

	adc_power_off(ADC1); // Apagar el ADC antes de la configuración
  	adc_disable_scan_mode(ADC1); // Deshabilitar el modo de escaneo del ADC
  	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC); // Establecer el tiempo de muestreo para todas las entradas del ADC

	adc_power_on(ADC1);  // Encender el ADC después de la configuración
}

// Lectura del ADC para medir la batería
static uint16_t read_adc_naiive(uint8_t channel)
{
	uint8_t channel_array[16];
	channel_array[0] = channel;
	adc_set_regular_sequence(ADC1, 1, channel_array);  // Configurar una secuencia regular de conversión con un solo canal
	adc_start_conversion_regular(ADC1); // Iniciar la conversión
	while (!adc_eoc(ADC1)); // Esperar a que la conversión termine
	uint16_t reg16 = adc_read_regular(ADC1); // Leer el valor convertido del registro regular del ADC
	return reg16;
}

// Actualización de la medición de la batería
void adc_update(void){
	battery = read_adc_naiive(1)*9/4095; // Actualización de la medición de la batería
                                            // Realizar una operación de escala y ajuste para convertir el valor del ADC en un voltaje aproximado de la batería
}

// En el código principal (main), se llama a la función adc_setup para configurar el ADC antes de iniciar el bucle principal
// Luego, dentro del bucle principal, se llama a adc_update para actualizar periódicamente la medición de la batería
// El valor de la batería se utiliza posteriormente en la interfaz de usuario en la pantalla LCD para mostrar el nivel de batería.
// ++++++++++++++++++++++++++++++++++ FIN BATERÍA +++++++++++++++++++++++++++++++++++++++++++++



#define GYR_RNW			(1 << 7) /* Write when zero */
#define GYR_MNS			(1 << 6) /* Multiple reads when 1 */
#define GYR_WHO_AM_I		0x0F
#define GYR_OUT_TEMP		0x26
#define GYR_STATUS_REG		0x27
#define GYR_CTRL_REG1		0x20
#define GYR_CTRL_REG1_PD	(1 << 3)
#define GYR_CTRL_REG1_XEN	(1 << 1)
#define GYR_CTRL_REG1_YEN	(1 << 0)
#define GYR_CTRL_REG1_ZEN	(1 << 2)
#define GYR_CTRL_REG1_BW_SHIFT	4
#define GYR_CTRL_REG4		0x23
#define GYR_CTRL_REG4_FS_SHIFT	4

#define GYR_OUT_X_L		0x28
#define GYR_OUT_X_H		0x29
#define GYR_OUT_Y_L		0x2A
#define GYR_OUT_Y_H		0x2B
#define GYR_OUT_Z_L		0x2C
#define GYR_OUT_Z_H		0x2D

/* FIN CODIGO GIROSCOPIO */

/*
 * This is our example, the heavy lifing is actually in lcd-spi.c but
 * this drives that code.
 */
int main(void)
{

	clock_setup();
	console_setup(115200); // Importante para el uso de USART

	/******************************* Para uso del giroscopio*/
	spi_setup();
	uint8_t temp;
	int16_t gyr_x = 0;
	int16_t gyr_y = 0;
	int16_t gyr_z = 0;

	uint8_t usart_encendido = 0;

//	usart_setup();

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_CTRL_REG1);
	spi_read(SPI5);
	spi_send(SPI5, GYR_CTRL_REG1_PD | GYR_CTRL_REG1_XEN |
			GYR_CTRL_REG1_YEN | GYR_CTRL_REG1_ZEN |
			(3 << GYR_CTRL_REG1_BW_SHIFT));
	spi_read(SPI5);
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_CTRL_REG4);
	spi_read(SPI5);
	spi_send(SPI5, (1 << GYR_CTRL_REG4_FS_SHIFT));
	spi_read(SPI5);
	gpio_set(GPIOC, GPIO1);

	sdram_init();
	lcd_spi_init();
	/***************************************************************/

	//console_puts("LCD Initialized\n");
	//console_puts("Should have a checker pattern, press any key to proceed\n");
	//msleep(2000);

    // +++++++++++++++++++++++++++++++++++  USART  +++++++++++++++++++++++++++++++++++++++++++++++++++
    /*Esta línea espera a recibir un carácter a través de USART. La función console_getc() bloquea la ejecución 
    hasta que se recibe un carácter y luego lo devuelve.*/
	(void) console_getc(1); 
    // +++++++++++++++++++++++++++++++++++  FIN COMUNICACIÓN USART  +++++++++++++++++++++++++++++++++++++++++++++++++++
	gfx_init(lcd_draw_pixel, 240, 320);
	gfx_fillScreen(LCD_GREY);
	gfx_fillRoundRect(10, 10, 220, 220, 5, LCD_WHITE);
	gfx_drawRoundRect(10, 10, 220, 220, 5, LCD_BLUE);
	gfx_fillCircle(20, 250, 10, LCD_YELLOW);
	gfx_fillCircle(120, 250, 10, LCD_BLACK);
	gfx_fillCircle(220, 250, 10, LCD_YELLOW);
	gfx_setTextSize(2);
	gfx_setCursor(15, 25);
	gfx_puts("Laboratorio 4");
	gfx_setCursor(15, 49);
	gfx_puts("-SISMOGRAFO-");
	gfx_setTextSize(1);
	gfx_setCursor(15, 86);
	gfx_puts("B61325 Alexander Calderon");
	gfx_setCursor(15, 97);
	gfx_puts("B60425 Johander Anchia");
	lcd_show_frame();
	msleep(2000);

    // +++++++++++++++++++++++++++++++++++  USART +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	(void) console_getc(1); 
    // +++++++++++++++++++++++++++++++++++  FIN COMUNICACIÓN USART  +++++++++++++++++++++++++++++++++++++++++++++++++++

	while (1) {
		gfx_setTextColor(LCD_YELLOW, LCD_BLACK);
		gfx_setTextSize(2);
		gfx_fillScreen(LCD_BLACK);
		gfx_setCursor(15, 36);
		gfx_puts("- SISMOGRAFO -");
		gfx_fillCircle(40, 120, 20+gyr_x*(0.00875F), LCD_WHITE);
		gfx_fillCircle(120, 120, 20+gyr_y*(0.00875F), LCD_GREY);
		gfx_fillCircle(200, 120, 20+gyr_z*(0.00875F), LCD_YELLOW);

		/*******************************************************************/
		gpio_clear(GPIOC, GPIO1);
		spi_send(SPI5, GYR_WHO_AM_I | GYR_RNW);
		spi_read(SPI5);
		spi_send(SPI5, 0);
		temp=spi_read(SPI5);
//		my_usart_print_int(USART2, (temp));
		gpio_set(GPIOC, GPIO1);

		gpio_clear(GPIOC, GPIO1);
		spi_send(SPI5, GYR_STATUS_REG | GYR_RNW);
		spi_read(SPI5);
		spi_send(SPI5, 0);
		temp=spi_read(SPI5);
//		my_usart_print_int(USART2, (temp));
		gpio_set(GPIOC, GPIO1);

		gpio_clear(GPIOC, GPIO1);
		spi_send(SPI5, GYR_OUT_TEMP | GYR_RNW);
		spi_read(SPI5);
		spi_send(SPI5, 0);
		temp=spi_read(SPI5);
//		my_usart_print_int(USART2, (temp));
		gpio_set(GPIOC, GPIO1);

		gpio_clear(GPIOC, GPIO1);
		spi_send(SPI5, GYR_OUT_X_L | GYR_RNW);
		spi_read(SPI5);
		spi_send(SPI5, 0);
		gyr_x=spi_read(SPI5);
		gpio_set(GPIOC, GPIO1);

		gpio_clear(GPIOC, GPIO1);
		spi_send(SPI5, GYR_OUT_X_H | GYR_RNW);
		spi_read(SPI5);
		spi_send(SPI5, 0);
		gyr_x|=spi_read(SPI5) << 8;
//		my_usart_print_int(USART2, (gyr_x));
		gpio_set(GPIOC, GPIO1);

		gpio_clear(GPIOC, GPIO1);
		spi_send(SPI5, GYR_OUT_Y_L | GYR_RNW);
		spi_read(SPI5);
		spi_send(SPI5, 0);
		gyr_y=spi_read(SPI5);
		gpio_set(GPIOC, GPIO1);

		gpio_clear(GPIOC, GPIO1);
		spi_send(SPI5, GYR_OUT_Y_H | GYR_RNW);
		spi_read(SPI5);
		spi_send(SPI5, 0);
		gyr_y|=spi_read(SPI5) << 8;
//		my_usart_print_int(USART2, (gyr_y));
		gpio_set(GPIOC, GPIO1);

		gpio_clear(GPIOC, GPIO1);
		spi_send(SPI5, GYR_OUT_Z_L | GYR_RNW);
		spi_read(SPI5);
		spi_send(SPI5, 0);
		gyr_z=spi_read(SPI5);
		gpio_set(GPIOC, GPIO1);

		gpio_clear(GPIOC, GPIO1);
		spi_send(SPI5, GYR_OUT_Z_H | GYR_RNW);
		spi_read(SPI5);
		spi_send(SPI5, 0);
		gyr_z|=spi_read(SPI5) << 8;
//		my_usart_print_int(USART2, (gyr_z));
		gpio_set(GPIOC, GPIO1);
		/************************************************************/

		// Indicador de giroscopio
		gfx_setTextSize(2);
		gfx_setCursor(30, 200);
		gfx_puts("X: ");
		gfx_setCursor(15, 240);
		char gyr_x_str[16];
		sprintf(gyr_x_str, "%d", (int)(gyr_x*0.00875F));
		gfx_puts(gyr_x_str);

		gfx_setCursor(110, 200);
		gfx_puts("Y: ");
		gfx_setCursor(100, 240);
		char gyr_y_str[16];
		sprintf(gyr_y_str, "%d", (int)(gyr_y*0.00875F));
		gfx_puts(gyr_y_str);
		
		gfx_setCursor(190, 200);
		gfx_puts("Z: ");
		gfx_setCursor(175, 240);
		char gyr_z_str[16];
		sprintf(gyr_z_str, "%d", (int)(gyr_z*0.00875F));
		gfx_puts(gyr_z_str);
		
		

		// Indicador de USART
		// gfx_setCursor(155, 310);
		// if (usart_encendido){ gfx_puts("USART:  ON"); }
		// else { gfx_puts("USART: OFF"); }
		
		// +++++++++++++++++++++++++++++++++++  BATERÍA ++++++++++++++++++++++++++++++++++++++++++++++
		// gfx_setTextSize(1);                  
		// gfx_setCursor(5, 310);
		// gfx_puts("Bateria: 100%");
		
		// Alarma de batería
		if(battery <= 7){
			batt_alarm = 1;
			gpio_set(GPIOG, GPIO14);
		}
		else{
			batt_alarm = 0;
			gpio_clear(GPIOG, GPIO14);
		} 
		
		// ++++++++++++++++++++++++++++++++++ FIN BATERÍA ++++++++++++++++++++




		lcd_show_frame();
	} // fin del loop
}
