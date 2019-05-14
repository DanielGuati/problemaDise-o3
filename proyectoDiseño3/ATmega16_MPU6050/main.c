/*
 * Problema de diseño 3: Juego de arcade Snake con implementación de acelerómetro y cuatro módulos de matriz de LEDS 8x8
 *
 * Andrés Felipe Florez Sierra
 * Daniel Mateo Guatibonza Solano
 * Juan Felipe Ramos Correa
 */ 
	
// Definición de librerías

	#define F_CPU 16000000UL					// Define la velocidad del reloj del microcontrolador. F = 16 MHz.
	#include <avr/io.h>										
	#include <util/delay.h>									
	#include <inttypes.h>									
	#include <stdlib.h>										
	#include <stdio.h>										
	#include <math.h>							// Librería que permite usar funciones matemáticas.			
	#include <avr/pgmspace.h>	
						
	#include "MPU6050_res_define.h"				// Librería que define las constantes de la comunicación mediante el protocolo I2C.
	#include "I2C_Master_H_file.h"				// Librería que implementa el protocolo de la comunicación I2C;
	#include "USART_RS232_H_file.h"				// Librería que permite la comunicación serial entre el microcontrolador y el computador.

// Definición de vector de constantes

	// Vector de inicialización de la matriz.
	const  uint32_t ceros[8] = {
		0b0000000000000000000000000000000000000000,
		0b0000000000000000000000000000000000000000,
		0b0000000000000000000000000000000000000000,
		0b0000000000000000000000000000000000000000,
		0b0000000000000000000000000000000000000000,
		0b0000000000000000000000000000000000000000,
		0b0000000000000000000000000000000000000000,
		0b0000000000000000000000000000000000000000
	};

	// Vector que indica la posición inicial del snake
	const  uint32_t inicioJuego[8] = 
	{
		0b00000000000000000000000000000000,
		0b00000000000000000000000000000000,
		0b00000000000000000000000000000000,
		0b00000000000000111100000000000000,
		0b00000000000000000000000000000000,
		0b00000000000000000000000000000000,
		0b00000000000000000000000000000000,
		0b00000000000000000000000000000000
	};

	// Vector que indica que el jugador perdió.
	const  uint32_t perdedor[8] = 
	{
		0b00000000000000000000000000000000,
		0b01000011110111101111011110010100,
		0b01000010010100001000010001010100,
		0b01000010010111101111011110010100,
		0b01000010010000101000011000010100,
		0b01000010010000101000010100000000,
		0b01111011110111101111010010010100,
		0b00000000000000000000000000000000
	};

	// Vector que indica que el jugador ganó.
	const  uint32_t ganador[8] = {
		0b00000000000000000000000000000000,
		0b01000101110100001000011001100000,
		0b01000100100110001000011001100000,
		0b01000100100101001000000000000000,
		0b01000100100100101000100000010000,
		0b01010100100100011000010000100000,
		0b00101001110100001000001111000000,
		0b00000000000000000000000000000000
	};

// Código correspondiente a la visualización de información en la matriz de LEDS 8x8.

	// Definición de variables correspondientes a la comunicación entre el microcontrolador y la matriz de LEDS 8x8.

	#define CLK_HIGH()  PORTD |= _BV(PD4)  
	#define CLK_LOW()   PORTD &= ~_BV(PD4) 
	#define CS_HIGH()   PORTD |= _BV(PD3)
	#define CS_LOW()    PORTD &= ~_BV(PD3)
	#define DATA_HIGH() PORTD |= _BV(PD2)
	#define DATA_LOW()  PORTD &= ~_BV(PD2)

	// Vector auxiliar de visualización.
	uint32_t display[8];

	// Definición de funciones que permiten la comunicación entre el microcontrolador y la matriz de LEDS 8x8.

	// Funcion enviar por SPI
	void spi_send(uint8_t data)	
	{
		uint8_t i;
		
		for (i = 0; i < 8; i++, data <<= 1)	// Realiza el barrido de las 8 posiciones del vector data
		{
			CLK_LOW();
			if (data & 0x80000000)
			DATA_HIGH();
			else
			DATA_LOW();
			CLK_HIGH();
		}
	
	}
	
	// Escribir en la matriz.
	void max7219_writec(uint8_t high_byte, uint32_t low_byte)
	{
		CS_LOW();
		spi_send(high_byte);
		spi_send(low_byte);
		CS_HIGH();
	}
	
	// Limpiar la matriz.
	void max7219_clear(void)
	{
		uint8_t i;
		for (i = 0; i < 8; i++)
		{
			max7219_writec(i+1, 0);
		}
	}
	
	// Inicializar la matriz.
	void max7219_init(void)
	{
	
		// Decode mode: none
		max7219_writec(0x04, 0);
		// Intensity: 3 (0-15)
		max7219_writec(0x0A, 1);
		// Scan limit: All "digits" (rows) on
		max7219_writec(0x0B, 7);
		// Shutdown register: Display on
		max7219_writec(0x0C, 1);
		// Display test: off
		max7219_writec(0x0F, 0);
		max7219_clear();
	}
	
	// Actualizar la pantalla.
	void update_display(void)
	{
		uint8_t i;
		for (i = 0; i < 8; i++)
		{
			max7219_writec(i+1, display[i]);
		}
	}
	
	// Carga el vector de la imagen que se quiere visualizar.
	void image(const uint32_t im[8])
	{
		uint8_t i;
		for (i = 0; i < 8; i++)
		display[i] = im[i];
	}
	
	 // Método que fija un pixel en 0, 1 o lo invierte dependiendo del parámetro que recibe.
	void set_pixel(uint8_t fila, uint8_t columna, uint8_t value)
	{
		switch (value)
		{
			case 0:									// Clear bit
			display[fila] &= (uint32_t) ~(0x80000000 >> columna);
			break;
			case 1:									// Set bit
			display[fila] |= (0x800000000 >> columna);
			break;
			default:								// XOR bit
			display[fila] ^= (0x800000000 >> columna);
			break;
		}
	}

// Código correspondiente a la comunicación entre el acelerómetro y la Expresso.

	// Definición de valores binarios que envía el acelerómetro.
	float Acc_x,Acc_y,Acc_z,Temperature,Giro_x,Giro_y,Giro_z;

	// Inicializa la comunicación entre la Expressp y el acelerómetro mediante el protocolo I2C.
	void MPU6050_Init()										/* Gyro initialization function */
	{
		_delay_ms(150);										/* Power up time >100ms */
		I2C_Start_Wait(0xD0);								/* Start with device write address */
		I2C_Write(SMPLRT_DIV);								/* Write to sample rate register */
		I2C_Write(0x07);									/* 1KHz sample rate */
		I2C_Stop();

		I2C_Start_Wait(0xD0);
		I2C_Write(PWR_MGMT_1);								/* Write to power management register */
		I2C_Write(0x01);									/* X axis gyroscope reference frequency */
		I2C_Stop();

		I2C_Start_Wait(0xD0);
		I2C_Write(CONFIG);									/* Write to Configuration register */
		I2C_Write(0x00);									/* Fs = 8KHz */
		I2C_Stop();

		I2C_Start_Wait(0xD0);
		I2C_Write(GYRO_CONFIG);								/* Write to Gyro configuration register */
		I2C_Write(0x18);									/* Full scale range +/- 2000 degree/C */
		I2C_Stop();

		I2C_Start_Wait(0xD0);
		I2C_Write(INT_ENABLE);								/* Write to interrupt enable register */
		I2C_Write(0x01);
		I2C_Stop();
	}

	// Método que inicializa las direcciones de los dispositivos (Acelerómetro y microcontrolador).
	void MPU_Start_Loc()
	{
		I2C_Start_Wait(0xD0);								/* I2C start with device write address */
		I2C_Write(ACCEL_XOUT_H);							/* Write start location address from where to read */ 
		I2C_Repeated_Start(0xD1);							/* I2C start with device read address */
	}


	// Método que permite obtener y guardar los valores provenientes del acelerómetro.
	void Read_RawValue()
	{
		MPU_Start_Loc();									/* Read Gyro values */
		Acc_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
		Acc_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
		Acc_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
		Temperature = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
		Giro_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
		Giro_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
		Giro_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Nack());
		I2C_Stop();
	}

// Código correspondiente a la implementación del juego.
	
	// Dirección [0->Arriba; 1->abajo; 2->Derecha; 3->Izquierda]
	uint8_t direccion;
	
	// Tamaño
	uint8_t largo;
	
	// Filas
	uint8_t *filas;
		
	// Columnas
	uint8_t *columnas;
						
	// Fila galleta		
	uint8_t filaGalleta;
		
	// Columna galleta
	uint8_t columnaGalleta;
	
	// Galleta encontrada
	uint8_t galletaEncontrada;

	// Método que permite inicializar el juego.
	void inicializarJuego()
	{
		image(inicioJuego);  
		update_display();
		_delay_ms(250);
	}
	
	// Método que verifica si el jugador ganó el juego
	int juegoGanado()
	{
		int gano = 1;
		uint8_t i;
		for (i = 0; i < 8; i++)
		{
			if(~(display[i] == 0x80000000))
			{
				gano = 0;
			}
			if(gano == 0)
			{
				break;
			}
		}
		return gano;
	}
	
	int juegoPerdido(uint8_t filaSiguiente, uint8_t columnaSiguiente)
	{
		int perdio = 0;
		uint8_t i;
		for (i = 0; i < largo; i++)
		{
			if(perdio == 1)
			{
				break;
			}
			else
			{
				if(filas[i] == filaSiguiente && columnas[i] == columnaSiguiente)
				{
					perdio = 1;
				}
			}
		}
		return perdio;
	}
	
	// Método que busca las coordenadas de la siguiente galleta
	void buscarGalleta()
	{
		// Se busca la siguiente galleta
		uint8_t posicionGalletaEncontrada = 0;
		while (posicionGalletaEncontrada == 0)
		{
			filaGalleta = rand() % 8;
			columnaGalleta = rand() % 32;
			if(display[filaGalleta] && (uint8_t) ~(0x80000000 >> columnaGalleta) == 0x80000000)
			{
				posicionGalletaEncontrada = 1;
			}
		}
		galletaEncontrada = 0;
		set_pixel(filaGalleta,columnaGalleta,1);
		update_display();
	}
	
	// Método que mueve al snake a la posición determinada por los parámetros.
	void moverSnake(int nuevaFila, int nuevaColumna)
	{
		set_pixel(filas[0],columnas[0],0);
		update_display();
		
		uint8_t i;
		for (i = 0; i < largo - 1; i++)
		{
			filas[i] = filas[i+1];
			columnas[i] = columnas[i+1];
		}
		filas[largo-1] = nuevaFila;
		columnas[largo-1] = nuevaColumna;
		
		set_pixel(nuevaFila,nuevaColumna,1);
		update_display();
	}

	// Método principal que permite la inicialización y funcionamiento del juego.
	int main()
	{
		// Inicialización de todos los pines del puerto D como salidas.
		DDRD = 0b11111111;
		
		// Inicialización de todos los pines del puerto B como salidas excepto el 7.
		DDRB = 0b01111111;
		
		// Inicialización de todos los pines del puerto B como salidas excepto el 7.
		DDRC = 0b11111000;
			
		// Declaración de las variables que indican la inclinación de la matriz.
		float anguloX, anguloY;
	
		// Constante de conversión de valor de acelerómetro a gravedad en [g].
		float A_R = 16384;
		
		// Constante de conversión de radianes a grados [180/pi].
		float RAD_A_DEG = 57.295779;
	
	
	// Variables del funcionamiento del juego.
		direccion = 2;
		largo = 4;
		filas[0] = 3; filas[1] = 3; filas[2] = 3; filas[3] = 3;
		columnas[0] = 14; columnas[1] = 15; columnas[2] = 16; columnas[3] = 17;
		galletaEncontrada = 0;
	
		// Indica si el jugador ganó [1 si ya ganó - 0 si no ha ganado].
		int gano = 0;
		
		// Indica si el jugador perdió [1 si ya perdió - 0 si no ha perdido].
		int perdio = 0;
	
		I2C_Init();
		MPU6050_Init();
	//  USART_Init(9600);  // se inicializa la comunicación serial [se comenta para mejorar rendimiento del microcontrolador].
	
		max7219_init();
	
		image(ceros);  
		update_display();
	
		inicializarJuego();
		update_display();
	
		/*
		* Ciclo infinito que permite el funcionamiento del juego.
		*/
		while(1)
		{	
			// Se verifica si el jugador ganó el juego
			gano = juegoGanado();
			// Si el jugador ganó se muestra al usuario
			if(gano == 1)
			{
				image(ganador);
				update_display();
				_delay_ms(1000);	
				image(ceros);
				update_display();
				inicializarJuego();
				update_display();
				gano = 0;
			}
			else
			{
				// Se busca una nueva galleta en caso de haber encontrado la enterior
				if(galletaEncontrada == 1)
				{
					// Se busca la siguiente galleta
					buscarGalleta();
				}
				
				// Cálculo del ángulo del sensor
				Read_RawValue();
				anguloX = atan(-1*(Acc_x/A_R)/sqrt(pow((Acc_y/A_R),2) + pow((Acc_z/A_R),2)))*RAD_A_DEG;					
				anguloY = atan((Acc_y/A_R)/sqrt(pow((Acc_x/A_R),2) + pow((Acc_z/A_R),2)))*RAD_A_DEG;			
			
				// Verificación de la dirección de movimiento
				if(anguloX < 0)
				{
					anguloX = -anguloX;
					if(anguloY < 0)
					{
						anguloY = -anguloY;
						if(anguloX > anguloY) // Ir a la izquierda
						{
							if(direccion == 0 || direccion == 1)
							{
								direccion = 3;
							}
						}
						else // Ir para arriba
						{
							if(direccion == 2 || direccion == 3)
							{
								direccion = 0;
							}
						}
					}
					else
					{
						if(anguloX > anguloY) // Ir a la izquierda
						{
							if(direccion == 0 || direccion == 1)
							{
								direccion = 3;
							}
						}
						else // Ir para abajo
						{
							if(direccion == 2 || direccion == 3)
							{
								direccion = 1;
							}
						}
					}
				}
				else
				{
					if(anguloY < 0)
					{
						anguloY = -anguloY;
						if(anguloX > anguloY) // Ir a la derecha
						{
							if(direccion == 0 || direccion == 1)
							{
								direccion = 2;
							}
						}
						else // Ir para arriba
						{
							if(direccion == 2 || direccion == 3)
							{
								direccion = 0;
							}
						}
					}
					else
					{
						if(anguloX > anguloY) // Ir a la derecha
						{
							if(direccion == 0 || direccion == 1)
							{
								direccion = 2;
							}
						}
						else // Ir para abajo
						{
							if(direccion == 2 || direccion == 3)
							{
								direccion = 1;
							}
						}
					}
				}
				
				// Ubicación de la siguiente coordenada del snake
				uint8_t filaActual = filas[largo-1];
				uint8_t columnaActual = columnas[largo-1];
				uint8_t filaSiguiente = filas[largo-1];
				uint8_t columnaSiguiente = columnas[largo-1];
				if(direccion==0)
				{
					filaSiguiente = filaActual - 1;
					if(filaSiguiente == -1)
					{
						filaSiguiente = 7;
					}
				}
				if(direccion==1)
				{
					filaSiguiente = filaActual + 1;
					if(filaSiguiente == 8)
					{
						filaSiguiente = 0;
					}
				}
				if(direccion==2)
				{
					columnaSiguiente = columnaActual + 1;
					if(columnaSiguiente == 32)
					{
						columnaSiguiente = 0;
					}
				}
				if(direccion==3)
				{
					columnaSiguiente = columnaSiguiente - 1;
					if(columnaSiguiente == -1)
					{
						columnaSiguiente = 31;
					}
				}
				
				// Verificación de pérdida
				perdio = juegoPerdido(filaSiguiente,columnaSiguiente);
				
				// Se informa al usuario que perdió
				if(perdio==1)
				{
					image(perdedor);
					update_display();
					_delay_ms(1000);
					image(ceros);
					update_display();
					inicializarJuego();
					update_display();
					perdio = 0;
				}
				else
				{
					// Se verifica si el jugador llegó a una galleta
					if(filaGalleta == filaSiguiente && columnaGalleta == columnaSiguiente)
					{
						galletaEncontrada = 1;
						largo = largo + 1;
						filas[largo-1] = filaGalleta;
						columnas[largo-1] = columnaGalleta;
					}
					else
					{
						moverSnake(filaSiguiente,columnaSiguiente);
					}
					_delay_ms(250);
				}
			}
	
		}
}