//-----------------------------------------------------------------------------//
//--------------------------------Grupo1---------------------------------------//
//------------------------- ----04/04/2023-------------------------------------//
//-----------------------------------------------------------------------------//
//----------------------------Realizado por:-----------------------------------//
//--------------David Roncero, Luis Pérez y Antonio Chapinal-------------------//
//-----------------------------------------------------------------------------//
//-----------------------------------------------------------------------------//
//------------------------------Terraneitor------------------------------------//
//-----------------------------------------------------------------------------//
//-----------------------------------------------------------------------------//


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Utiles_SDM.h"



//Declaración de constantes
#define tiempototal  60000;
#define time  500;
#define time2 1000;


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//Variable que controlan la modulación de la señales echo y trigger
uint16_t inicio_tiempo_echo = 0;
uint16_t inicio_tiempo_trigger = 0;
uint16_t final_tiempo_echo =0;
uint16_t final_tiempo_trigger =0;


uint16_t diferencia_tiempo = 0;
int zona_de_obstaculos = 0;
int modo_automatico = 0;
uint16_t pasos_giro =0;
uint16_t pasos_suma = 0;
uint8_t estado = 0;
uint16_t distancia = 0;

int numero= 0;
unsigned short valor = 0;
int adc_on=0;
uint8_t estado_zumbador = 0;
short DC = 0;
short DC_2 = 0;

uint8_t texto[6] = "      ";
uint8_t texto2[6] = "      ";
signed short contador = 0;


int control_adelante = 1;
int control_atras = 1;
int control_izq = 1;
int control_derecha = 1;
int control_automatico = 1;
int inicio = 1;

signed short velocidad;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TS_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//Interrupción asociada a un contador en modo TOC que usamos para contar el tiempo en el sesnor de ultrasonidos

void TIM2_IRQHandler(void) {

   if ((TIM2->SR & 0x0004)!=0) {

       TIM2->CCR2 += tiempototal; //tiempototal es aprox. el tiempo que tardamos en hacer el "truco " enviar una señal cada x tiempo

       estado = 1;

       TIM2->SR &= ~0x04;
      }
   }


//Interrupción usada para el zumbador, cada 0.5 segundos suena

void TIM3_IRQHandler(void) {
   if ((TIM3->SR & 0x0004)!=0) {



       TIM3->CCR2 += time;

       if (estado_zumbador==1) {
           estado_zumbador=0;

       }

       else{
           estado_zumbador=1;
       }

       TIM3->SR &= ~0x04;
      }

}



	void medirDistancia(){

		  if(estado == 1){

			  	GPIOD->BSRR = (1<<2);
				inicio_tiempo_trigger = TIM2->CNT;
				final_tiempo_trigger = inicio_tiempo_trigger + 12;
				while(TIM2->CNT < final_tiempo_trigger);
				GPIOD->BSRR = (1<<2)<<16;



				while((TIM2->SR&(1<<1)) == 0);
				TIM2->SR &= ~(1<<1);
				inicio_tiempo_echo = TIM2->CCR1;


				while((TIM2->SR&(1<<1)) == 0);
				TIM2->SR &= ~(1<<1);
				final_tiempo_echo = TIM2->CCR1;


				diferencia_tiempo = final_tiempo_echo - inicio_tiempo_echo;


			    if(diferencia_tiempo < 0){
			    	diferencia_tiempo += 0x0FFFF;
			     }


				distancia = (unsigned)(0.034*diferencia_tiempo)/2;
				estado = 0;

			}

	}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_TS_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  	  	  	  HAL_UART_Receive_IT(&huart1, texto, 1);


  	  	  GPIOC->MODER &= ~(1 << (12*2 +1));
  	  	  GPIOC->MODER |= (0 << (12*2));


  	  	  	 //Configuramos PA5 como salida, es el ECHO
  	  	  	 GPIOA->MODER |= (1 << (5*2 +1));
             GPIOA->MODER &= ~(1 << (5*2));
             GPIOA->AFR[0]|=(1 << (5*4)); //Pin asociado a la salida del timer
             //GPIOA->AFR[0] &= ~(0x0E << 2*4);


             //PD2 asociado al TRIGGER
             GPIOD->MODER |= (1 << (2*2));
             GPIOD->MODER &= ~(1 << (2*2+1));


             //RUEDA 1

             GPIOB->MODER |=(1 << (2*8 +1)); //Pin con PWM
             GPIOB->MODER &=~(1 << (2*8));

             GPIOA->MODER &= ~(1 << (2*11 +1)); //Pin normal
             GPIOA->MODER |= (1 << (2*11));

             //RUEDA 2
             GPIOB->MODER |=(1 << (2*9 +1));  //Pin con PWM
             GPIOB->MODER &=~(1 << (2*9));

             GPIOA->MODER &= ~(1 << (12*2 +1)); //Pin normal
             GPIOA->MODER |= (1 << (12*2));


             GPIOB->AFR[1]|=(0x02 << (1*4)); //Salida tim4 canal 3 asociado al PB8
             GPIOB->AFR[1]|=(0x02 << (2*4)); //Salida tim4 canal 4 asociado al PB9

             //CONFIGURACION ADC
             GPIOA->MODER |= 0x00000300;    // PA4 como analógico
             ADC1->CR2 &= ~(0x00000001);    // ADON = 0 (ADC apagado)
             ADC1->CR1 = 0x00000000;        // RES = 00 (resolución = 12 bits)
                                            // SCAN = 0 (modo scan deshabilitado)
                                            // EOCIE = 0 (deshabilitada la interupción por EOC)

             ADC1->CR2 = 0x00000412;        // EOCS = 1 (activado el bit EOC al acabar cada conversión)
                                            // DELS = 001 (retardo de la conversión hasta que se lea el dato anterior)
                                            // CONT = 1 (conversión continua)
             ADC1->SMPR1 = 0;               // Sin sampling time (4 cycles)
             ADC1->SMPR2 = 0;
             ADC1->SMPR3 = 0;
             ADC1->SQR1 = 0x00000000;       // 1 elemento solo en la secuencia
             ADC1->SQR5 = 0x00000004;       // El elemento es el canal AIN4
             ADC1->CR2 |= 0x00000001;       // ADON = 1 (ADC activado)

             while ((ADC1->SR&0x0040)==0);  // Si ADCONS = 0, o sea no estoy para convertir,

             ADC1->CR2 |= 0x40000000;



             //Configuracion del timer 4 canales 3 y 4 para el PWM de las ruedas
             TIM4->CR1 = 0x0080;
             TIM4->CR2 = 0x0000;
             TIM4->SMCR = 0x0000;

             TIM4->PSC = 32000;      // Preescalado=32000 -> f_contador=32000000/32000 = 1000 pasos/segundo
             TIM4->CNT = 0;          // Inicializo el valor del contador a cero
             TIM4->ARR = 9;          // Pongo una frecuencia PWM de 100 Hz y sólo cuento 10 pasos

             TIM4->CCR3 = DC;
             TIM4->CCR4 = DC_2;

             TIM4->DIER = 0x0000;   // No se genera INT al terminar de contar -> CCyIE = 0
             TIM4->CCMR2 = 0x6868;
             TIM4->CCER = 0x1100;

             // Habilitación de contador y limpieza de flags
             TIM4->EGR |= 0x0001;   // UG = 1 -> Se genera evento de actualización
             TIM4->SR = 0;          // Limpio los flags del contador
             TIM4->CR1 |= 0x0001;   // CEN = 1 -> Arranco el contador


             //Configuración del timer 2 modo toc para contar el tiempo en el ultrasonidos
             TIM2->CR1 = 0x0000;
             TIM2->CR2 = 0x0000;
             TIM2->SMCR = 0x0000;

             TIM2->PSC = 31;
             TIM2->CNT = 0;       // Inicializo el valor del contador a cero
             TIM2->ARR = 0xFFFF;
             TIM2->CCR1 = 0xFFFF;
             TIM2->CCR2 = tiempototal;

             // Selección de IRQ o no: DIER
             TIM2->DIER |= (1<<2);    // Se genera INT al terminar de contar -> CCyIE = 1

             // Modo de salida del contador
             TIM2->CCMR1 = 0x0001;
             TIM2->CCER = 0x000B;


             // Habilitación de contador y limpieza de flags
             TIM2->EGR |= 0x0001;   // UG = 1 -> Se genera evento de actualización
             TIM2->SR = 0;          // Limpio los flags del contador
             TIM2->CR1 |= 0x0001;   // CEN = 1 -> Arranco el contador

             NVIC->ISER[0] |= (1 << 28);  // Habilita la IRQ del TIM 2 en el NVIC (posición 28)



             //CONFIGURACIÓN DEL TIM3-CH2 y3
             TIM3->CR1 = 0x0000;
			 TIM3->CR2 = 0x0000;
			 TIM3->SMCR = 0x0000;      // Siempre "0" en este curso


			 TIM3->PSC = 31999;
			 TIM3->CNT = 0;          // Inicializo el valor del contador a cero
			 TIM3->ARR = 0xFFFF;     // Valor recomendado si no es PWM
			 TIM3->CCR2 = 500;

		     // Selección de IRQ o no: DIER
			 TIM3->DIER = 0x0004;

			 TIM3->CCMR1 = 0x0000;
			 TIM3->CCER = 0x0000;

			 // Habilitación de contador y limpieza de flags
			 TIM3->EGR |= 0x0001;   // UG = 1 -> Se genera evento de actualización
			 TIM3->SR = 0;          // Limpio los flags del contador
			 TIM3->CR1 |= 0x0001;   // CEN = 1 -> Arranco el contador

			 NVIC->ISER[0] |= (1 << 29);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  valor = ADC1->DR;

	 	  //ADC

	 	  	  if (valor <1024){
	 	  		   adc_on = 1;
	 	  	  }
	 	  		  else {
	 	  		   adc_on = 0;
	 	  	  }




//Estructura código


//Menú que solo se imprime al principio para que el usuario conozca las opciones
			if (inicio == 1){

				inicio = 0;

 		        uint8_t start[] = "Selecione una opción:\r\n ";
 		        HAL_UART_Transmit(&huart1, start, sizeof(start), 1000);

 		        uint8_t start1[] = "Selecione 1 para ir hacia adelante\r\n ";
 		        HAL_UART_Transmit(&huart1, start1, sizeof(start1), 1000);

 		        uint8_t start2[] = "Selecione 3 para ir hacia atrás\r\n ";
 		        HAL_UART_Transmit(&huart1, start2, sizeof(start2), 1000);

 		        uint8_t start3[] = "Selecione 4 para activar el modo automático\r\n ";
 		        HAL_UART_Transmit(&huart1, start3, sizeof(start3), 1000);

 		        uint8_t start4[] = "Selecione 5 para ir a la izquierda\r\n ";
 		        HAL_UART_Transmit(&huart1, start4, sizeof(start4), 1000);

 		        uint8_t start5[] = "Selecione 6 para ir a la derecha\r\n ";
 		        HAL_UART_Transmit(&huart1, start5, sizeof(start5), 1000);

 		        uint8_t start6[] = "--------------------------------------------\r\n ";
 		        HAL_UART_Transmit(&huart1, start6, sizeof(start6), 1000);




			}


	 	  	  if (texto[0] == 0x31){

	 	  		  if(control_adelante == 1){
	 	  			control_adelante = 0;
	 		        uint8_t tx_data[] = "Modo 0 activado adelante \r\n";
	 		        HAL_UART_Transmit(&huart1, tx_data, sizeof(tx_data), 1000);
	 	  		  }

	  		 			pasos_giro= TIM3->CNT;
	  		 			pasos_suma = pasos_giro + time2;
	  		 			while (TIM3->CNT < pasos_suma){

								uint8_t bit_value; // Variable para almacenar el valor del pin PC12
								bit_value = (GPIOC->IDR & (1<<12)) >> 12;

								if (bit_value == 1){

									bit_value = (GPIOC->IDR & (1<<12)) >> 12;

									if (bit_value == 0){
										bit_value = (GPIOC->IDR & (1<<12)) >> 12;
										contador++;

									}

								}
								bit_value = (GPIOC->IDR & (1<<12)) >> 12;
								if(bit_value == 0){
									bit_value = (GPIOC->IDR & (1<<12)) >> 12;
									if (bit_value == 1){

									contador++;
								}
								}
								bit_value = (GPIOC->IDR & (1<<12)) >> 12;
								velocidad = contador;
							}

	  		 			contador=0;

	  		 			unsigned char pruebas[6];

	  		 			Bin2Ascii(velocidad, pruebas);
	  		 			HAL_UART_Transmit(&huart1, pruebas, sizeof(pruebas), 1000);

	  	 		        uint8_t start6[] = "\r\n ";
	  	 		        HAL_UART_Transmit(&huart1, start6, sizeof(start6), 1000);


	  	 		     if (pruebas[5] == '0' && pruebas[4] == '0'){

	  	 		     								uint8_t speed1[] = " 0 cm/s \r\n" ;
	  	 		     								HAL_UART_Transmit(&huart1, speed1, sizeof(speed1), 1000);
	  	 		     							}
	  	 		     							else if (pruebas[5] >='1' && pruebas[5] <= '6' && pruebas[4] == '0' ){
	  	 		     								uint8_t speed2[] = " 4 cm/s \r\n";
	  	 		     								 HAL_UART_Transmit(&huart1, speed2, sizeof(speed2), 1000);
	  	 		     							}
	  	 		     							else if (pruebas[5] >='7' && pruebas[5] <= '9' && pruebas[4] == '0' ){
	  	 		     								uint8_t speed3[] = " 6 cm/s \r\n";
	  	 		     								HAL_UART_Transmit(&huart1, speed3, sizeof(speed3), 1000);
	  	 		     							}

	  	 		     							else if (pruebas[4] >='1' && pruebas[5] >= '0'){

	  	 		     								uint8_t speed4[] = " 8 cm/s \r\n";
	  	 		     								HAL_UART_Transmit(&huart1, speed4, sizeof(speed4), 1000);

	  	 		     							}




					control_atras = 1;
					control_izq = 1;
					control_derecha = 1;
					control_automatico = 1;


					if(adc_on == 1){

		 				DC=8;
		 				TIM4->CCR3 = DC;

		 				DC_2=8;
		 				TIM4->CCR4 = DC_2;
						GPIOA->BSRR = (1<<12)<<16;
						GPIOA->BSRR = (1<<11)<<16;

						GPIOA->BSRR = (1<<1);

					}

					else if (adc_on == 0){

		 				DC=4;
		 				TIM4->CCR3 = DC;

		 				DC_2=4;
		 				TIM4->CCR4 = DC_2;
						GPIOA->BSRR = (1<<12)<<16;
						GPIOA->BSRR = (1<<11)<<16;

						GPIOA->BSRR = (1<<1);

					}

					modo_automatico = 0;



	 	  	  }

	 	  	  else if(texto[0] == 0x36){


	 	  		  if(control_izq == 1){
	 	  			control_izq = 0;
	 		        uint8_t tx_2[] = "Modo 1 activado izq\r\n";
	 		        HAL_UART_Transmit(&huart1, tx_2, sizeof(tx_2), 1000);
	 	  		  }



					control_atras = 1;
					control_adelante = 1;
					control_derecha = 1;
					control_automatico = 1;

					DC=0;
					TIM4->CCR3 = DC;

					DC_2=7;
					TIM4->CCR4 = DC_2;
					GPIOA->BSRR = (1<<12)<<16;
					GPIOA->BSRR = (1<<11)<<16;
					GPIOA->BSRR = (1<<1);
					modo_automatico = 0;

	 	  	  }

	 	  	  else if (texto[0] == 0x35){

	 	  		  if(control_derecha == 1){
	 	  			control_derecha = 0;
	 		        uint8_t tx_3[] = "Modo 2 activado derecha\r\n";
	 		        HAL_UART_Transmit(&huart1, tx_3, sizeof(tx_3), 1000);
	 	  		  }

					control_atras = 1;
					control_adelante = 1;
					control_izq = 1;
					control_automatico = 1;

					DC=7;
					TIM4->CCR3 = DC;

					DC_2=0;
					TIM4->CCR4 = DC_2;
					GPIOA->BSRR = (1<<12)<<16;
					GPIOA->BSRR = (1<<11)<<16;
					GPIOA->BSRR = (1<<1);
					modo_automatico =0;


	 	  	  }

	 	  	  else if (texto[0] == 0x33){

	 	  		  if(control_atras == 1){
	 	  			control_atras = 0;
	 		        uint8_t tx_4[] = "Modo 3 activado atras\r\n";
	 		        HAL_UART_Transmit(&huart1, tx_4, sizeof(tx_4), 1000);
	 	  		  }

					control_derecha = 1;
					control_adelante = 1;
					control_izq = 1;
					control_automatico = 1;

					DC=0;
					TIM4->CCR3 = DC;

					DC_2=0;
					TIM4->CCR4 = DC_2;
					GPIOA->BSRR = (1<<12);
					GPIOA->BSRR = (1<<11);
					GPIOA->BSRR = (1<<1);
					modo_automatico =0;

	 	  	  }

	 	  	  else if(texto[0] == 0x34){

	 	  		  if(control_automatico == 1){
	 	  			control_automatico = 0;
	 		        uint8_t tx_5[] = "Modo 4 activado automatico\r\n";
	 		        HAL_UART_Transmit(&huart1, tx_5, sizeof(tx_5), 1000);
	 	  		  }

					control_atras = 1;
					control_adelante = 1;
					control_izq = 1;
					control_derecha = 1;


	 	  		  modo_automatico=1;

					//MIDO LA DISTANCIA


		 	  		if(modo_automatico == 1){


		 	  		 	  	  		 	 	medirDistancia();




		 	  		 	  	  		 	 					  if(distancia <= 10 && zona_de_obstaculos == 0 ){


		 	  		 	  	  		 	 						GPIOA->BSRR = (1<<1)<<16;


		 	  		 	  	  		 	 						DC=0;
		 	  		 	  	  		 	 						TIM4->CCR3 = DC;

		 	  		 	  	  		 	 						DC_2=0;
		 	  		 	  	  		 	 						TIM4->CCR4 = DC_2;

		 	  		 	  	  		 	 						zona_de_obstaculos = 1; //Entro en la zona de obstaculos

		 	  		 	  	  		 	 						}


		 	  		 	  	  		 	 						else if(distancia >= 10 && distancia <= 20 && zona_de_obstaculos == 0) {
		 	  		 	  	  		 	 							DC=5;
		 	  		 	  	  		 	 							DC_2=5;
		 	  		 	  	  		 	 							TIM4->CCR3 = DC;
		 	  		 	  	  		 	 							TIM4->CCR4 = DC_2;

		 	  		 	  	  		 	 								if (adc_on == 1){
		 	  		 	  	  		 	 									DC = 3;
		 	  		 	  	  		 	 									TIM4->CCR3 = DC;
		 	  		 	  	  		 	 									DC_2 = 3;
		 	  		 	  	  		 	 									TIM4->CCR4 = DC_2;
		 	  		 	  	  		 	 								}

		 	  		 	  	  		 	 								if (estado_zumbador == 1){

		 	  		 	  	  		 	 									GPIOA->BSRR = (1<<1);
		 	  		 	  	  		 	 								}

		 	  		 	  	  		 	 								else{

		 	  		 	  	  		 	 									GPIOA->BSRR = (1<<1)<<16;
		 	  		 	  	  		 	 								}

		 	  		 	  	  		 	 							}


		 	  		 	  	  		 	 						else if(distancia > 20 && zona_de_obstaculos == 0){

		 	  		 	  	  		 	 							GPIOA->BSRR = (1<<1);
		 	  		 	  	  		 	 							DC=7;
		 	  		 	  	  		 	 							TIM4->CCR3 = DC;

		 	  		 	  	  		 	 							DC_2=7;
		 	  		 	  	  		 	 							TIM4->CCR4 = DC_2;

		 	  		 	  	  		 	 								if (adc_on == 1){
		 	  		 	  	  		 	 									DC = 3;
		 	  		 	  	  		 	 									TIM4->CCR3 = DC;
		 	  		 	  	  		 	 									DC_2 = 3;
		 	  		 	  	  		 	 									TIM4->CCR4 = DC_2;
		 	  		 	  	  		 	 								}
		 	  		 	  	  		 	 						}




		 	  		 	  	  		 //ZONA DE OBSTACULOS

		 	  		 	  	  		 	  	if (zona_de_obstaculos == 1 ){


		 	  		 	  	  		 			//ESPERA
		 	  		 	  	  		 			pasos_giro= TIM3->CNT;
		 	  		 	  	  		 			pasos_suma = pasos_giro + time2;
		 	  		 	  	  		 			while (TIM3->CNT < pasos_suma){
		 	  		 	  	  		 				TIM4->CCR3 = 0;
		 	  		 	  	  		 				TIM4->CCR4 = 0;
		 	  		 	  	  		 			}

		 	  		 	  	  		 			//MIDO
		 	  		 	  	  		 			medirDistancia();

		 	  		 	  	  		 			//MUEVO UN LADO
		 	  		 	  	  		 			if (distancia<10){

		 	  		 	  	  		 				pasos_giro= TIM3->CNT;
		 	  		 	  	  		 				pasos_suma = pasos_giro + time2;
		 	  		 	  	  		 				while (TIM3->CNT < pasos_suma){
		 	  		 	  	  		 				DC=5;
		 	  		 	  	  		 				DC_2=0;
		 	  		 	  	  		 				TIM4->CCR3 = DC;
		 	  		 	  	  		 				TIM4->CCR4 = DC_2;

		 	  		 	  	  		 					if (adc_on == 1){
		 	  		 	  	  		 						DC = 3;
		 	  		 	  	  		 						DC_2 = 0;
		 	  		 	  	  		 						TIM4->CCR3 = DC;
		 	  		 	  	  		 						TIM4->CCR4 = DC_2;
		 	  		 	  	  		 					}
		 	  		 	  	  		 				}

		 	  		 	  	  		 			}

		 	  		 	  	  		 			else if (distancia>10){
		 	  		 	  	  		 			zona_de_obstaculos = 0;
		 	  		 	  	  		 			}


		 	  		 	  	  		 			//ESPERA
		 	  		 	  	  		 			pasos_giro= TIM3->CNT;
		 	  		 	  	  		 			pasos_suma = pasos_giro + time2;
		 	  		 	  	  		 			while (TIM3->CNT < pasos_suma){
		 	  		 	  	  		 			DC=0;
		 	  		 	  	  		 			DC_2=0;
		 	  		 	  	  		 			TIM4->CCR3 = 0;
		 	  		 	  	  		 			TIM4->CCR4 = DC_2;
		 	  		 	  	  		 			}

		 	  		 	  	  		 			//MIDO
		 	  		 	  	  		 			medirDistancia();

		 	  		 	  	  		 			//MUEVO OTRO LADO
		 	  		 	  	  		 			if (distancia<10){

		 	  		 	  	  		 			pasos_giro= TIM3->CNT;
		 	  		 	  	  		 			pasos_suma = pasos_giro + 2*time2;
		 	  		 	  	  		 				while (TIM3->CNT < pasos_suma){
		 	  		 	  	  		 				DC=0;
		 	  		 	  	  		 				DC_2=5;
		 	  		 	  	  		 				TIM4->CCR3 = DC;
		 	  		 	  	  		 				TIM4->CCR4 = DC_2;

		 	  		 	  	  		 					if (adc_on == 1){
		 	  		 	  	  		 					DC = 0;
		 	  		 	  	  		 					DC_2 = 3;
		 	  		 	  	  		 					TIM4->CCR3 = DC;
		 	  		 	  	  		 					TIM4->CCR4 = DC_2;

		 	  		 	  	  		 					}
		 	  		 	  	  		 				}
		 	  		 	  	  		 			}

		 	  		 	  	  		 			else if (distancia>10){
		 	  		 	  	  		 			zona_de_obstaculos = 0;
		 	  		 	  	  		 			}


		 	  		 	  	  		 			//ESPERA
		 	  		 	  	  		 			pasos_giro= TIM3->CNT;
		 	  		 	  	  		 			pasos_suma = pasos_giro + time2;
		 	  		 	  	  		 			while (TIM3->CNT < pasos_suma){
		 	  		 	  	  		 			DC=0;
		 	  		 	  	  		 			DC_2=0;
		 	  		 	  	  		 			TIM4->CCR3 = 0;
		 	  		 	  	  		 			TIM4->CCR4 = DC_2;
		 	  		 	  	  		 			}

		 	  		 	  	  		 			//MIDO
		 	  		 	  	  		 			medirDistancia();

		 	  		 	  	  		 			//MUEVO ATRAS
		 	  		 	  	  		 			if (distancia<10){
		 	  		 	  	  		 			pasos_giro= TIM3->CNT;
		 	  		 	  	  		 			pasos_suma = pasos_giro + time2;
		 	  		 	  	  		 				while (TIM3->CNT < pasos_suma){
		 	  		 	  	  		 				DC=0;
		 	  		 	  	  		 				DC_2=5;
		 	  		 	  	  		 				TIM4->CCR3 = DC;
		 	  		 	  	  		 				TIM4->CCR4 = DC_2;

		 	  		 	  	  		 					if (adc_on == 1){
		 	  		 	  	  		 						DC = 0;
		 	  		 	  	  		 						TIM4->CCR3 = DC;
		 	  		 	  	  		 						DC_2 = 3;
		 	  		 	  	  		 						TIM4->CCR4 = DC_2;
		 	  		 	  	  		 					}
		 	  		 	  	  		 				}
		 	  		 	  	  		 			}

		 	  		 	  	  		 			else if (distancia>10){
		 	  		 	  	  		 			zona_de_obstaculos = 0;
		 	  		 	  	  		 			}

		 	  		 	  	  		 			//Dado que no es posible otra alternativa salimos de la zona de obstáculos
		 	  		 	  	  		 			zona_de_obstaculos = 0;


		 	  		 	  	  		 	  	}

		 	  	}



	 	  	  }

	 	  	  else{

	 	  	  		modo_automatico = 0;
					DC=0;
					TIM4->CCR3 = DC;

					DC_2=0;
					TIM4->CCR4 = DC_2;
					GPIOA->BSRR = (1<<12)<<16;
					GPIOA->BSRR = (1<<11)<<16;
					GPIOA->BSRR = (1<<1);


	 	  	  }




    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TS Initialization Function
  * @param None
  * @retval None
  */
static void MX_TS_Init(void)
{

  /* USER CODE BEGIN TS_Init 0 */

  /* USER CODE END TS_Init 0 */

  /* USER CODE BEGIN TS_Init 1 */

  /* USER CODE END TS_Init 1 */
  /* USER CODE BEGIN TS_Init 2 */

  /* USER CODE END TS_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Zumbador_GPIO_Port, Zumbador_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SEG14_Pin SEG15_Pin SEG16_Pin SEG17_Pin
                           SEG18_Pin SEG19_Pin SEG20_Pin SEG21_Pin
                           SEG22_Pin SEG23_Pin */
  GPIO_InitStruct.Pin = SEG14_Pin|SEG15_Pin|SEG16_Pin|SEG17_Pin
                          |SEG18_Pin|SEG19_Pin|SEG20_Pin|SEG21_Pin
                          |SEG22_Pin|SEG23_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Zumbador_Pin */
  GPIO_InitStruct.Pin = Zumbador_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Zumbador_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG1_Pin SEG2_Pin COM0_Pin COM1_Pin
                           COM2_Pin SEG12_Pin */
  GPIO_InitStruct.Pin = SEG1_Pin|SEG2_Pin|COM0_Pin|COM1_Pin
                          |COM2_Pin|SEG12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG6_Pin SEG7_Pin SEG8_Pin SEG9_Pin
                           SEG10_Pin SEG11_Pin SEG3_Pin SEG4_Pin
                           SEG5_Pin */
  GPIO_InitStruct.Pin = SEG6_Pin|SEG7_Pin|SEG8_Pin|SEG9_Pin
                          |SEG10_Pin|SEG11_Pin|SEG3_Pin|SEG4_Pin
                          |SEG5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Trigger_Pin */
  GPIO_InitStruct.Pin = Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Trigger_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(huart, texto, 1); // Vuelve a activar Rx por haber acabado
 // el buffer
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
