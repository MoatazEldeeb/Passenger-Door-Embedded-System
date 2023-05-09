#include <stdint.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "motor_driver.h"
#include "tm4c123gh6pm.h"
#define PortF_IRQn 30
#define PortA_IRQn 0
#define PortB_IRQn 1

void PortF_Init(void);
void PortA_Init(void);
void PortB_Init(void);

//define a Semaphore handle
xSemaphoreHandle driverSemaphore;
xSemaphoreHandle passengerSemaphore;
xSemaphoreHandle PermissionSemaphore;
xSemaphoreHandle obstacleSemaphore;
xSemaphoreHandle xCountingSemaphore;

xQueueHandle xQueue;
portBASE_TYPE xstatus;
int PassengerLocked=0;

//this Task "Handler" is awakened when the semaphore is available
//void Handler(void *pvParameters){
	
//	xSemaphoreTake(xBinarySemaphore,0);
//	for(;;)
//	{
//			xSemaphoreTake(xBinarySemaphore,portMAX_DELAY);  
			
		
//			//Toggle the red led
//			GPIOF->DATA ^= 0x02;
//	}
//}


// Port B(2,3) : down and up for driver
void driver(void *pvParameters){
	char ptr;
	uint32_t SW;
	uint8_t locked;
	
	xSemaphoreTake(driverSemaphore,0);
	for(;;)
	{
			xSemaphoreTake(driverSemaphore,portMAX_DELAY);  
			xQueueReceive(xQueue,&ptr,0);
			if(ptr =='u'){
				
				SW = GET_BIT(GPIOB->DATA,3);
				
				if(SW == 0){
					move_window_up();
				}
				Delay_ms(500);
				SW = GET_BIT(GPIOB->DATA,3);
				
				if(SW == 0){
					while(SW==0){
					SW = GET_BIT(GPIOB->DATA,3);
				}
		
				stop_motor();
				}
				
				
				
			
			}
				else if(ptr =='d'){
					SW = GET_BIT(GPIOB->DATA,2);
				
				if(SW == 0){
					move_window_down();
				}
				Delay_ms(500);
				SW = GET_BIT(GPIOB->DATA,2);
				
				if(SW == 0){
					while(SW==0){
					SW = GET_BIT(GPIOB->DATA,2);
				}
		
				stop_motor();
				}
				
					
				
				
				}
					
				}
			
		}


void obsctacle(void *pvParameters){
	
	xSemaphoreTake(obstacleSemaphore,0);
	for(;;)
	{
			xSemaphoreTake(obstacleSemaphore,portMAX_DELAY);  
			//stop motor and move in counter direction
			//Delay 0.5s
		if(window_state() == 'u'){
			stop_motor();
			move_window_down();
			Delay_ms(500);
			stop_motor();
		}
	}
}

// (0,1) : down and up for passenger
void passenger(void *pvParameters){
	char ptr;
	uint32_t SW;
	uint8_t locked;
	
	xSemaphoreTake(passengerSemaphore,0);
	for(;;)
	{
			xSemaphoreTake(passengerSemaphore,portMAX_DELAY);  
			xQueueReceive(xQueue,&ptr,0);
			stop_motor();
			locked = GET_BIT(GPIOB->DATA,5);
			if (locked==0){
			if(ptr =='u'){
				
				SW = GET_BIT(GPIOB->DATA,1);
				
				if(SW == 0){
					move_window_up();
				}
					while(SW==0){
					SW = GET_BIT(GPIOB->DATA,1);
				}
		
				stop_motor();
				
			
			}
				else if(ptr =='d'){
					SW = GET_BIT(GPIOB->DATA,0);
				
				if(SW == 0){
					move_window_down();
				}
					while(SW==0){
					SW = GET_BIT(GPIOB->DATA,0);
				}
		
				stop_motor();
				}
				
				
				}
					
				}
			}
		


	

void vTestTask1(void *pvParameters){	
	uint32_t SW;
	for(;;){
		SW = GET_BIT(GPIOF->DATA,0);
		
		if(SW == 0){
			SET_BIT(GPIOF->DATA,3);
		}
		
		while(SW==0){
			SW = GET_BIT(GPIOF->DATA,0);
		}
		
		CLEAR_BIT(GPIOF->DATA,3);
		
	}
}

//This Periodic task is preempted by the task "Handler"
void vTestTask(void *pvParameters){	
	uint32_t SW;
	xSemaphoreTake(xCountingSemaphore,0);
	
	for(;;){
		xSemaphoreTake(xCountingSemaphore,portMAX_DELAY);
		
		SW = GET_BIT(GPIOF->DATA,0);
		
		if(SW == 0){
			SET_BIT(GPIOF->DATA,3);
		}
		
		while(SW == 0){
			SW = GET_BIT(GPIOF->DATA,0);
		}
		
		CLEAR_BIT(GPIOF->DATA,3);
	}
}

void vContinousTask(void *pvParameters){	
	for(;;){
	}
}
                         /*main function*/
/*------------------------------------------------------------------------*/
int main( void )
{
    PortF_Init();
		PortA_Init();
		PortB_Init();
		__ASM("CPSIE i");
		
		vSemaphoreCreateBinary(driverSemaphore);
		vSemaphoreCreateBinary(passengerSemaphore);
		vSemaphoreCreateBinary(PermissionSemaphore);
		vSemaphoreCreateBinary(obstacleSemaphore);
		//xCountingSemaphore = xSemaphoreCreateCounting(3000,0);
		vSemaphoreCreateBinary(xCountingSemaphore);

	
		xQueue=xQueueCreate(2,sizeof(char));
		//xBinarySemaphore = xSemaphoreCreateBinary();
	if( xCountingSemaphore != NULL & PermissionSemaphore != NULL)
		{
			/* Create the 'handler' task. This is the task that will be synchronized
			with the interrupt. The handler task is created with a high priority to
			ensure it runs immediately after the interrupt exits. In this case a
			priority of 3 is chosen. */
			//xTaskCreate( Handler, "Handler", 240, NULL, 3, NULL );
			/* Create the task that will periodically generate a software interrupt.
			This is created with a priority below the handler task to ensure it will
			get preempted each time the handler task exits the Blocked state. */
			//xTaskCreate( vPeriodicTask, "Periodic", 240, NULL, 1, NULL );
			xTaskCreate( passenger, "passenger", 240, NULL, 3, NULL );
			xTaskCreate( driver, "driver", 240, NULL, 4, NULL );
			//xTaskCreate( ChangePermission, "ChangePermission", 240, NULL, 3, NULL );
			//xTaskCreate( obsctacle, "obstacle", 240, NULL, 4, NULL );
			
			//xTaskCreate( vTestTask, "vTestTask", 240, NULL, 5, NULL );
			
			//xTaskCreate( vTestTask, "vTestTask", 240, NULL, 5, NULL );
			//xTaskCreate( vContinousTask, "vContinousTask", 240, NULL, 1, NULL );


			/* Start the scheduler so the created tasks start executing. */
			vTaskStartScheduler();
		}

    /* If all is well we will never reach here as the scheduler will now be
    running the tasks.  If we do reach here then it is likely that there was
    insufficient heap memory available for a resource to be created. */
    for( ;; );
}


/*------------------------------------------------------------------------*/
//Initialize the hardware of Port-F4

void PortF_Init(void){ 
  SYSCTL->RCGCGPIO |= 0x00000020;    // 1) F clock
  GPIOF->LOCK = 0x4C4F434B;  				 // 2) unlock PortF PF0  
  GPIOF->CR = 0x1F;          				 // allow changes to PF4-0       
  GPIOF->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOF->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL  
  GPIOF->DIR = 0x0E;         				 // 5) PF4,PF0 input, PF3,PF2,PF1 output   
  GPIOF->AFSEL = 0x00;      				 // 6) no alternate function
  GPIOF->PUR = ~(0x0E);       				   // enable pullup resistors on PF4,PF0       
  GPIOF->DEN = 0xFF;       				   // 7) enable digital pins PF4-PF0
	GPIOF->DATA = 0x00;
	
	// Setup the interrupt on PortF
	GPIOF->ICR = 0x11;     // Clear any Previous Interrupt
	GPIOF->IM |=0x11;      // Unmask the interrupts for PF0 and PF4
	//GPIOF->IS |= 0x11;     // Make bits PF0 and PF4 level sensitive
	//GPIOF->IEV &= ~0x11;   // Sense on Low Level

	NVIC_EnableIRQ(PortF_IRQn);        // Enable the Interrupt for PortF in NVIC
}

// OUTPUTS:  port A pins(4,5,6,7)
//	(4,5) : Motor Pins
//
void PortA_Init(void){ 
  SYSCTL->RCGCGPIO |= 0x00000001;    // 1) A clock
  GPIOA->LOCK = 0x4C4F434B;  				 // 2) unlock PortA PA0  
  GPIOA->CR = 0xFF;          				 // allow changes to PA4-0       
  GPIOA->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOA->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL  
  GPIOA->DIR = 0x0F0;         				 // 5) 4 OUTPUT , 4 INPUT  
  GPIOA->AFSEL = 0x00;      				 // 6) no alternate function
  GPIOA->PUR = 0x00;       				   // enable pullup resistors on first 4 pins      
  GPIOA->DEN = 0xFF;       				   // 7) enable digital pins PF4-PF0
	GPIOA->DATA = 0x00;
	
}

// INPUTS:  port B pins(0,1,2,3,4,5) with interputs
// (0,1) : down and up for passenger
// (2,3) : down and up for driver
// (4) : obstacle
// (5) : lock Passenger
void PortB_Init(void){ 
  SYSCTL->RCGCGPIO |= 0x00000002;    // 1) A clock
  GPIOB->LOCK = 0x4C4F434B;  				 // 2) unlock PortA PA0  
  GPIOB->CR = 0xFF;          				 // allow changes to PA4-0       
  GPIOB->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOB->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL  
  GPIOB->DIR = 0x00;         				 // 5) 4 OUTPUT , 4 INPUT  
  GPIOB->AFSEL = 0x00;      				 // 6) no alternate function
  GPIOB->PUR = (0x3F);       				   // enable pullup resistors on first 4 pins   
//	GPIOB->PDR = (0xFF);
  GPIOB->DEN = 0x3F;       				   // 7) enable digital pins PF4-PF0
	GPIOB->DATA = 0x00;
	
	// Setup the interrupt on PortA
	GPIOB->ICR = 0x3F;     // Clear any Previous Interrupt 
	GPIOB->IM = 0x3F;      // Unmask the interrupts for PF0 and PF4
//	GPIOB->IS |= 0x3F;     // Make bits PF0 and PF4 level sensitive
//	GPIOB->IEV &= ~0x3F;   // Sense on Low Level

	NVIC_EnableIRQ(PortB_IRQn);        // Enable the Interrupt for PortF in NVIC
}


/*------------------------------------------------------------------------*/
//Port-F handler
void GPIOF_Handler(void){

	uint32_t i;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	char s;
	uint32_t SW;
	
	               //driver 
	//Give the semaphore to the Task named handler
	if((GPIOF->MIS & 0x01)){
			//s = 'u';
			//xstatus = xQueueSendFromISR(xQueue,&s,&xHigherPriorityTaskWoken);
		  xSemaphoreGiveFromISR(xCountingSemaphore,&xHigherPriorityTaskWoken);
			
			GPIOF->ICR |= 0x01;       // clear the interrupt flag of PORTF
			i= GPIOF->ICR ;           // Reading the register to force the flag to be cleared
	}
	 else if((GPIOF->MIS & 0x10)){
			//s = 'd';
			//xstatus = xQueueSendFromISR(xQueue,&s,&xHigherPriorityTaskWoken);
		  xSemaphoreGiveFromISR(PermissionSemaphore,&xHigherPriorityTaskWoken);
			GPIOF->ICR |= 0x10;       // clear the interrupt flag of PORTF
			i= GPIOF->ICR ;           // Reading the register to force the flag to be cleared
			
	}

	
	/* Giving the semaphore may have unblocked a task - if it did and the
	unblocked task has a priority equal to or above the currently executing
	task then xHigherPriorityTaskWoken will have been set to pdTRUE and
	portEND_SWITCHING_ISR() will force a context switch to the newly unblocked
	higher priority task.
	NOTE: The syntax for forcing a context switch within an ISR varies between
	FreeRTOS ports. The portEND_SWITCHING_ISR() macro is provided as part of
	the Corte M3 port layer for this purpose. taskYIELD() must never be called
	from an ISR! */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}


void GPIOB_Handler(void){
	uint32_t i;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	char s;
	
		//BIT 0
		if((GPIOB->MIS & 0x01)){
				s = 'd';
				xstatus = xQueueSendFromISR(xQueue,&s,&xHigherPriorityTaskWoken);
				xSemaphoreGiveFromISR(passengerSemaphore,&xHigherPriorityTaskWoken);
				Delay_ms(100);
				SET_BIT(GPIOB->ICR,0);        // clear the interrupt flag of PORTF
				i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
		}
	//BIT 1
	 else if((GPIOB->MIS & 0x02)){
			s = 'u';
			xstatus = xQueueSendFromISR(xQueue,&s,&xHigherPriorityTaskWoken);
		  xSemaphoreGiveFromISR(passengerSemaphore,&xHigherPriorityTaskWoken);
		 Delay_ms(100);
			SET_BIT(GPIOB->ICR,1);        // clear the interrupt flag of PORTF
			i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
	
		 
	}
	 //BIT 2
	 else if((GPIOB->MIS & 0x04)){
			s = 'd';
			xstatus = xQueueSendFromISR(xQueue,&s,&xHigherPriorityTaskWoken);
		  xSemaphoreGiveFromISR(driverSemaphore,&xHigherPriorityTaskWoken);
		 Delay_ms(100);
			SET_BIT(GPIOB->ICR,2);
			i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
	
	}
	 //BIT 3
	 else if((GPIOB->MIS & 0x08)){
			s = 'u';
			xstatus = xQueueSendFromISR(xQueue,&s,&xHigherPriorityTaskWoken);
		  xSemaphoreGiveFromISR(driverSemaphore,&xHigherPriorityTaskWoken);
		 Delay_ms(100);
			SET_BIT(GPIOB->ICR,3);        // clear the interrupt flag of PORTF
			i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
	
	}
	 //BIT 4
	 else if((GPIOB->MIS & 0x10)){
			s = 'd';
			xstatus = xQueueSendFromISR(xQueue,&s,&xHigherPriorityTaskWoken);
		  xSemaphoreGiveFromISR(PermissionSemaphore,&xHigherPriorityTaskWoken);
			SET_BIT(GPIOB->ICR,4);        // clear the interrupt flag of PORTF
			i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
	
		 
	}
	 //BIT 5
	 else if((GPIOB->MIS & 0x20)){
			s = 'd';
			xstatus = xQueueSendFromISR(xQueue,&s,&xHigherPriorityTaskWoken);
		  xSemaphoreGiveFromISR(obstacleSemaphore,&xHigherPriorityTaskWoken);
			SET_BIT(GPIOB->ICR,5);        // clear the interrupt flag of PORTF
			i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
	
	}
	 
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}


	
	

	