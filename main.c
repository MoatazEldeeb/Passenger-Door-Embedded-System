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

void PortA_Init(void);
void PortB_Init(void);

//define a Semaphore handle
xSemaphoreHandle driverSemaphore;
xSemaphoreHandle passengerSemaphore;
xSemaphoreHandle obstacleSemaphore;
xSemaphoreHandle permissionSemaphore;
xSemaphoreHandle limitSemaphore;
xSemaphoreHandle PassengerLockedMutex;

xQueueHandle xQueue;
xQueueHandle limitQueue;

portBASE_TYPE xstatus;
uint8_t PassengerLocked = 0;

char user;

// Port B(2,3) : down and up for driver
void driver(void *pvParameters){
	char ptr;
	uint32_t SW;
	
	xSemaphoreTake(driverSemaphore,0);
	for(;;)
	{
			xSemaphoreTake(driverSemaphore,portMAX_DELAY);  
			xQueueReceive(xQueue,&ptr,0);
			
			if(ptr =='u'){
				
				SW = GET_BIT(GPIOB->DATA,3);
				
				//For Button Debounce
				if(SW == 0){
					vTaskDelay(50);
					SW = GET_BIT(GPIOB->DATA,3);
					if(SW==0){
						move_window_up();
						user = 'd';
					}
				}
				
				vTaskDelay(1000);
				SW = GET_BIT(GPIOB->DATA,3);
				
				if(SW == 0){
					while(SW==0){
					SW = GET_BIT(GPIOB->DATA,3);
				}
		
				stop_motor();
				user = 'd';
			
				}else{
					
				}
				
				
				
			
			}
				else if(ptr =='d'){
					
					SW = GET_BIT(GPIOB->DATA,2);
				//For Button Debounce
				if(SW == 0){
					vTaskDelay(50);
					SW = GET_BIT(GPIOB->DATA,2);
					if(SW==0 ){
						move_window_down();
						user = 'd';
					}
				}
				vTaskDelay(1000);
				SW = GET_BIT(GPIOB->DATA,2);
				
				if(SW == 0){
					while(SW==0){
					SW = GET_BIT(GPIOB->DATA,2);
				}
		
				stop_motor();
				user = 'd';
				}
				}
					
				}
			
		}


void obstacle(void *pvParameters){
	uint32_t SW;
	xSemaphoreTake(obstacleSemaphore,0);
	for(;;)
	{
			xSemaphoreTake(obstacleSemaphore,portMAX_DELAY);  
		
		// Delay to make sure that obstacle button is pressed
		vTaskDelay(100);
		SW = GET_BIT(GPIOB->DATA,4);
		if(SW ==0){
			
					
			if(window_state() == 'u'){
				move_window_down();
				user = 'i';
				vTaskDelay(1000);
				stop_motor();
			}
			SW = GET_BIT(GPIOB->DATA,4);
			while(SW==0){
				SW = GET_BIT(GPIOB->DATA,4);
			}
		}
	}
}

void changePermission(void *pvParameters){
	uint32_t SW;
	xSemaphoreTake(permissionSemaphore,0);
	for(;;)
	{
			xSemaphoreTake(permissionSemaphore,portMAX_DELAY); 
			xSemaphoreTake(PassengerLockedMutex, portMAX_DELAY);
			PassengerLocked = GET_BIT(GPIOB->DATA,5);
			if (PassengerLocked == 1 & user== 'p'){
				stop_motor();
			}
			
			xSemaphoreGive(PassengerLockedMutex);
	}
}

void limit(void *pvParameters){
	uint32_t SW;
	char ptr;
	xSemaphoreTake(limitSemaphore,0);
	for(;;)
	{
			
			xSemaphoreTake(limitSemaphore,portMAX_DELAY); 
			xQueueReceive(limitQueue,&ptr,0);
			if(ptr =='u'){
				if(window_state() == 'u'){
					stop_motor();
					user = 'l';
					SW = GET_BIT(GPIOB->DATA , 6);
					while(SW == 0){
						SW = GET_BIT(GPIOB->DATA , 6);
					}
				}
				
			}else if (ptr == 'd'){
				if(window_state() == 'd'){
					stop_motor();
					user = 'l';
					SW = GET_BIT(GPIOB->DATA , 6);
					while(SW == 0){
						SW = GET_BIT(GPIOB->DATA , 6);
					}
					
				}
			}
	}
}

// (0,1) : down and up for passenger
void passenger(void *pvParameters){
	char ptr;
	uint32_t SW;
	
	xSemaphoreTake(passengerSemaphore,0);
	for(;;)
	{
			xSemaphoreTake(passengerSemaphore,portMAX_DELAY);  
			xQueueReceive(xQueue,&ptr,0);
			if (PassengerLocked==0){
			if(ptr =='u'){
				
				SW = GET_BIT(GPIOB->DATA,1);
				
				//For Button Debounce
				if(SW == 0& PassengerLocked==0){
					vTaskDelay(50);
					SW = GET_BIT(GPIOB->DATA,1);
					if(SW==0& PassengerLocked==0){
						move_window_up();
						user = 'p';
					}
				}
				vTaskDelay(1000);
				SW = GET_BIT(GPIOB->DATA,1);
				
				
				if(SW == 0 & PassengerLocked==0){
					// Added a mutex to handle the case while pressing button and the passenger lock is locked stop reading
					xSemaphoreTake(PassengerLockedMutex, portMAX_DELAY);
					while(SW==0 & PassengerLocked==0){
						xSemaphoreGive(PassengerLockedMutex);
						SW = GET_BIT(GPIOB->DATA,1);
						xSemaphoreTake(PassengerLockedMutex, portMAX_DELAY);
				}
					xSemaphoreGive(PassengerLockedMutex);
		
				stop_motor();
				user = 'p';
			}
				
			
			}
				else if(ptr =='d'){
					SW = GET_BIT(GPIOB->DATA,0);
				//For Button Debounce
				if(SW == 0& PassengerLocked==0 ){
					vTaskDelay(50);
					SW = GET_BIT(GPIOB->DATA,0);
					if(SW==0& PassengerLocked==0){
						move_window_down();
						user = 'p';
					}
				}
				vTaskDelay(1000);
				SW = GET_BIT(GPIOB->DATA,0);
				
				if(SW == 0 & PassengerLocked==0 ){ //pressed , not locked , limit mesh mtdas 
					xSemaphoreTake(PassengerLockedMutex, portMAX_DELAY);
					while(SW==0 & PassengerLocked==0 ){
					xSemaphoreGive(PassengerLockedMutex);
					SW = GET_BIT(GPIOB->DATA,0);
					xSemaphoreTake(PassengerLockedMutex, portMAX_DELAY);
				}
					xSemaphoreGive(PassengerLockedMutex);
		
				stop_motor();
				user = 'p';
				}
			}
				
				
				}
					
		}
}

                         /*main function*/
/*------------------------------------------------------------------------*/
int main( void )
{
		PortA_Init();
		PortB_Init();
		__ASM("CPSIE i");
		PassengerLocked = GET_BIT(GPIOB->DATA,5);
		vSemaphoreCreateBinary(driverSemaphore);
		vSemaphoreCreateBinary(passengerSemaphore);
		vSemaphoreCreateBinary(obstacleSemaphore);
		vSemaphoreCreateBinary(limitSemaphore);
		
		vSemaphoreCreateBinary(permissionSemaphore);
		PassengerLockedMutex = xSemaphoreCreateMutex();

		limitQueue=xQueueCreate(2,sizeof(char));
		xQueue=xQueueCreate(2,sizeof(char));
	
	if( driverSemaphore != NULL )
		{
			xTaskCreate( passenger, "passenger", 240, NULL, 2, NULL );
			xTaskCreate( driver, "driver", 240, NULL, 3, NULL );
			xTaskCreate( changePermission, "ChangePermission", 240, NULL, 4, NULL );
			xTaskCreate( obstacle, "obstacle", 240, NULL, 6, NULL );
			xTaskCreate( limit, "limit", 240, NULL, 5, NULL );


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


// OUTPUTS:  port A pins(4,5,)
//	(4,5) : Motor Pins
void PortA_Init(void){ 
  SYSCTL->RCGCGPIO |= 0x00000001;    // 1) A clock
  GPIOA->LOCK = 0x4C4F434B;  				 // 2) unlock PortA PA0  
  GPIOA->CR = 0x30;          				 // allow changes to PA4-0       
  GPIOA->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOA->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL  
  GPIOA->DIR = 0x30;         				 // 5) 4 OUTPUT , 4 INPUT  
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
// (6,7): Limit down and up
void PortB_Init(void){ 
  SYSCTL->RCGCGPIO |= 0x00000002;    // 1) A clock
  GPIOB->LOCK = 0x4C4F434B;  				 // 2) unlock PortA PA0  
  GPIOB->CR = 0xFF;          				 // allow changes to PA4-0       
  GPIOB->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOB->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL  
  GPIOB->DIR = 0x00;         				 // 5) 4 OUTPUT , 4 INPUT  
  GPIOB->AFSEL = 0x00;      				 // 6) no alternate function
  GPIOB->PUR = (0xFF);       				   // enable pullup resistors on first 4 pins   
  GPIOB->DEN = 0xFF;       				   // 7) enable digital pins PF4-PF0
	GPIOB->DATA = 0x00;
	
	// Setup the interrupt on PortA
	GPIOB->ICR = 0xFF;     // Clear any Previous Interrupt 
	GPIOB->IM = 0xFF;      // Unmask the interrupts for PF0 and PF4
	GPIOB->IS &= ~(1<<7)|~(1<<6)|~(1<<5)|~(1<<4)|~(1<<3)|~(1<<2)|~(1<<1)|~(1<<0);    // Make bits PF0 and PF4 level sensitive
	GPIOB->IBE |= (1<<5) ;  // 
//	GPIOB->IBE &=~(1<<5)|~(1<<4)|~(1<<3)|~(1<<2)|~(1<<1)|~(1<<0);     /* trigger is controlled by IEV */
//	GPIOB->IEV &= ~(1<<5)|~(1<<4)|~(1<<3)|~(1<<2)|~(1<<1)|~(1<<0);      /* falling edge trigger */

	NVIC_EnableIRQ(PortB_IRQn);        // Enable the Interrupt for PortF in NVIC
}


/*------------------------------------------------------------------------*/

void GPIOB_Handler(void){
	uint32_t i;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	char s;
	
		//BIT 0
		if((GPIOB->MIS & 0x01)){
				s = 'd';
				xstatus = xQueueSendFromISR(xQueue,&s,&xHigherPriorityTaskWoken);
				xSemaphoreGiveFromISR(passengerSemaphore,&xHigherPriorityTaskWoken);
				SET_BIT(GPIOB->ICR,0);        // clear the interrupt flag of PORTF
				i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
		}
	//BIT 1
	 else if((GPIOB->MIS & 0x02)){
			s = 'u';
			xstatus = xQueueSendFromISR(xQueue,&s,&xHigherPriorityTaskWoken);
		  xSemaphoreGiveFromISR(passengerSemaphore,&xHigherPriorityTaskWoken);
			SET_BIT(GPIOB->ICR,1);        // clear the interrupt flag of PORTF
			i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
	
		 
	}
	 //BIT 2
	 else if((GPIOB->MIS & 0x04)){
			s = 'd';
			xstatus = xQueueSendFromISR(xQueue,&s,&xHigherPriorityTaskWoken);
		  xSemaphoreGiveFromISR(driverSemaphore,&xHigherPriorityTaskWoken);
			SET_BIT(GPIOB->ICR,2);
			i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
	
	}
	 //BIT 3
	 else if((GPIOB->MIS & 0x08)){
			s = 'u';
			xstatus = xQueueSendFromISR(xQueue,&s,&xHigherPriorityTaskWoken);
		  xSemaphoreGiveFromISR(driverSemaphore,&xHigherPriorityTaskWoken);
			SET_BIT(GPIOB->ICR,3);        // clear the interrupt flag of PORTF
			i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
	
	}
	 //BIT 4
	 else if((GPIOB->MIS & 0x10)){
		  xSemaphoreGiveFromISR(obstacleSemaphore,&xHigherPriorityTaskWoken);
			SET_BIT(GPIOB->ICR,4);        // clear the interrupt flag of PORTF
			i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
	
		 
	}
	 //BIT 5
	 else if((GPIOB->MIS & 0x20)){
		  xSemaphoreGiveFromISR(permissionSemaphore,&xHigherPriorityTaskWoken);
			SET_BIT(GPIOB->ICR,5);        // clear the interrupt flag of PORTF
			i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
	}
	 //BIT 6
	 else if((GPIOB->MIS & 0x40)){
			s= 'd';
			xstatus = xQueueSendFromISR(limitQueue,&s,&xHigherPriorityTaskWoken);
		  xSemaphoreGiveFromISR(limitSemaphore,&xHigherPriorityTaskWoken);
		 
			SET_BIT(GPIOB->ICR,6);        // clear the interrupt flag of PORTF
			i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
	}
	 //BIT 7
	 else if((GPIOB->MIS & 0x80)){
			s = 'u';
			xstatus = xQueueSendFromISR(limitQueue,&s,&xHigherPriorityTaskWoken);
		  xSemaphoreGiveFromISR(limitSemaphore,&xHigherPriorityTaskWoken);
			SET_BIT(GPIOB->ICR,7);        // clear the interrupt flag of PORTF
			i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
	}
	
	 
	 
	 
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}