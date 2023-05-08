#include <stdint.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

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

void driver(void *pvParameters){
	char ptr;
	xSemaphoreTake(driverSemaphore,0);
	for(;;)
	{
			xSemaphoreTake(driverSemaphore,portMAX_DELAY);  
			xQueueReceive(xQueue,&ptr,0);
			if(ptr =='u'){
				GPIOF->DATA = 0x02;
			}
				else if(ptr =='d'){
					GPIOF->DATA = 0x04;
				}
			//Toggle the red led
			
		
	}
}

void passenger(void *pvParameters){
	char ptr;
	xSemaphoreTake(passengerSemaphore,0);
	for(;;)
	{
			xSemaphoreTake(passengerSemaphore,portMAX_DELAY);  
			xQueueReceive(xQueue,&ptr,0);
		if(PassengerLocked==0){
				if(ptr =='u'){
					GPIOF->DATA = 0x08;
				}
				else if(ptr =='d'){
					GPIOF->DATA = 0x04;
				}
				//Toggle the red led
		}
		
	}
}

void ChangePermission(void *pvParameters){
	
	xSemaphoreTake(PermissionSemaphore,0);
	for(;;)
	{
			xSemaphoreTake(PermissionSemaphore,portMAX_DELAY);  
		
			if (PassengerLocked==1 )
			{
				PassengerLocked=0;
			}
			else if(PassengerLocked==0){
				PassengerLocked=1;
				GPIOF->DATA = 0x02;
			}
				
		
	}
}
	


//This Periodic task is preempted by the task "Handler"
void vPeriodicTask(void *pvParameters){

	for(;;){
	
		GPIOF->DATA |= 0x08;
		vTaskDelay(500);
	}

}
                         /*main function*/
/*------------------------------------------------------------------------*/
int main( void )
{
    PortF_Init();
		__ASM("CPSIE i");
		
		vSemaphoreCreateBinary(driverSemaphore);
		vSemaphoreCreateBinary(passengerSemaphore);
		vSemaphoreCreateBinary(PermissionSemaphore);
	
		xQueue=xQueueCreate(2,sizeof(char));
		//xBinarySemaphore = xSemaphoreCreateBinary();
	if( passengerSemaphore != NULL & PermissionSemaphore != NULL)
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
			
			xTaskCreate( passenger, "passenger", 240, NULL, 1, NULL );
			xTaskCreate( ChangePermission, "ChangePermission", 240, NULL, 3, NULL );
			
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
  GPIOF->PUR = 0x11;       				   // enable pullup resistors on PF4,PF0       
  GPIOF->DEN = 0x1F;       				   // 7) enable digital pins PF4-PF0
	GPIOF->DATA = 0x00;
	
	// Setup the interrupt on PortF
	GPIOF->ICR = 0x11;     // Clear any Previous Interrupt 
	GPIOF->IM |=0x11;      // Unmask the interrupts for PF0 and PF4
	GPIOF->IS |= 0x11;     // Make bits PF0 and PF4 level sensitive
	GPIOF->IEV &= ~0x11;   // Sense on Low Level

	NVIC_EnableIRQ(PortF_IRQn);        // Enable the Interrupt for PortF in NVIC
}

// OUTPUTS:  port A pins(4,5,6,7)
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

// INPUTS:  port B pins(0,1,2,3,3,5) with interputs

void PortB_Init(void){ 
  SYSCTL->RCGCGPIO |= 0x00000002;    // 1) A clock
  GPIOB->LOCK = 0x4C4F434B;  				 // 2) unlock PortA PA0  
  GPIOB->CR = 0x3F;          				 // allow changes to PA4-0       
  GPIOB->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOB->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL  
  GPIOB->DIR = 0x00;         				 // 5) 4 OUTPUT , 4 INPUT  
  GPIOB->AFSEL = 0x00;      				 // 6) no alternate function
  GPIOB->PUR = 0x3F;       				   // enable pullup resistors on first 4 pins      
  GPIOB->DEN = 0x3F;       				   // 7) enable digital pins PF4-PF0
	GPIOB->DATA = 0x00;
	
	// Setup the interrupt on PortA
	GPIOB->ICR = 0x3F;     // Clear any Previous Interrupt 
	GPIOB->IM |=0x3F;      // Unmask the interrupts for PF0 and PF4
	GPIOB->IS |= 0x3F;     // Make bits PF0 and PF4 level sensitive
	GPIOB->IEV &= ~0x3F;   // Sense on Low Level

	NVIC_EnableIRQ(PortB_IRQn);        // Enable the Interrupt for PortF in NVIC
}


/*------------------------------------------------------------------------*/
//Port-F handler
void GPIOF_Handler(void){

	uint32_t i;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	char s;
	               //driver 
	//Give the semaphore to the Task named handler
	if((GPIOF->MIS & 0x01)){
			s = 'u';
			xstatus = xQueueSendFromISR(xQueue,&s,&xHigherPriorityTaskWoken);
		  xSemaphoreGiveFromISR(passengerSemaphore,&xHigherPriorityTaskWoken);
			GPIOF->ICR |= 0x01;        // clear the interrupt flag of PORTF
			i= GPIOF->ICR ;           // Reading the register to force the flag to be cleared
	}
	 else if((GPIOF->MIS & 0x10)){
			//s = 'd';
			//xstatus = xQueueSendFromISR(xQueue,&s,&xHigherPriorityTaskWoken);
		  xSemaphoreGiveFromISR(PermissionSemaphore,&xHigherPriorityTaskWoken);
			GPIOF->ICR |= 0x10;        // clear the interrupt flag of PORTF
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
	
	
	if((GPIOB->MIS & 0x01)==1){
			s = 'u';
			xstatus = xQueueSendFromISR(xQueue,&s,&xHigherPriorityTaskWoken);
		  xSemaphoreGiveFromISR(driverSemaphore,&xHigherPriorityTaskWoken);
			GPIOB->ICR |= 0x01;        // clear the interrupt flag of PORTF
			i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
	}
	 else if((GPIOB->MIS & 0x02)){
			s = 'd';
			xstatus = xQueueSendFromISR(xQueue,&s,&xHigherPriorityTaskWoken);
		  xSemaphoreGiveFromISR(driverSemaphore,&xHigherPriorityTaskWoken);
			GPIOB->ICR |= 0x10;        // clear the interrupt flag of PORTF
			i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
	
		 
	}
	 else if((GPIOB->MIS & 0x03)){
			s = 'd';
			xstatus = xQueueSendFromISR(xQueue,&s,&xHigherPriorityTaskWoken);
		  xSemaphoreGiveFromISR(driverSemaphore,&xHigherPriorityTaskWoken);
			GPIOB->ICR |= 0x100;        // clear the interrupt flag of PORTF
			i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
	
	}
	 else if((GPIOB->MIS & 0x04)){
			s = 'd';
			xstatus = xQueueSendFromISR(xQueue,&s,&xHigherPriorityTaskWoken);
		  xSemaphoreGiveFromISR(driverSemaphore,&xHigherPriorityTaskWoken);
			GPIOB->ICR |= 0x1000;        // clear the interrupt flag of PORTF
			i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
	
	}
	 else if((GPIOB->MIS & 0x05)){
			s = 'd';
			xstatus = xQueueSendFromISR(xQueue,&s,&xHigherPriorityTaskWoken);
		  xSemaphoreGiveFromISR(driverSemaphore,&xHigherPriorityTaskWoken);
			GPIOB->ICR |= 0x10000;        // clear the interrupt flag of PORTF
			i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
	
		 
	}
	 else if((GPIOB->MIS & 0x06)){
			s = 'd';
			xstatus = xQueueSendFromISR(xQueue,&s,&xHigherPriorityTaskWoken);
		  xSemaphoreGiveFromISR(driverSemaphore,&xHigherPriorityTaskWoken);
			GPIOB->ICR |= 0x100000;        // clear the interrupt flag of PORTF
			i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared
	
	}
	 
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}


	
	

	