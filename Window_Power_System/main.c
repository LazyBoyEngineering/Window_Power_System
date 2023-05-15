#include <stdint.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

/**********************************************************************************
*																DEFINITIONS																				*
***********************************************************************************/

#define PASS_PORT						GPIOB
#define UP_PB_P_AUTO       	0		//PORTB
#define UP_PB_P_MAN       	1		//PORTB
#define DOWN_PB_P_MAN 			4		//PORTB
#define DOWN_PB_P_AUTO			5		//PORTB

#define DRIVER_PORT					GPIOA
#define UP_PB_D_AUTO       	2		//PORTA
#define UP_PB_D_MAN       	3		//PORTA
#define DOWN_PB_D_MAN     	4		//PORTA
#define DOWN_PB_D_AUTO     	5		//PORTA

#define LIMIT_PORT					GPIOB
#define UP_LIMIT      			6		//PORTB
#define DOWN_LIMIT    			7		//PORTB

#define IRR_PORT						GPIOA
#define LOCK_SW							6		//PORTA
#define JAM_PB							7		//PORTA

uint8_t man_pass_up = 0;
uint8_t auto_pass_up = 0;
uint8_t man_pass_down = 0;
uint8_t auto_pass_down = 0;

uint8_t man_driver_up = 0;
uint8_t auto_driver_up = 0;
uint8_t man_driver_down = 0;
uint8_t auto_driver_down = 0;

/**********************************************************************************
*																		MOTOR																					*
***********************************************************************************/

#define DC_PORT		GPIOF

#define ENA 			1
#define IN1				2
#define IN2 			3


typedef enum{
	UP, DOWN, STOP
}DcMotor_State;

/**********************************************************************************
*																		MODES																					*
***********************************************************************************/

#define PULLUP 		0
#define PULLDOWN 	1
#define MODE_PB		PULLUP

/**********************************************************************************
*															FUNCTION PROTOTYPES																	*
***********************************************************************************/

void PortA_Init(void);
void PortB_Init(void);
void PortF_Init(void);
void DC_Motor_Write(DcMotor_State Dir);

/**********************************************************************************
*														  FUNCTION DEFINITIONS																*
***********************************************************************************/

void PortA_Init(void){ 
  SYSCTL->RCGCGPIO |= 0x00000001;    // 1) Operate PortA Clock
  GPIOA->LOCK = 0x4C4F434B;  				 // 2) unlock PortA  
  GPIOA->CR = 0xFC;          				 // allow changes to PA7-PA2       
  GPIOA->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOA->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL  
  GPIOA->DIR = 0x00;         				 // 5) PA7-PA2 input   
  GPIOA->AFSEL = 0x00;      				 // 6) no alternate function
  GPIOA->PUR = 0xFC;       				   // enable pullup resistors on PA7-PA2      
  GPIOA->DEN = 0xFC;       				   // 7) enable digital pins PA7-PA2
	GPIOA->DATA = 0x00;
	
	
	// Setup the interrupt on PortA
	GPIOA->ICR |= 0xFC;     // Clear any Previous Interrupt 
	GPIOA->IM |= 0x00;     
	GPIOA->ICR |= 0xFC;
	GPIOA->IS |= 0x00;     
	GPIOA->IBE |= 0x00;
	GPIOA->IEV &= ~0x00;   
	GPIOA->ICR |= 0xFC;
	GPIOA->IM |= 0xFC;			// Unmask the interrupts for PA7-PA2


	NVIC_EnableIRQ(GPIOA_IRQn);        // Enable the Interrupt for PortA in NVIC
}

void PortB_Init(void){ 
  SYSCTL->RCGCGPIO |= 0x00000002;    // 1) Operate PortB Clock
  GPIOB->LOCK = 0x4C4F434B;  				 // 2) unlock PortB 
  GPIOB->CR = 0xF3;          				 // allow changes to PB7-PB6       
  GPIOB->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOB->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL  
  GPIOB->DIR = 0x00;         				 // 5) PB7-PB6 input   
  GPIOB->AFSEL = 0x00;      				 // 6) no alternate function
  GPIOB->PUR = 0xF3;       				   // enable pullup resistors on PB7-PB6     
  GPIOB->DEN = 0xF3;       				   // 7) enable digital pins PB7-PB6
	GPIOB->DATA = 0x00;
	
	
	// Setup the interrupt on PortB
	GPIOB->ICR |= 0x33;     // Clear any Previous Interrupt 
	GPIOB->IM |= 0x00;     
	GPIOB->ICR |= 0x33;
	GPIOB->IS |= 0x00;     
	GPIOB->IBE |= 0x00;
	GPIOB->IEV &= ~0x00;   
	GPIOB->ICR |= 0x33;
	GPIOB->IM |= 0x33;			// Unmask the interrupts for PB7-PB6

	NVIC_EnableIRQ(GPIOB_IRQn);        // Enable the Interrupt for PortB in NVIC
}

void PortF_Init(void){ 
  SYSCTL->RCGCGPIO |= 0x00000020;    // 1) Operate PortF Clock
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

	NVIC_EnableIRQ(GPIOF_IRQn);        // Enable the Interrupt for PortF in NVIC
}

void DC_Motor_Write(DcMotor_State Dir){
	switch(Dir){
		case UP:
			DC_PORT->DATA |= (1 << IN1);
			DC_PORT->DATA &= ~(1 << IN2);
			break;
		case DOWN:
			DC_PORT->DATA |= (1 << IN2);
			DC_PORT->DATA &= ~(1 << IN1);
			break;
		case STOP:
			DC_PORT->DATA &= ~(1 << IN1);
			DC_PORT->DATA &= ~(1 << IN2);
			break;
	}
	DC_PORT->DATA |= (1 << ENA);
}

/**********************************************************************************
*																		  MAIN																				*
***********************************************************************************/
//define a Semaphores Handles
xSemaphoreHandle xBinarySemaphore;
xSemaphoreHandle xJamSemaphore;
xSemaphoreHandle xDriverSemaphore;
xSemaphoreHandle xPassengerSemaphore;

//define a Mutex Handles
xSemaphoreHandle auto_driver_up_mutex;
xSemaphoreHandle auto_driver_down_mutex;
xSemaphoreHandle auto_pass_up_mutex;
xSemaphoreHandle auto_pass_down_mutex;

xTaskHandle PassengerHandle;

//this Task "Handler" is awakened when the semaphore is available
void JamTask(void *pvParameters){
	for(;;)
	{
			xSemaphoreTake(xJamSemaphore,portMAX_DELAY);
      if(((IRR_PORT->DATA & (1 << JAM_PB)) == MODE_PB)){
				DC_Motor_Write(STOP);
				int32_t i, j;
				for(i = 0 ; i < 500; i++){
					for(j = 0; j < 3180; j++){
						DC_Motor_Write(DOWN);
					}
				}
				
				while(!(((PASS_PORT->DATA & (1 << UP_PB_P_AUTO)) == 0) || 
								((PASS_PORT->DATA & (1 << DOWN_PB_P_AUTO)) == 0) ||  
								((PASS_PORT->DATA & (1 << UP_PB_P_MAN)) == 0) || 
								((PASS_PORT->DATA & (1 << DOWN_PB_P_MAN)) == 0) || 
								((DRIVER_PORT->DATA & (1 << UP_PB_D_AUTO)) == 0) || 
								((DRIVER_PORT->DATA & (1 << DOWN_PB_D_AUTO)) == 0) ||  
								((DRIVER_PORT->DATA & (1 << UP_PB_D_MAN)) == 0) || 
								((DRIVER_PORT->DATA & (1 << DOWN_PB_D_MAN)) == 0))){
					
					DC_Motor_Write(STOP);
				}
			}
    }
}

void DriverTask(void *pvParameters){
	
	for(;;)
	{
		xSemaphoreTake(xDriverSemaphore,portMAX_DELAY);
		vTaskSuspend(PassengerHandle);
		if(auto_driver_down) 
		{
			while(auto_driver_down){
				if((DRIVER_PORT->DATA & (1 << UP_PB_D_MAN)) == MODE_PB || ((DRIVER_PORT->DATA & (1 << UP_PB_D_AUTO)) == MODE_PB)){
					break;
				}
				if((LIMIT_PORT->DATA & (1 << DOWN_LIMIT)) == MODE_PB){
					break;
				}
				DC_Motor_Write(DOWN);
			}
			DC_Motor_Write(STOP);
			xSemaphoreTake(auto_driver_down_mutex, portMAX_DELAY);
			auto_driver_down = 0;
			xSemaphoreGive(auto_driver_down_mutex);
		}
		else if(auto_driver_up) 
		{
			while(auto_driver_up){
				if(((DRIVER_PORT->DATA & (1 << DOWN_PB_D_MAN)) == MODE_PB) || ((DRIVER_PORT->DATA & (1 << DOWN_PB_D_AUTO)) == MODE_PB)){
					break;
				}
				if((LIMIT_PORT->DATA & (1 << UP_LIMIT)) == MODE_PB){
					break;
				}
				DC_Motor_Write(UP);
			}
			DC_Motor_Write(STOP);
			xSemaphoreTake(auto_driver_up_mutex, portMAX_DELAY);
			auto_driver_up = 0;
			xSemaphoreGive(auto_driver_up_mutex);
		}
		else if((DRIVER_PORT->DATA & (1 << DOWN_PB_D_MAN)) == MODE_PB) 
		{
			auto_pass_down = 0;
			auto_pass_up = 0;
			while((DRIVER_PORT->DATA & (1 << DOWN_PB_D_MAN)) == MODE_PB){
				if((LIMIT_PORT->DATA & (1 << DOWN_LIMIT)) == MODE_PB){
					break;
				}
				DC_Motor_Write(DOWN);
			}
			DC_Motor_Write(STOP);
			
		}
		else if((DRIVER_PORT->DATA & (1 << UP_PB_D_MAN)) == MODE_PB) 
		{
			auto_pass_down = 0;
			auto_pass_up = 0;
			while((DRIVER_PORT->DATA & (1 << UP_PB_D_MAN)) == MODE_PB){
				if((LIMIT_PORT->DATA & (1 << UP_LIMIT)) == MODE_PB){
					break;
				}
				DC_Motor_Write(UP);
			}
			
			DC_Motor_Write(STOP);
			
		}
	}
}

//This Periodic task is preempted by the task "Handler"
void PassengerTask(void *pvParameters){
	
	for(;;)
	{
		xSemaphoreTake(xPassengerSemaphore,portMAX_DELAY);
		if((IRR_PORT->DATA & (1 << LOCK_SW)) == MODE_PB){
			continue;
		}
		else if(auto_pass_down) 
		{
			while(auto_pass_down){
				if(((PASS_PORT->DATA & (1 << UP_PB_P_MAN)) == MODE_PB )||
					((PASS_PORT->DATA & (1 << UP_PB_P_AUTO)) == MODE_PB) ||
					((DRIVER_PORT->DATA & (1 << UP_PB_D_MAN)) == MODE_PB) ||
					((DRIVER_PORT->DATA & (1 << UP_PB_D_AUTO)) == MODE_PB)) {
						auto_pass_down = 0;
					break;
				}
				if((LIMIT_PORT->DATA & (1 << DOWN_LIMIT)) == MODE_PB){
					break;
				}
				if((IRR_PORT->DATA & (1 << LOCK_SW)) == MODE_PB){
					break;
				}
				DC_Motor_Write(DOWN);
			}
			DC_Motor_Write(STOP);
			xSemaphoreTake(auto_pass_down_mutex, portMAX_DELAY);
			auto_pass_down = 0;
			xSemaphoreGive(auto_pass_down_mutex);
		}
		else if(auto_pass_up) 
		{
			while(auto_pass_up){
				if(((PASS_PORT->DATA & (1 << DOWN_PB_P_MAN)) == MODE_PB) ||
					((PASS_PORT->DATA & (1 << DOWN_PB_P_AUTO)) == MODE_PB) ||
					((DRIVER_PORT->DATA & (1 << DOWN_PB_D_MAN)) == MODE_PB) ||
					((DRIVER_PORT->DATA & (1 << DOWN_PB_D_AUTO)) == MODE_PB)){
						auto_pass_up = 0;
					break;
				}
				if((LIMIT_PORT->DATA & (1 << UP_LIMIT)) == MODE_PB){
					break;
				}
				if((IRR_PORT->DATA & (1 << LOCK_SW)) == MODE_PB){
					break;
				}
				DC_Motor_Write(UP);
			}
			DC_Motor_Write(STOP);
			xSemaphoreTake(auto_pass_up_mutex, portMAX_DELAY);
			auto_pass_up = 0;
			xSemaphoreGive(auto_pass_up_mutex);
		}
		else if((PASS_PORT->DATA & (1 << DOWN_PB_P_MAN)) == MODE_PB) 
		{
			while((PASS_PORT->DATA & (1 << DOWN_PB_P_MAN)) == MODE_PB){
				if((LIMIT_PORT->DATA & (1 << DOWN_LIMIT)) == MODE_PB){
					break;
				}
				if((IRR_PORT->DATA & (1 << LOCK_SW)) == MODE_PB){
					break;
				}
				DC_Motor_Write(DOWN);
			}
			DC_Motor_Write(STOP);
		}
		else if((PASS_PORT->DATA & (1 << UP_PB_P_MAN)) == MODE_PB) 
		{
			while((PASS_PORT->DATA & (1 << UP_PB_P_MAN)) == MODE_PB){
				if((LIMIT_PORT->DATA & (1 << UP_LIMIT)) == MODE_PB){
					break;
				}
				if((IRR_PORT->DATA & (1 << LOCK_SW)) == MODE_PB){
					break;
				}
				DC_Motor_Write(UP);
			}
			DC_Motor_Write(STOP);
		}
	}
}
/*------------------------------------------------------------------------*/
int main( void ){
	
		PortA_Init();
		PortB_Init();
    PortF_Init();
		__ASM("CPSIE i");
		
		vSemaphoreCreateBinary(xBinarySemaphore);
		vSemaphoreCreateBinary(xJamSemaphore);
		vSemaphoreCreateBinary(xDriverSemaphore);
		vSemaphoreCreateBinary(xPassengerSemaphore);
		//xBinarySemaphore = xSemaphoreCreateBinary();
	
		// Create the mutex
		auto_driver_down_mutex = xSemaphoreCreateMutex();
		auto_driver_up_mutex = xSemaphoreCreateMutex();
		auto_pass_down_mutex = xSemaphoreCreateMutex();
		auto_pass_up_mutex = xSemaphoreCreateMutex();
	
	
	if( xBinarySemaphore != NULL )
		{
			
			xTaskCreate( JamTask, "Jam", 250, NULL, 3, NULL );
			xTaskCreate( DriverTask, "Driver", 250, NULL, 2, NULL );
			xTaskCreate( PassengerTask, "Passenger", 250, NULL, 1, &PassengerHandle );
			
			vTaskStartScheduler();
		}

    /* If all is well we will never reach here as the scheduler will now be
    running the tasks.  If we do reach here then it is likely that there was
    insufficient heap memory available for a resource to be created. */
    for( ;; );
		
}
/*------------------------------------------------------------------------*/
	
//Port-A handler
void GPIOA_Handler(void){

	uint32_t i;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	if((IRR_PORT->DATA & (1 << JAM_PB)) == MODE_PB ){
		//Give the semaphore to the Task named handler
		if (auto_pass_up || auto_driver_up){
			xSemaphoreGiveFromISR(xJamSemaphore,&xHigherPriorityTaskWoken);
		}
		auto_pass_up = 0;
		auto_driver_up = 0;
	}
	else if((DRIVER_PORT->DATA & (1 << DOWN_PB_D_AUTO)) == MODE_PB ){
		auto_driver_down = 1;
		auto_pass_down = 0;
		auto_pass_up = 0;
		//while((PASS_PORT->DATA & (1 << DOWN_PB_P_AUTO)) == MODE_PB ){}
		xSemaphoreGiveFromISR(xDriverSemaphore,&xHigherPriorityTaskWoken);
	}
	else if((DRIVER_PORT->DATA & (1 << UP_PB_D_AUTO)) == MODE_PB ){
		auto_driver_up = 1;
		auto_pass_down = 0;
		auto_pass_up = 0;
		//while((PASS_PORT->DATA & (1 << UP_PB_P_AUTO)) == MODE_PB ){}
		xSemaphoreGiveFromISR(xDriverSemaphore,&xHigherPriorityTaskWoken);
	}
	else if((DRIVER_PORT->DATA & (1 << DOWN_PB_D_MAN)) == MODE_PB ){
		auto_pass_down = 0;
		auto_pass_up = 0;
		//while((PASS_PORT->DATA & (1 << DOWN_PB_D_AUTO)) == MODE_PB ){}
		xSemaphoreGiveFromISR(xDriverSemaphore,&xHigherPriorityTaskWoken);
	}
	else if((DRIVER_PORT->DATA & (1 << UP_PB_D_MAN)) == MODE_PB ){
		auto_pass_down = 0;
		auto_pass_up = 0;
		//while((PASS_PORT->DATA & (1 << UP_PB_D_AUTO)) == MODE_PB ){}
		xSemaphoreGiveFromISR(xDriverSemaphore,&xHigherPriorityTaskWoken);
	}
	/*else if((IRR_PORT->DATA & (1 << LOCK_SW)) == MODE_PB ){
		//xSemaphoreGiveFromISR(xLockSemaphore,&xHigherPriorityTaskWoken);
	}*/
	//else{}
	
	
	
	GPIOA->ICR = 0xFC;        // clear the interrupt flag of PORTF
  i= GPIOA->ICR ;           // Reading the register to force the flag to be cleared

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

//Port-F handler
void GPIOB_Handler(void){

	uint32_t i;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	if(auto_driver_down || auto_driver_up || man_driver_down || man_driver_up){
	}
	else if((PASS_PORT->DATA & (1 << DOWN_PB_P_AUTO)) == MODE_PB ){
		auto_pass_down =1;
		//while((PASS_PORT->DATA & (1 << DOWN_PB_P_AUTO)) == MODE_PB ){}
		xSemaphoreGiveFromISR(xPassengerSemaphore,&xHigherPriorityTaskWoken);
		xTaskResumeFromISR(PassengerHandle);
	}
	else if((PASS_PORT->DATA & (1 << UP_PB_P_AUTO)) == MODE_PB ){
		auto_pass_up =1;
		//while((PASS_PORT->DATA & (1 << UP_PB_P_AUTO)) == MODE_PB ){}
		xSemaphoreGiveFromISR(xPassengerSemaphore,&xHigherPriorityTaskWoken);
		xTaskResumeFromISR(PassengerHandle);
	}
	else if((PASS_PORT->DATA & (1 << DOWN_PB_D_MAN)) == MODE_PB ){
		//while((PASS_PORT->DATA & (1 << DOWN_PB_D_MAN)) == MODE_PB ){}
		xSemaphoreGiveFromISR(xPassengerSemaphore,&xHigherPriorityTaskWoken);
		xTaskResumeFromISR(PassengerHandle);
	}
	else if((PASS_PORT->DATA & (1 << UP_PB_D_MAN)) == MODE_PB ){
		//while((PASS_PORT->DATA & (1 << UP_PB_D_MAN)) == MODE_PB ){}
		xSemaphoreGiveFromISR(xPassengerSemaphore,&xHigherPriorityTaskWoken);
		xTaskResumeFromISR(PassengerHandle);
	}
	//else{}
		
	
	GPIOB->ICR = 0xF3;        // clear the interrupt flag of PORTF
  i= GPIOB->ICR ;           // Reading the register to force the flag to be cleared

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

//Port-F handler
void GPIOF_Handler(void){

	uint32_t i;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	//Give the semaphore to the Task named handler
  xSemaphoreGiveFromISR(xJamSemaphore,&xHigherPriorityTaskWoken);
	
	GPIOF->ICR = 0x11;        // clear the interrupt flag of PORTF
  i= GPIOF->ICR ;           // Reading the register to force the flag to be cleared

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