//# Window_Power_System
//# Commented Code same as in main.c but for more infortmation 
#include <stdint.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

/**********************************************************************************
 *																		DEFINITIONS																		*
 ***********************************************************************************/
/*	All Passenger Port and Pins	*/
#define PASS_PORT						GPIOB
#define UP_PB_P_AUTO       	0		//PORTB
#define UP_PB_P_MAN       	1		//PORTB
#define DOWN_PB_P_MAN 			4		//PORTB
#define DOWN_PB_P_AUTO			5		//PORTB

/*	All Driver Port and Pins	*/
#define DRIVER_PORT					GPIOA
#define UP_PB_D_AUTO       	2		//PORTA
#define UP_PB_D_MAN       	3		//PORTA
#define DOWN_PB_D_MAN     	4		//PORTA
#define DOWN_PB_D_AUTO     	5		//PORTA

/*	Both Limit Switches Port and Pins	*/
#define LIMIT_PORT					GPIOB
#define UP_LIMIT      			6		//PORTB
#define DOWN_LIMIT    			7		//PORTB

/*	Both Irregularities Port and Pins	*/
#define IRR_PORT						GPIOA
#define LOCK_SW							6		//PORTA
#define JAM_PB							7		//PORTA

/*	All Motor Port and Pins	*/
#define DC_PORT		GPIOF
#define ENA 			1
#define IN1				2
#define IN2 			3

/*	Both Push Button Modes	*/
#define PULLUP 		0
#define PULLDOWN 	1
#define MODE_PB		PULLUP

/*	Motor States	*/
typedef enum{
	UP, DOWN, STOP
}DcMotor_State;

/*	Window State Global Variables	*/
uint8_t man_pass_up = 0;
uint8_t auto_pass_up = 0;
uint8_t man_pass_down = 0;
uint8_t auto_pass_down = 0;

uint8_t man_driver_up = 0;
uint8_t auto_driver_up = 0;
uint8_t man_driver_down = 0;
uint8_t auto_driver_down = 0;

/**********************************************************************************
 *															FUNCTION PROTOTYPES																	*
 ***********************************************************************************/

/*
 * Description :
 * Initialize the PORT A:
 *  1) Operate PortA Clock
 *  2) Unlock PortA
 *  3) Allow changes to PA7-PA2
 *  4) Disable analog function
 *  5) GPIO clear bit PCTL
 *  6) Set PA7-PA2 as input
 *  7) Clarify pin has no alternate function
 *  8) Enable pullup resistors on PA7-PA2
 *  9) Enable digital pins PA7-PA2
 * 10) Setup PA7-PA2 Interrupts
 * 11) Enable PORT A Interrupt
 */
void PortA_Init(void);

/*
 * Description :
 * Initialize the PORT B:
 *  1) Operate PortB Clock
 *  2) Unlock PortB
 *  3) Allow changes to PB7-PB4, PB1-PB0
 *  4) Disable analog function
 *  5) GPIO clear bit PCTL
 *  6) Set PB7-PB4, PB1-PB0 as input
 *  7) Clarify pin has no alternate function
 *  8) Enable pullup resistors on PB7-PB4, PB1-PB0
 *  9) Enable digital pins PB7-PB4, PB1-PB0
 * 10) Setup PB5-PB4, PB1-PB0 Interrupts
 * 11) Enable PORT B Interrupt
 */
void PortB_Init(void);

/*
 * Description :
 * Initialize the PORT F:
 *  1) Operate PortF Clock
 *  2) Unlock PortF
 *  3) Allow changes to PF4-PF0
 *  4) Disable analog function
 *  5) GPIO clear bit PCTL
 *  6) Set PF4-PF0 as input
 *  7) Clarify pin has no alternate function
 *  8) Enable pullup resistors on PF4-PF0
 *  9) Enable digital pins PF4-PF0
 * 10) Setup PF4, PF0 Interrupts
 * 11) Enable PORT F Interrupt
 */
void PortF_Init(void);

/*
 * Description :
 * Operate Motor on the Window Direction Given
 */
void DC_Motor_Write(DcMotor_State Dir);


/**********************************************************************************
 *														  FUNCTION DEFINITIONS																*
 ***********************************************************************************/

void PortA_Init(void)
{ 
	SYSCTL->RCGCGPIO |= 0x00000001;    					// 1) Operate PortA Clock
	GPIOA->LOCK       = 0x4C4F434B;  				 		// 2) unlock PortA  
	GPIOA->CR         = 0xFC;          					// allow changes to PA7-PA2       
	GPIOA->AMSEL      = 0x00;       				 		// 3) disable analog function
	GPIOA->PCTL       = 0x00000000;  				 		// 4) GPIO clear bit PCTL  
	GPIOA->DIR        = 0x00;         				 	// 5) PA7-PA2 input   
	GPIOA->AFSEL      = 0x00;      				 			// 6) no alternate function
	GPIOA->PUR        = 0xFC;       				   	// enable pullup resistors on PA7-PA2      
	GPIOA->DEN        = 0xFC;       				   	// 7) enable digital pins PA7-PA2
	GPIOA->DATA       = 0x00;


	// Setup the interrupt on PortA
	GPIOA->ICR |= 0xFC;     	// Clear any Previous Interrupt 
	GPIOA->IM  |= 0x00;     	// Mask the interrupts for PA7-PA2
	GPIOA->ICR |= 0xFC;				// Clear any Previous Interrupt
	GPIOA->IS  |= 0x00;     	// Make bits PA7-PA2 edge sensitive
	GPIOA->IBE |= 0x00;				// Single Edge Control defined by IEV register
	GPIOA->IEV &= ~0x00;   		// Sense on Rising Edge
	GPIOA->ICR |= 0xFC;				// Clear any Previous Interrupt
	GPIOA->IM  |= 0xFC;				// Unmask the interrupts for PA7-PA2


	NVIC_EnableIRQ(GPIOA_IRQn);        // Enable the Interrupt for PortA in NVIC
}

void PortB_Init(void)
{ 
	SYSCTL->RCGCGPIO |= 0x00000002;    					// 1) Operate PortB Clock
	GPIOB->LOCK 			= 0x4C4F434B;  						// 2) unlock PortB 
	GPIOB->CR 				= 0xF3;          				 	// allow changes to PB7-PB6       
	GPIOB->AMSEL			= 0x00;       				 		// 3) disable analog function
	GPIOB->PCTL 			= 0x00000000;  				 		// 4) GPIO clear bit PCTL  
	GPIOB->DIR 				= 0x00;         				 	// 5) PB7-PB6 input   
	GPIOB->AFSEL 			= 0x00;      				 			// 6) no alternate function
	GPIOB->PUR 				= 0xF3;       				   	// enable pullup resistors on PB7-PB6     
	GPIOB->DEN 				= 0xF3;       				   	// 7) enable digital pins PB7-PB6
	GPIOB->DATA 			= 0x00;


	// Setup the interrupt on PortB
	GPIOB->ICR 	|= 0x33;     	// Clear any Previous Interrupt 
	GPIOB->IM 	|= 0x00;     	// Mask the interrupts for PB5-PB4, PB1-PB0
	GPIOB->ICR 	|= 0x33;			// Clear any Previous Interrupt
	GPIOB->IS 	|= 0x00;     	// Make bits PB5-PB4, PB1-PB0 edge sensitive
	GPIOB->IBE 	|= 0x00;			// Single Edge Control defined by IEV register
	GPIOB->IEV 	&= ~0x00;   	// Sense on Rising Edge
	GPIOB->ICR 	|= 0x33;			// Clear any Previous Interrupt
	GPIOB->IM 	|= 0x33;			// Unmask the interrupts for PB5-PB4, PB1-PB0

	NVIC_EnableIRQ(GPIOB_IRQn);        // Enable the Interrupt for PortB in NVIC
}

void PortF_Init(void)
{ 
	SYSCTL->RCGCGPIO |= 0x00000020;    					// 1) Operate PortF Clock
	GPIOF->LOCK 			= 0x4C4F434B;  				 		// 2) unlock PortF PF0  
	GPIOF->CR 				= 0x1F;          				 	// allow changes to PF4-0       
	GPIOF->AMSEL			= 0x00;       				 		// 3) disable analog function
	GPIOF->PCTL 			= 0x00000000;  				 		// 4) GPIO clear bit PCTL  
	GPIOF->DIR 				= 0x0E;         				 	// 5) PF4,PF0 input, PF3,PF2,PF1 output   
	GPIOF->AFSEL 			= 0x00;      				 			// 6) no alternate function
	GPIOF->PUR 				= 0x11;       				   	// enable pullup resistors on PF4,PF0       
	GPIOF->DEN 				= 0x1F;       				   	// 7) enable digital pins PF4-PF0
	GPIOF->DATA 			= 0x00;

	// Setup the interrupt on PortF
	GPIOF->ICR 	 = 0x11;     // Clear any Previous Interrupt 
	GPIOF->IM 	|=0x11;      	// Unmask the interrupts for PF0 and PF4
	GPIOF->IS 	|= 0x11;     	// Make bits PF0 and PF4 level sensitive
	GPIOF->IEV 	&= ~0x11;   	// Sense on Low Level

	NVIC_EnableIRQ(GPIOF_IRQn);        // Enable the Interrupt for PortF in NVIC
}

void DC_Motor_Write(DcMotor_State Dir)
{
	/*	Which direction is used	*/
	switch(Dir)
	{
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
	/*	Always output 1 on Enable pin	*/
	DC_PORT->DATA |= (1 << ENA);
}


/**********************************************************************************
 *																		  HANDLES																			*
 ***********************************************************************************/
// Define Semaphores Handles
xSemaphoreHandle xBinarySemaphore;
xSemaphoreHandle xJamSemaphore;
xSemaphoreHandle xDriverSemaphore;
xSemaphoreHandle xPassengerSemaphore;

// Define Mutex Handles
xSemaphoreHandle auto_driver_up_mutex;
xSemaphoreHandle auto_driver_down_mutex;
xSemaphoreHandle auto_pass_up_mutex;
xSemaphoreHandle auto_pass_down_mutex;

xTaskHandle PassengerHandle;

/**********************************************************************************
 *																		  TASKS																				*
 ***********************************************************************************/

/*
 * Description :
 * Task is used in case the window was jammed during an auto close operation
 */
void JamTask(void *pvParameters){
	for(;;)
	{
		xSemaphoreTake(xJamSemaphore,portMAX_DELAY);
		if(((IRR_PORT->DATA & (1 << JAM_PB)) == MODE_PB))
		{
			DC_Motor_Write(STOP);																															// To not go from Up to Down movement instantly
			int32_t i, j;
			for(i = 0 ; i < 500; i++)
			{
				for(j = 0; j < 3180; j++)
				{
					DC_Motor_Write(DOWN);																													// Moving window down for 0.5 seconds
				}
			}																																		

			// This is for all buttons to get user out of the task and operate correctly
			while(!(((PASS_PORT->DATA & (1 << UP_PB_P_AUTO)) == MODE_PB) 	|| 					
					((PASS_PORT->DATA & (1 << DOWN_PB_P_AUTO)) == MODE_PB) 		||  				 
					((PASS_PORT->DATA & (1 << UP_PB_P_MAN)) == MODE_PB) 			|| 						 
					((PASS_PORT->DATA & (1 << DOWN_PB_P_MAN)) == MODE_PB) 		|| 					 
					((DRIVER_PORT->DATA & (1 << UP_PB_D_AUTO)) == MODE_PB) 		|| 				  
					((DRIVER_PORT->DATA & (1 << DOWN_PB_D_AUTO)) == MODE_PB) 	||  			 
					((DRIVER_PORT->DATA & (1 << UP_PB_D_MAN)) == MODE_PB) 		|| 					 
					((DRIVER_PORT->DATA & (1 << DOWN_PB_D_MAN)) == MODE_PB)))
			{				
				DC_Motor_Write(STOP);
			}
		}
	}
}


/*
 * Description :
 * Task is used for the driver to operate the window
 */
void DriverTask(void *pvParameters)
{
	// All of the following are only on the driver's side
	for(;;)
	{
		xSemaphoreTake(xDriverSemaphore,portMAX_DELAY);
		vTaskSuspend(PassengerHandle);
		// Automatic Down Mode
		if(auto_driver_down) 
		{
			while(auto_driver_down)
			{
				// Up order is given during down operation (only from driver's side)
				if((DRIVER_PORT->DATA & (1 << UP_PB_D_MAN)) == MODE_PB || ((DRIVER_PORT->DATA & (1 << UP_PB_D_AUTO)) == MODE_PB))
				{
					break;
				}
				// Down Limit Switch is hit
				if((LIMIT_PORT->DATA & (1 << DOWN_LIMIT)) == MODE_PB)
				{
					break;
				}
				DC_Motor_Write(DOWN);
			}
			DC_Motor_Write(STOP);
			// Mutex for Global Variables
			xSemaphoreTake(auto_driver_down_mutex, portMAX_DELAY);
			auto_driver_down = 0;
			xSemaphoreGive(auto_driver_down_mutex);
		}

		// Automatic Up Mode
		else if(auto_driver_up) 
		{
			while(auto_driver_up)
			{
				// Down order is given during up operation (only from driver's side)
				if(((DRIVER_PORT->DATA & (1 << DOWN_PB_D_MAN)) == MODE_PB) || ((DRIVER_PORT->DATA & (1 << DOWN_PB_D_AUTO)) == MODE_PB))
				{
					break;
				}
				// Up Limit Switch is hit
				if((LIMIT_PORT->DATA & (1 << UP_LIMIT)) == MODE_PB)
				{
					break;
				}
				DC_Motor_Write(UP);
			}
			DC_Motor_Write(STOP);

			// Mutex for Global Variables
			xSemaphoreTake(auto_driver_up_mutex, portMAX_DELAY);
			auto_driver_up = 0;
			xSemaphoreGive(auto_driver_up_mutex);
		}

		// Manual Down Mode
		else if((DRIVER_PORT->DATA & (1 << DOWN_PB_D_MAN)) == MODE_PB) 
		{
			while((DRIVER_PORT->DATA & (1 << DOWN_PB_D_MAN)) == MODE_PB)
			{
				// Down Limit Switch is hit
				if((LIMIT_PORT->DATA & (1 << DOWN_LIMIT)) == MODE_PB)
				{
					break;
				}
				DC_Motor_Write(DOWN);
			}
			DC_Motor_Write(STOP);
		}
		// Manual Up Mode
		else if((DRIVER_PORT->DATA & (1 << UP_PB_D_MAN)) == MODE_PB) 
		{
			while((DRIVER_PORT->DATA & (1 << UP_PB_D_MAN)) == MODE_PB)
			{
				// Up Limit Switch is hit
				if((LIMIT_PORT->DATA & (1 << UP_LIMIT)) == MODE_PB)
				{
					break;
				}
				DC_Motor_Write(UP);
			}
			DC_Motor_Write(STOP);
		}
	}
}


/*
 * Description :
 * Task is used for the passenger to operate the window
 */
void PassengerTask(void *pvParameters)
{
	// All of the following are only on the passenger's side
	for(;;)
	{
		xSemaphoreTake(xPassengerSemaphore,portMAX_DELAY);

		/* In case the lock switch is pressed nothing in the passenger's side will work	*/
		if((IRR_PORT->DATA & (1 << LOCK_SW)) == MODE_PB)
		{
			continue;
		}

		// Automatic Down Mode
		else if(auto_pass_down) 
		{
			while(auto_pass_down)
			{
				// Up order is given during down operation (from driver's or passenger's side)
				if(((PASS_PORT->DATA & (1 << UP_PB_P_MAN)) == MODE_PB)   || 
					 ((PASS_PORT->DATA & (1 << UP_PB_P_AUTO)) == MODE_PB)  ||
					 ((DRIVER_PORT->DATA & (1 << UP_PB_D_MAN)) == MODE_PB) ||
					 ((DRIVER_PORT->DATA & (1 << UP_PB_D_AUTO)) == MODE_PB))
				{
					break;
				}
				// Down Limit Switch is hit
				if((LIMIT_PORT->DATA & (1 << DOWN_LIMIT)) == MODE_PB)
				{
					break;
				}
				if((IRR_PORT->DATA & (1 << LOCK_SW)) == MODE_PB)
				{
					break;
				}
				DC_Motor_Write(DOWN);
			}
			DC_Motor_Write(STOP);
			// Mutex for Global Variables
			xSemaphoreTake(auto_pass_down_mutex, portMAX_DELAY);
			auto_pass_down = 0;
			xSemaphoreGive(auto_pass_down_mutex);
		}

		// Automatic Up Mode
		else if(auto_pass_up) 
		{
			while(auto_pass_up)
			{
				// Down order is given during up operation (from driver's or passenger's side)
				if(((PASS_PORT->DATA & (1 << DOWN_PB_P_MAN)) == MODE_PB) 		|| 
					 ((PASS_PORT->DATA & (1 << DOWN_PB_P_AUTO)) == MODE_PB) 	|| 
					 ((DRIVER_PORT->DATA & (1 << DOWN_PB_D_MAN)) == MODE_PB) 	|| 
					 ((DRIVER_PORT->DATA & (1 << DOWN_PB_D_AUTO)) == MODE_PB))
				{
					break;
				}
				// Up Limit Switch is hit
				if((LIMIT_PORT->DATA & (1 << UP_LIMIT)) == MODE_PB)
				{
					break;
				}
				if((IRR_PORT->DATA & (1 << LOCK_SW)) == MODE_PB)
				{
					break;
				}
				DC_Motor_Write(UP);
			}
			DC_Motor_Write(STOP);
			// Mutex for Global Variables
			xSemaphoreTake(auto_pass_up_mutex, portMAX_DELAY);
			auto_pass_up = 0;
			xSemaphoreGive(auto_pass_up_mutex);
		}

		// Manual Down Mode
		else if((PASS_PORT->DATA & (1 << DOWN_PB_P_MAN)) == MODE_PB) 
		{
			while((PASS_PORT->DATA & (1 << DOWN_PB_P_MAN)) == MODE_PB)
			{
				// Down Limit Switch is hit
				if((LIMIT_PORT->DATA & (1 << DOWN_LIMIT)) == MODE_PB)
				{
					break;
				}
				if((IRR_PORT->DATA & (1 << LOCK_SW)) == MODE_PB)
				{
					break;
				}
				DC_Motor_Write(DOWN);
			}
			DC_Motor_Write(STOP);
		}

		// Manual Up Mode
		else if((PASS_PORT->DATA & (1 << UP_PB_P_MAN)) == MODE_PB) 
		{
			while((PASS_PORT->DATA & (1 << UP_PB_P_MAN)) == MODE_PB)
			{
				// Up Limit Switch is hit
				if((LIMIT_PORT->DATA & (1 << UP_LIMIT)) == MODE_PB)
				{
					break;
				}
				if((IRR_PORT->DATA & (1 << LOCK_SW)) == MODE_PB)
				{
					break;
				}
				DC_Motor_Write(UP);
			}
			DC_Motor_Write(STOP);
		}	
	}
}
/**********************************************************************************
 *																		  MAIN																				*
 ***********************************************************************************/
int main( void )
{

	/*	Port Initialization	*/
	PortA_Init();
	PortB_Init();
	PortF_Init();

	/Enable Global Interrupt Flag/
	__ASM("CPSIE i");

	/*	Create Binary Semaphores	*/
	vSemaphoreCreateBinary(xBinarySemaphore);
	vSemaphoreCreateBinary(xJamSemaphore);
	vSemaphoreCreateBinary(xDriverSemaphore);
	vSemaphoreCreateBinary(xPassengerSemaphore);

	/*	Create Mutex	*/
	auto_driver_down_mutex  = xSemaphoreCreateMutex();
	auto_driver_up_mutex    = xSemaphoreCreateMutex();
	auto_pass_down_mutex    = xSemaphoreCreateMutex();
	auto_pass_up_mutex      = xSemaphoreCreateMutex();

	if( xBinarySemaphore != NULL )
	{

		/*	Create Tasks	*/
		xTaskCreate( JamTask, "Jam", 250, NULL, 3, NULL );
		xTaskCreate( DriverTask, "Driver", 250, NULL, 2, NULL );
		xTaskCreate( PassengerTask, "Passenger", 250, NULL, 1, &PassengerHandle );


		/*	Start Scheduler	*/		
		vTaskStartScheduler();
	}

	for( ;; );

}
/**********************************************************************************
 *																	GPIO HANDLERS																		*
 ***********************************************************************************/

/*	External GPIO A Interrupt Service Routine	*/
void GPIOA_Handler(void)
{

	uint32_t i;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	// Jam Push Button is Pressed
	if((IRR_PORT->DATA & (1 << JAM_PB)) == MODE_PB )
	{
		if (auto_pass_up || auto_driver_up)
		{
			xSemaphoreGiveFromISR(xJamSemaphore,&xHigherPriorityTaskWoken);
		}
		auto_pass_up = 0;
		auto_driver_up = 0;
	}
	// Driver Automatic Down Push Button is Pressed
	else if((DRIVER_PORT->DATA & (1 << DOWN_PB_D_AUTO)) == MODE_PB )
	{
		auto_driver_down = 1;
		xSemaphoreGiveFromISR(xDriverSemaphore,&xHigherPriorityTaskWoken);
	}
	// Driver Automatic Up Push Button is Pressed
	else if((DRIVER_PORT->DATA & (1 << UP_PB_D_AUTO)) == MODE_PB )
	{
		auto_driver_up = 1;
		xSemaphoreGiveFromISR(xDriverSemaphore,&xHigherPriorityTaskWoken);
	}
	// Driver Manual Down Push Button is Pressed
	else if((DRIVER_PORT->DATA & (1 << DOWN_PB_D_MAN)) == MODE_PB )
	{
		xSemaphoreGiveFromISR(xDriverSemaphore,&xHigherPriorityTaskWoken);
	}
	// Driver Manual Up Push Button is Pressed
	else if((DRIVER_PORT->DATA & (1 << UP_PB_D_MAN)) == MODE_PB )
	{
		xSemaphoreGiveFromISR(xDriverSemaphore,&xHigherPriorityTaskWoken);
	}


	// Clear Previous Interrupts
	GPIOA->ICR = 0xFC;
	i= GPIOA->ICR ;

	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

/*	External GPIO B Interrupt Service Routine	*/
void GPIOB_Handler(void)
{

	uint32_t i;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	// In case driver is operating passenger doesn't operate
	if(auto_driver_down || auto_driver_up || man_driver_down || man_driver_up)
	{}
	// Passenger Automatic Down Push Button is Pressed
	else if((PASS_PORT->DATA & (1 << DOWN_PB_P_AUTO)) == MODE_PB )
	{
		auto_pass_down =1;
		xSemaphoreGiveFromISR(xPassengerSemaphore,&xHigherPriorityTaskWoken);
		xTaskResumeFromISR(PassengerHandle);
	}
	// Passenger Automatic Up Push Button is Pressed
	else if((PASS_PORT->DATA & (1 << UP_PB_P_AUTO)) == MODE_PB )
	{
		auto_pass_up =1;
		xSemaphoreGiveFromISR(xPassengerSemaphore,&xHigherPriorityTaskWoken);
		xTaskResumeFromISR(PassengerHandle);
	}
	// Passenger Manual Down Push Button is Pressed
	else if((PASS_PORT->DATA & (1 << DOWN_PB_D_MAN)) == MODE_PB )
	{
		xSemaphoreGiveFromISR(xPassengerSemaphore,&xHigherPriorityTaskWoken);
		xTaskResumeFromISR(PassengerHandle);
	}
	// Passenger Manual Up Push Button is Pressed
	else if((PASS_PORT->DATA & (1 << UP_PB_D_MAN)) == MODE_PB )
	{
		xSemaphoreGiveFromISR(xPassengerSemaphore,&xHigherPriorityTaskWoken);
		xTaskResumeFromISR(PassengerHandle);
	}


	GPIOB->ICR = 0xF3;
	i= GPIOB->ICR ;

	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

/*	External GPIO F Interrupt Service Routine	*/
void GPIOF_Handler(void)
{

	uint32_t i;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	GPIOF->ICR = 0x11;
	i= GPIOF->ICR ;

	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
