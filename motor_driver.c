#include "motor_driver.h"
#include "FreeRTOSConfig.h"
//digitalWrite(in1, HIGH);
//digitalWrite(in2, LOW);

void move_window_up(){
	
	SET_BIT((GPIOA->DATA),4);
	CLEAR_BIT((GPIOA->DATA),5);
	
}

//digitalWrite(in1, LOW);
//digitalWrite(in2, HIGH);
void move_window_down(){
	CLEAR_BIT((GPIOA->DATA),4);
	SET_BIT((GPIOA->DATA),5);
	
}
void stop_motor(){
	CLEAR_BIT((GPIOA->DATA),4);
	CLEAR_BIT((GPIOA->DATA),5);
}
	
char window_state(){
	int x = GET_BIT((GPIOA->DATA),4);
	int y = GET_BIT((GPIOA->DATA),5);
	
	if( x==0 & y==0){
		return 's';
	}
	else if(x==0 & y==1){
		return 'd';
	}
	else if(x==1 & y==0){
		return 'u';
	}
	
}

void Delay_ms(int time_ms)
{
    int i, j;
    for(i = 0 ; i < time_ms; i++)
        for(j = 0; j < 3180; j++)
            {
							/* excute NOP for 1ms */
						}  
}

