#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#define LOGIC_HIGH (1u)
#define LOGIC_LOW (0u)

#define SET_BIT(REG, BIT) (REG |= (1 << BIT))
#define CLEAR_BIT(REG, BIT) (REG &= (~(1 << BIT)))
#define TOGGLE_BIT(REG, BIT) (REG ^= (1 << BIT))
#define GET_BIT(REG, BIT) ((REG & (1 << BIT)) >> BIT)


void move_window_up();

void move_window_down();
	
void stop_motor();

char window_state();

void Delay_ms(int time_ms);


	
#endif

