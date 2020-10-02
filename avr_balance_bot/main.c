/*
 * BalanceBot.c
 *
 *
 * Stepper 1 pins: PD2, PD3, PD4, PD5
 * Stepper 2 pins: PB0, PB1, PB2, PB3
 *
 * TWI for MPU-6050
 *
 * https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 * Author : Kim
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <math.h>
#include <stdlib.h>

#define SCL_CLOCK  100000L
#define BDIV (uint8_t)(F_CPU / SCL_CLOCK  - 16) / 2 + 1

#define MPU_ADDR 0x68
#define MPU_ACCEL_ADDR 0x3B
#define MPU_GYRO_ADDR 0x43

#define MIN_STEP_DELAY_MS (1.0)
#define MAX_PITCH_OFFSET (5.0)
#define P (MIN_STEP_DELAY_MS / MAX_PITCH_OFFSET)

#define GYRO_WEIGHT (0.96)
#define ACCELEROMETER_SENSITIVITY (1.0 / 16384.0)
#define GYROSCOPE_SENSITIVITY (1.0 / 131.0)
#define EST_DELTA_TIME get_delay_ms()

int16_t accel[3];
int16_t gyro[3];
float targetAngle = 15;
float pitch = 0;

void init_twi();
void start_TWI();
void stop_TWI();
uint8_t get_twi_status();

void init_mpu();
void get_mpu_data(uint8_t, int8_t *);
void calculate_pitch();
float get_accel_pitch();
uint8_t get_delay_ms();

int main(void)
{
	uint8_t i;
	// Initialize motor steps
	uint8_t step_val = 0b00000011;
	// Set stepper motor outputs
	DDRD |= 0b00111100;
	DDRB |= 0b00001111;
	
	// Initialize TWI and use TWI to initialize MPU
	init_twi();
	init_mpu();
	
	// Get initial pitch
	pitch = get_accel_pitch();
	
    while(1){
		// Get accel and gyro data from MPU-6050
		get_mpu_data(MPU_ACCEL_ADDR, (int8_t*) accel);
		get_mpu_data(MPU_GYRO_ADDR, (int8_t*) gyro);
		
		// Calculate pitch of robot, uses accel and gyro data
		calculate_pitch();

		// Sleep for as long as needed until next step
		for(i = 0; i < get_delay_ms(); i++){
			_delay_ms(1);
		}
		
		// Step step_val forwards or backwards depending on if tilted forwards or back
		if(pitch > targetAngle){
			step_val = ((step_val << 1) | (step_val >> 3)) & 0b1111;
		} else {
			step_val = ((step_val >> 1) | (step_val << 3)) & 0b1111;
		}

		// Map step_val to the appropriate output registers for each motor
	    	// Map the stepper state to ports PD2, PD3, PD4, and PD5
		PORTD &= ~0b00111100;
		PORTD |= (step_val << 2);
	    	// Map the stepper state to ports PB0, PB1, PB2, and PB3
		PORTB &= ~0b00001111;
		PORTB |= (step_val);
    }
}

void init_twi(){
	TWSR = 0;
	TWBR = BDIV;
}
void start_twi(){
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));
	if (get_twi_status() != TW_START) exit(1);
}
void stop_twi(){
	TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
	while((TWCR & (1<<TWSTO)));
}
void write_twi(uint8_t data, uint8_t success_status){
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	if(get_twi_status() != success_status) exit(2);
}
uint8_t read_twi(uint8_t success_status){
	TWCR = (1<<TWINT)|(1<<TWEN)|(((success_status == TW_MR_DATA_ACK) && 1)<<TWEA);
	while(!(TWCR & (1<<TWINT)));
	if(get_twi_status() != success_status) exit(3);
	return TWDR;
}
uint8_t get_twi_status(){
	uint8_t status;
	status = TWSR & 0xF8;
	return status;
}

void init_mpu(){
	start_twi();
	write_twi(MPU_ADDR << 1 | TW_WRITE, TW_MT_SLA_ACK);
	write_twi(0x6B, TW_MT_DATA_ACK); // Next write will be to power mgmt
	write_twi(0, TW_MT_DATA_ACK);
	stop_twi();
}
void get_mpu_data(uint8_t sensor_addr, int8_t *storage){
	start_twi();
	write_twi(MPU_ADDR << 1 | TW_WRITE, TW_MT_SLA_ACK);
	write_twi(sensor_addr, TW_MT_DATA_ACK); // Sensor start data
	stop_twi();
	start_twi();
	write_twi(MPU_ADDR << 1 | TW_READ, TW_MR_SLA_ACK);
	
	// Get 16-bit sensor data along the X, Y, and Z axes
	storage[1] = read_twi(TW_MR_DATA_ACK);
	storage[0] = read_twi(TW_MR_DATA_ACK);
	storage[3] = read_twi(TW_MR_DATA_ACK);
	storage[2] = read_twi(TW_MR_DATA_ACK);
	storage[5] = read_twi(TW_MR_DATA_ACK);
	storage[4] = read_twi(TW_MR_DATA_NACK);
	stop_twi();
}
float get_accel_pitch(){
	// Knowing the Z axis is up/down, use X and Y to determine pitch by to force
	// Is prone to overshoot, adjusted in calculate_pitch()
	return atan2f(accel[0], accel[1]) * 180.0 / 3.14159;
}
void calculate_pitch(){
	// http://www.pieter-jan.com/node/11
	pitch += gyro[2] * GYROSCOPE_SENSITIVITY * EST_DELTA_TIME / 1000.0;
	int forceMagnitude = abs(accel[0]) + abs(accel[1]) + abs(accel[2]);
	forceMagnitude *= ACCELEROMETER_SENSITIVITY;
	if(abs(forceMagnitude) < 2.0){
		// If there is not a lot of force, used to eliminate cases of external movement
		float p = get_accel_pitch();
		pitch = pitch * GYRO_WEIGHT + p * (1.0 - GYRO_WEIGHT);
	}
}
uint8_t get_delay_ms(){
	// Find the delay in stepper motor steps to make a speed
	float error = abs(targetAngle - pitch);
	float result = error * P;
	float reciprocal = 1.0 / fmax(0.01, result); // max is to avoid dividing by zero
	return fmax(MIN_STEP_DELAY_MS, reciprocal);
}
