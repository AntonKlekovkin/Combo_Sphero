#include "mbed.h"
#include "MPU9250.h"

extern MPU9250 mpu9250;
extern float SelfTest[6];
extern float gyroBias[3];
extern float accelBias[3];
extern float magCalibration[3];
extern uint8_t Ascale;
extern uint8_t Gscale;
extern uint8_t Mscale;
extern uint8_t Mmode;
extern float aRes, gRes, mRes;
extern float magbias[3];


void InitMPU(Serial uart)
{
	
	
	// Read the WHO_AM_I register, this is a good test of communication
		uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-6050
		uart.printf("I AM 0x%x\r\n", whoami); 
		uart.printf("I SHOULD BE 0x73\r\n");
			 
		if (whoami == 0x73) // WHO_AM_I should always be 0x73
		{  
			uart.printf("MPU9250 WHO_AM_I is 0x%x\n\r", whoami);
			uart.printf("MPU9250 is online...\n\r");
			
			wait(1);
			
			mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
			mpu9250.MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
			uart.printf("x-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[0]);  
			uart.printf("y-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[1]);  
			uart.printf("z-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[2]);  
			uart.printf("x-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[3]);  
			uart.printf("y-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[4]);  
			uart.printf("z-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[5]);  
			mpu9250.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
			uart.printf("x gyro bias = %f\n\r", gyroBias[0]);
			uart.printf("y gyro bias = %f\n\r", gyroBias[1]);
			uart.printf("z gyro bias = %f\n\r", gyroBias[2]);
			uart.printf("x accel bias = %f\n\r", accelBias[0]);
			uart.printf("y accel bias = %f\n\r", accelBias[1]);
			uart.printf("z accel bias = %f\n\r", accelBias[2]);
			wait(2);
			mpu9250.initMPU9250(); 
			uart.printf("MPU9250 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
			mpu9250.initAK8963(magCalibration);
			uart.printf("AK8963 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
			uart.printf("Accelerometer full-scale range = %f  g\n\r", 2.0f*(float)(1<<Ascale));
			uart.printf("Gyroscope full-scale range = %f  deg/s\n\r", 250.0f*(float)(1<<Gscale));
			if(Mscale == 0) uart.printf("Magnetometer resolution = 14  bits\n\r");
			if(Mscale == 1) uart.printf("Magnetometer resolution = 16  bits\n\r");
			if(Mmode == 2) uart.printf("Magnetometer ODR = 8 Hz\n\r");
			if(Mmode == 6) uart.printf("Magnetometer ODR = 100 Hz\n\r");
			wait(1);
   }
   else
   {
			uart.printf("Could not connect to MPU9250: \n\r");
			uart.printf("%#x \n",  whoami);
	 			 
			while(1) ; // Loop forever if communication doesn't happen
   }
		
		mpu9250.getAres(); // Get accelerometer sensitivity
    mpu9250.getGres(); // Get gyro sensitivity
    mpu9250.getMres(); // Get magnetometer sensitivity
    uart.printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f/aRes);
    uart.printf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f/gRes);
    uart.printf("Magnetometer sensitivity is %f LSB/G \n\r", 1.0f/mRes);
    magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
    magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
}
