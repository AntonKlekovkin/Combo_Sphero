#include "mbed.h"
#include "MPU6050_1.h"
#include "math.h"

#define NOM 6  //number of receiving bytes by UART

void Rx_Interrupt(void);
void time(void);
void obrabotka_bufera(void);
void str (float);
void motor1(int, float);
void motor2(int, float);
void IR_Enc_A1(void);
void IR_Enc_B1(void);
void IR_Enc_A2(void);
void IR_Enc_B2(void);
///////////////////////////////////////////////////////////////////////////////////////////////////
// for UART's bufer 


char buffer[NOM];   //bufer for receiving by UART

char flag_uart=0;  //flag for UART, if 1 information get
int timeout=0;        //timeout for UART
int timeout_set=3000;                         // setted value of timeout
int count_bufer=0; //counter of elements of UART's buffer
int n1=0, n2=0, systick_ms=0;
char metka=0;

MPU6050 mpu6050;
Timer t;
float sum = 0;
uint32_t sumCount = 0;

// int16_t ax, ay, az;
// int16_t gx, gy, gz;

// float accel_ugol_x, accel_ugol_y, accel_ugol_z;
// float gyro_ugol_x, gyro_ugol_y, gyro_ugol_z;
// float alpha_x, alpha_y, alpha_z;
// float alpha_x_old, alpha_y_old, alpha_z_old;

// VectorFloat accel(ax,ay,az);
// VectorFloat gyro(gx,gy,gz);

///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// for motor
DigitalOut motor_dir1(p25);
DigitalOut motor_dir11(p26);
PwmOut motor_speed1(p21);

DigitalOut motor_dir2(p15);
DigitalOut motor_dir22(p16);
PwmOut motor_speed2(p22);

InterruptIn Enc_A1(p29);
InterruptIn Enc_B1(p30);

InterruptIn Enc_A2(p18);
InterruptIn Enc_B2(p17);
///////////////////////////////////////////////////////////////////////////////////////////////////
// for LED
DigitalOut myled1(LED1);
DigitalOut myled3(LED3);
DigitalOut myled4(LED4);

PwmOut p(LED2);

Serial uart(p9, p10);  // tx, rx

Ticker tim;



///////////////////////////////////////////////////////////////////////////////////////////////////
void Rx_Interrupt()
{
    
    timeout=0;

    if(flag_uart==1)            
    {
        return;             //if buffer is at the work, then exit
    }

    //write receiving data to the buffer
    buffer[count_bufer] = uart.getc();

        
    //maximum is NOM bytes, if counter equal NOM, we got all bytes
    count_bufer++;
    if(count_bufer==NOM)
    {
        count_bufer=0;
        flag_uart=1;
        obrabotka_bufera();
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////
void time()
{
		
    static int i1=0, i2=0;
            
    i1++;
		i2++;

    if(i1==5)
    {
      myled1 = !myled1;
      i1=0;
    }
    
// 		if(i2==10)
// 		{
// 			mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
// 			
// 			accel.x=ax;
// 			accel.y=ay;
// 			accel.z=az;
// 			
// 			accel.normalize();
// 			
// 			gyro.x=gx;
// 			gyro.y=gy;
// 			gyro.z=gz;
// 			
// 			//gyro.normalize();
// 			
// 			accel_ugol_x = atan2(accel.x, sqrt(accel.y*accel.y + accel.z*accel.z)) * (180 / 3.1415926);
// 			accel_ugol_y = atan2(accel.y, sqrt(accel.x*accel.x + accel.z*accel.z)) * (180 / 3.1415926);
// 			accel_ugol_z = atan2(accel.z, sqrt(accel.x*accel.x + accel.y*accel.y)) * (180 / 3.1415926);
// 			
// 			alpha_x = (1 - kk) * (alpha_x_old + gyro.x * tim) + kk * (accel_ugol_x);
// 			alpha_y = (1 - kk) * (alpha_y_old + gyro.y * tim) + kk * (accel_ugol_y);
// 			alpha_z = (1 - kk) * (alpha_z_old + gyro.z * tim) + kk * (accel_ugol_z);
// 			
// 			str((int)alpha_x);
// 			uart.puts(",");
// 			str((int)alpha_y);
// 			uart.puts(",");
// 			str((int)alpha_z);
// 			uart.printf("\r\n");
// 			
// 			alpha_x_old=alpha_x;
// 			alpha_y_old=alpha_y;
// 			alpha_z_old=alpha_z;
// 			
// 			i2=0;
// 		}
		
		
		
    if(metka==1)
    {
        systick_ms++;
        str(systick_ms);
        uart.puts(",");
        str (n1);
        uart.puts(",");
        str (n2);
        uart.printf("\r\n\r\n");
    }
    
    
    
    
    //if we got something by UART4, we are waiting for timeout...
    //if(count_bufer != 0)
//    {
//        timeout++;
//        //if timeout is finished, we will get nothing 
//        if(timeout>timeout_set) 
//        {
//            count_bufer=0;
//            flag_uart=1;
//            timeout=0;
//            
//            obrabotka_bufera();
//        }
//    }
}

void str (float zn)
{ 
    int mas_zn[6];
    int i,minus=0,ZN;
    
    if(zn<0)
    {
      minus=1;
      zn*=-1;
    }
    
    ZN=(int)zn;
      
    for (i=5;i>=0;i--)
    {
      mas_zn[i]=ZN%10;
      ZN/=10;
    }
    
    if (minus==1)
    {
      mas_zn[0]=-3;
    }
    
    for (i=0;i<=5;i++)
    {
      uart.putc(mas_zn[i]+48);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void IR_Enc_A1()
{
    char A, B;
    
    A=Enc_A1;
    B=Enc_B1;

       if ((A==1)&&(B==0)) {n1++;}
       if ((A==0)&&(B==1)) {n1++;}

       if ((A==1)&&(B==1)) {n1--;}
       if ((A==0)&&(B==0)) {n1--;}
       
       str (n1);
       uart.printf("\r\n ");
       
}

void IR_Enc_B1()
{
    char A, B;
    
    A=Enc_A1;
    B=Enc_B1;

    if ((B==1)&&(A==1)) {n1++;}
    if ((B==0)&&(A==0)) {n1++;}
    if ((B==1)&&(A==0)) {n1--;}
    if ((B==0)&&(A==1)) {n1--;}
    
    str (n1);
    uart.printf("\r\n ");
}

void IR_Enc_A2()
{
    char A, B;
    
    A=Enc_A2;
    B=Enc_B2;

       if ((A==1)&&(B==0)) {n2++;}
       if ((A==0)&&(B==1)) {n2++;}

       if ((A==1)&&(B==1)) {n2--;}
       if ((A==0)&&(B==0)) {n2--;}
       
       str (n2);
       uart.printf("\r\n ");
       
}

void IR_Enc_B2()
{
    char A, B;
    
    A=Enc_A2;
    B=Enc_B2;

    if ((B==1)&&(A==1)) {n2++;}
    if ((B==0)&&(A==0)) {n2++;}
    if ((B==1)&&(A==0)) {n2--;}
    if ((B==0)&&(A==1)) {n2--;}
    
    str (n2);
    uart.printf("\r\n ");
}

///////////////////////////////////////////////////////////////////////////
// analysis of UART's data
//////////////////////////////////////////////////////////////////////////
void obrabotka_bufera(void)
{
    int i=0;
    
    float speed1, speed2;
    int dir1, dir2;
    
    if(flag_uart==1)            // if data is got by UART
    {
            if(buffer[0]==125)  // if we received speeds of motors
            {
                
                // wheel 1
                
                dir1=buffer[1];                
                speed1=(float)buffer[2]/100;
                
                // wheel 2
                dir2=buffer[3];                
                speed2=(float)buffer[4]/100;                                                              
                
                              
                if(buffer[5]==1)   //forward
                {
                    motor1(dir1, speed1);
                    motor2(dir2, speed2);
                    uart.putc(1);
                    myled4=1;
                    
                    metka=1;
                }
                else if(buffer[5]==0)  //stop
                {
                    motor1(0,0);
                    motor2(0,0);
                    uart.putc(0);
                    myled4=0;
                    
                    metka=0;
                    systick_ms=0;
                    n1=0;
                    n2=0;
                }
                
            }
            
            //clear bufer
            for(i=0;i<NOM;i++)
            {
                buffer[i]=0;
            }

            flag_uart=0;    //clear flag
        }
}
///////////////////////////////////////////////////////////////////////////////////////////////////
void motor1(int dir, float speed)
{
    if(dir==1)
    {
        motor_dir1=1;
        motor_dir11=0;  
    }
    else if(dir==2)
    {
        motor_dir1=0;
        motor_dir11=1;  
    }
    else if(dir==0)
    {
        motor_dir1=0;
        motor_dir11=0;  
    }
    
    motor_speed1=1-speed;   
}

void motor2(int dir, float speed)
{
    if(dir==1)
    {
        motor_dir2=1;
        motor_dir22=0;  
    }
    else if(dir==2)
    {
        motor_dir2=0;
        motor_dir22=1;  
    }
    else if(dir==0)
    {
        motor_dir2=0;
        motor_dir22=0;  
    } 
    motor_speed2=1-speed;   
}


///////////////////////////////////////////////////////////////////////////////////////////////////
int main() 
{
		
	
    Enc_A1.fall(&IR_Enc_A1);
    Enc_B1.fall(&IR_Enc_B1);
    
    Enc_A1.rise(&IR_Enc_A1);
    Enc_B1.rise(&IR_Enc_B1);
    
    Enc_A2.fall(&IR_Enc_A2);
    Enc_B2.fall(&IR_Enc_B2);
    
    Enc_A2.rise(&IR_Enc_A2);
    Enc_B2.rise(&IR_Enc_B2);
    
    tim.attach_us(&time, 100000);
    
    uart.baud(115200);
    //uart.format(8, Serial::Even, 1);
    uart.attach(&Rx_Interrupt);
    
    uart.putc(1);
    uart.printf("OK\r\n");
    
		i2c.frequency(400000);  // use fast (400 kHz) I2C   
  
		t.start();  
	
//		mpu.initialize();
/*
	// Read the WHO_AM_I register, this is a good test of communication
		uint8_t whoami = mpu6050.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
		uart.printf("I AM 0x%x\r\n", whoami); 
		uart.printf("I SHOULD BE 0x68\r\n");
			 
		if (whoami == 0x68) // WHO_AM_I should always be 0x68
		{  
			uart.printf("MPU6050 is online...\r\n");
			wait(1);
					
			mpu6050.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
			uart.printf("x-axis self test: acceleration trim within : "); uart.printf("%f", SelfTest[0]); uart.printf("% of factory value \r\n");
			uart.printf("y-axis self test: acceleration trim within : "); uart.printf("%f", SelfTest[1]); uart.printf("% of factory value \r\n");
			uart.printf("z-axis self test: acceleration trim within : "); uart.printf("%f", SelfTest[2]); uart.printf("% of factory value \r\n");
			uart.printf("x-axis self test: gyration trim within : "); uart.printf("%f", SelfTest[3]); uart.printf("% of factory value \r\n");
			uart.printf("y-axis self test: gyration trim within : "); uart.printf("%f", SelfTest[4]); uart.printf("% of factory value \r\n");
			uart.printf("z-axis self test: gyration trim within : "); uart.printf("%f", SelfTest[5]); uart.printf("% of factory value \r\n");
			wait(1);

			if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) 
			{
				mpu6050.resetMPU6050(); // Reset registers to default in preparation for device calibration
				mpu6050.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
				mpu6050.initMPU6050(); 
				uart.printf("MPU6050 initialized for active data mode....\r\n"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
				wait(2);
			}
			else
			{
				uart.printf("Device did not the pass self-test!\r\n");
			}
		}
		else
		{
			uart.printf("Could not connect to MPU6050: \r\n");
			uart.printf("%#x \n",  whoami);
			
			while(1) ; // Loop forever if communication doesn't happen
		}
		
*/		
		
    while(1) 
    {
// 			

// 			
// 			// If data ready bit set, all data registers have new data
//   if(mpu6050.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) 
// 	{  // check if data ready interrupt
//     mpu6050.readAccelData(accelCount);  // Read the x/y/z adc values
//     mpu6050.getAres();
//     
//     // Now we'll calculate the accleration value into actual g's
//     ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
//     ay = (float)accelCount[1]*aRes - accelBias[1];   
//     az = (float)accelCount[2]*aRes - accelBias[2];  
//    
//     mpu6050.readGyroData(gyroCount);  // Read the x/y/z adc values
//     mpu6050.getGres();
//  
//     // Calculate the gyro value into actual degrees per second
//     gx = (float)gyroCount[0]*gRes; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
//     gy = (float)gyroCount[1]*gRes; // - gyroBias[1];  
//     gz = (float)gyroCount[2]*gRes; // - gyroBias[2];   

//     tempCount = mpu6050.readTempData();  // Read the x/y/z adc values
//     temperature = (tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
//    }  
//    
//     Now = t.read_us();
//     deltat = (float)((Now - lastUpdate)/1000000.0f) ; // set integration time by time elapsed since last filter update
//     lastUpdate = Now;
//     
//     sum += deltat;
//     sumCount++;
//     
//     if(lastUpdate - firstUpdate > 10000000.0f) {
//      beta = 0.04;  // decrease filter gain after stabilized
//      zeta = 0.015; // increasey bias drift gain after stabilized
//     }
//     
//    // Pass gyro rate as rad/s
//     mpu6050.MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f);

//     // Serial print and/or display at 0.5 s rate independent of data rates
//     delt_t = t.read_ms() - count;
//     if (delt_t > 99)  // update LCD once per half-second independent of read rate
// 		{
// // 			uart.printf("ax = %f", 1000*ax); 
// // 			uart.printf(" ay = %f", 1000*ay); 
// // 			uart.printf(" az = %f  mg\r\n", 1000*az); 

// // 			uart.printf("gx = %f", gx); 
// // 			uart.printf(" gy = %f", gy); 
// // 			uart.printf(" gz = %f  deg/s\r\n", gz); 
// // 			
// // 			uart.printf(" temperature = %f  C\r\n", temperature); 
// // 			
// // 			uart.printf("q0 = %f\r\n", q[0]);
// // 			uart.printf("q1 = %f\r\n", q[1]);
// // 			uart.printf("q2 = %f\r\n", q[2]);
// // 			uart.printf("q3 = %f\r\n", q[3]);      
// 			
// 					
// 		// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
// 		// In this coordinate system, the positive z-axis is down toward Earth. 
// 		// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
// 		// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
// 		// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
// 		// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
// 		// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
// 		// applied in the correct order which for this configuration is yaw, pitch, and then roll.
// 		// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
// 			yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
// 			pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
// 			roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
// 			pitch *= 180.0f / PI;
// 			yaw   *= 180.0f / PI; 
// 			roll  *= 180.0f / PI;

// 	//    pc.printf("Yaw, Pitch, Roll: \n\r");
// 	//    pc.printf("%f", yaw);
// 	//    pc.printf(", ");
// 	//    pc.printf("%f", pitch);
// 	//    pc.printf(", ");
// 	//    pc.printf("%f\n\r", roll);
// 	//    pc.printf("average rate = "); pc.printf("%f", (sumCount/sum)); pc.printf(" Hz\n\r");

// 			 //uart.printf("Yaw, Pitch, Roll, dt: %f %f %f %d\r\n", yaw, pitch, roll, delt_t);
// 			 uart.printf("%f\r\n", roll);
// 			 //uart.printf("average rate = %f\r\n", (float) sumCount/sum);
//       
//       myled3= !myled3;
// 			count = t.read_ms(); 
// 			sum = 0;
// 			sumCount = 0; 
//     }
	}
}
