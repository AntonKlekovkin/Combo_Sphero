#include "mbed.h"
//#include "MPU6050_1.h"
#include "MPU9250.h"
#include "math.h"

#include "my_adc.h"
#include "my_timer.h"
#include "my_pid.h"
#include "my_motor.h"
#include "my_uart.h"
#include "my_init.h"


#define NOM 6  //number of receiving bytes by UART


void time(void);
void time2(void);

void str (float);
void motor1(int, float);
void motor2(int, float);

void sinchr(int32_t, int32_t, uint8_t);
extern "C" void RIT_IRQHandler();

void Debug_Printf(void);


float debug_buffer[200];
float debug_buffer2[200];
int debug_buffer_count=0;
int debug_buffer_count2=0;


float V_zero1=2045, V_zero2=2045;
float Current1_ma, Current2_ma;

//Serial uart(p9, p10);  // tx, rx



///////////////////////////////////////////////////////////////////////////////////////////////////
// for MPU
MPU9250 mpu9250;
Timer t;
float sum = 0;
uint32_t sumCount = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////
// for LED

PwmOut p(LED2);


DigitalOut test_p8(p8);


///////////////////////////////////////////////////////////////////////////////////////////////////
// for motor
//extern My_motor Wheel(p25, p26, p21);
//extern My_motor Rotor(p15, p16, p22);



int n1=0, n2=0;



///////////////////////////////////////////////////////////////////////////////////////////////////
extern "C" void TIMER0_IRQHandler(void) //Use extern "C" so C++ can link it properly, for C it is not required
{
	
		
	
	LPC_TIM0->IR |= (1<<0); //Clear MR0 Interrupt flag
}
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
int main() 
{
//		float deltat = 0.0f;                             // integration interval for both filter schemes
//		int lastUpdate = 0, firstUpdate = 0, Now = 0;    // used to calculate integration interval
//		int delt_t = 0; // used to control display output rate
//		int count = 0;  // used to control display output rate
	
     Enc_A1.fall(&IR_Enc_A1);
     Enc_B1.fall(&IR_Enc_B1);
     
     Enc_A1.rise(&IR_Enc_A1);
     Enc_B1.rise(&IR_Enc_B1);
     
     Enc_A2.fall(&IR_Enc_A2);
     Enc_B2.fall(&IR_Enc_B2);
     
     Enc_A2.rise(&IR_Enc_A2);
     Enc_B2.rise(&IR_Enc_B2);
	
		Wheel.speed.period_us(2000);
		
		init_adc(p17);
		init_adc(p18);
	
		
		SysTick_Config(SystemCoreClock/20000);	//20 kHz, 50 us
		SysTickStop();
		Wheel.Stop();
		Wheel.Pid.Reset_on_zero();
		Rotor.Stop();
		Rotor.Pid.Reset_on_zero();
		//NVIC_EnableIRQ(SysTick_IRQn);
		NVIC_SetPriority(SysTick_IRQn, 1); 
		
		myled4=1;
		//initTimer0();
		//rit_init();
        
    uart.baud(115200);
    //uart.format(8, Serial::Even, 1);
    uart.attach(&Rx_Interrupt);
    
    uart.putc(1);
    uart.printf("OK\r\n");	
	
//		Wheel.Set_direction(2);
//		Wheel.speed=0.5;
//		while(1);
//		wait(3);
//		Rotor.speed=0;
		
		t.start();  
		mpu9250.InitMPU(&uart);
		
    while(1) 
    {
			myled4=1;
			
			if(Wheel.flag_rotate==0 && Rotor.flag_rotate==0)
			{
				adc_start_read(2);
				V_zero1=adc_read_channel2()*0.00080586;	//8 us
				
				adc_start_read(3);
				V_zero2=adc_read_channel3()*0.00080586;	//8 us
				
				mpu9250.CalculateGyroData();
				
				debug_buffer[debug_buffer_count]=mpu9250.gx; //Wheel.Pid.out;
				debug_buffer2[debug_buffer_count]=mpu9250.gy;
				debug_buffer_count++;
				if(debug_buffer_count==200)
				{
					debug_buffer_count=0;
				}
				
				Wheel.omega=0;
				Rotor.omega=0;
				Wheel.pulse_enc=0;
				Wheel.Stop();
				Wheel.Pid.Reset_on_zero();
				Rotor.Stop();
				Rotor.Pid.Reset_on_zero();
				//Rotor.pulse_enc=0;
			}
			else
			{
				mpu9250.CalculateGyroData();
				Wheel.Pid.setted_value=Wheel.Pid.Calculate_feedback(-mpu9250.gz);
				
				//uart.printf("%f\r\n", mpu9250.gz);
				
				Rotor.Pid.setted_value = Rotor.Pid.Calculate_feedback_rotor(Wheel.omega, Rotor.omega, mpu9250.gx, mpu9250.gy);
				//uart.printf("%f %f\r\n", Wheel.Pid.setted_value, Rotor.Pid.setted_value);
			}
			
				
			if(Wheel.flag_start==1)
			{
				//Wheel.speed=0.5;
				Wheel.flag_rotate=1;
				uart.printf("V_zero=%f\r\n", V_zero1);
				wait_us(20);								
				SysTickStart();
				//wait(2);
				Wheel.flag_start=0;
			}
			
			myled4=0;
	
			wait_ms(20);
			
			
		}
}
		

extern "C" void SysTick_Handler (void) 
{
	static int i=0, i2=0;
	static float sum_current1=0, sum_current2=0;
	static int sum_count1=0, sum_count2=0;
	float out;
	float sp;
	
	test_p8=1;
	
	i++;
	i2++;
	
	//test_p8=1;
	//test_p8=0;
	
	adc_start_read(2);
	sum_current1+=(adc_read_channel2()*0.00080586 - V_zero1)*6622.5;
	
	adc_start_read(3);
	sum_current2+=(adc_read_channel3()*0.00080586 - V_zero2)*6622.5;
	
	sum_count1++;
	sum_count2++;
	
	
	if(i2==50)
	{
			
		Current1_ma=sum_current1/sum_count1;
		sum_current1=0;
		sum_count1=0;
				
		Wheel.speed=1-( (Wheel.Pid.Calculate_pid(Current1_ma))/100 + (1-Wheel.speed) );
		
//		debug_buffer[debug_buffer_count]=Current1_ma; //Wheel.Pid.out;
//		debug_buffer2[debug_buffer_count]=(1-Wheel.speed)*100;
//		debug_buffer_count++;
//		if(debug_buffer_count==200)
//		{
//			debug_buffer_count=0;
//		}
		
		Wheel.omega = (Wheel.pulse_enc*500*6.28/4480)*0.124138;
		Rotor.omega = Rotor.pulse_enc*500*6.28/3600;
		
		Wheel.pulse_enc=0;
		Rotor.pulse_enc=0;
		
		//i2=0;
	}	
	else if(i2==51)
	{
			
		Current2_ma=sum_current2/sum_count2;
		sum_current2=0;
		sum_count2=0;
		
		if(Rotor.direction==2)
		{
			sp=(1-Rotor.speed)*(-1);
		}
		else
		{
			sp=1-Rotor.speed;
		}
		out=Rotor.Pid.Calculate_pid(Current2_ma)/100 + sp;
		if(out<0)
		{
			out=out*(-1);
			Rotor.Set_direction(2);
			Rotor.speed=1-out;
		}
		else
		{
			Rotor.Set_direction(1);
			Rotor.speed=1-out;
		}
		//Rotor.speed=1-( Rotor.Pid.Calculate_pid(Current2_ma)/100 + (1-Rotor.speed) );
		
		debug_buffer[debug_buffer_count]=Current2_ma; //Wheel.Pid.out;
		debug_buffer2[debug_buffer_count]=(1-Rotor.speed)*100;
		debug_buffer_count++;
		if(debug_buffer_count==200)
		{
			debug_buffer_count=0;
		}
				
		i2=0;
	}	
	
	if(i==2000)
	{
		myled1 = !myled1;
		i=0;
	}
	
	test_p8=0;
}

void Debug_Printf()
{
	int i;
	float sum=0, average;
	
	for(i=0;i<200;i++)
	{
		uart.printf("I=%f PID=%f\r\n", debug_buffer[i],debug_buffer2[i] );	
		sum+=debug_buffer[i];
	}
	average=sum/200;
	uart.printf("V_zero=%f\r\n", V_zero1);	
	uart.printf("Average Current=%f\r\n", average);	
}
