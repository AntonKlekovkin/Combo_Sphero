#include <lpc17xx.h>

#define NOM 6  //number of receiving bytes by UART

void Rx_Interrupt(void);
void obrabotka_bufera(void);
void SysTickStop(void);
void SysTickStart(void);

void Debug_Printf(float*);

extern float debug_buffer[200];
extern int debug_buffer_count=0;

char buffer[NOM];   //bufer for receiving by UART
extern Serial uart(p9, p10);  // tx, rx

char flag_uart=0;  //flag for UART, if 1 information get
int timeout=0;        //timeout for UART
int timeout_set=3000;                         // setted value of timeout
int count_bufer=0; //counter of elements of UART's buffer

extern My_motor Wheel(p25, p26, p21);
extern My_motor Rotor(p15, p16, p22);

extern DigitalOut myled1(LED1);
extern DigitalOut myled3(LED3);
extern DigitalOut myled4(LED4);

extern MPU9250 mpu9250;

void SysTickStop()
{
	SysTick->VAL   = 0UL;  
	SysTick->CTRL  &= ~(1<<0);
}
void SysTickStart()
{
	SysTick->VAL   = 0UL;       
	SysTick->CTRL  |= (1<<0);
}

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
///////////////////////////////////////////////////////////////////////////
// analysis of UART's data
//////////////////////////////////////////////////////////////////////////
void obrabotka_bufera(void)
{
    int i=0;
    
    if(flag_uart==1)            // if data is got by UART
    {
            if(buffer[0]==125)  // if we received speeds of motors
            {
                
                // wheel 1
                
                Wheel.Set_direction(buffer[1]);    
                //speed1=(float)buffer[2]/100;
								Wheel.Pid.setted_value=(float)buffer[2]*10;
                
                // wheel 2
								Rotor.Set_direction(buffer[3]);
								Rotor.Pid.setted_value=(float)buffer[4]*10;
                //dir2=buffer[3];                
                //speed2=(float)buffer[4]/100;                                                              
                
                
              
                if(buffer[5]==1)   //forward
                {
                  Wheel.flag_start=1;
//										if(Wheel.Pid.setted_value !=0)
//										{
//											Wheel.flag_start=1;
//										}
//										else
//										{
//											SysTickStop();
//											//Debug_Printf();
//											Wheel.Stop();
//											Wheel.Pid.Reset_on_zero();
//										}
//										
//										if(Rotor.Pid.setted_value !=0)
//										{
//											Rotor.flag_start=1;
//										}
//										else
//										{
//											Rotor.Stop();
//										}
										
										myled4=1;
                    
                    
                }
                else if(buffer[5]==0)  //stop
                {
                    //motor1(0,0);
                    //motor2(0,0);
                    //uart.putc(0);
                    
										SysTickStop();
										Debug_Printf(debug_buffer);
										Wheel.Stop();
										Wheel.Pid.Reset_on_zero();
										Rotor.Stop();
										Rotor.Pid.Reset_on_zero();
										
										
										myled4=0;
                    
                    //metka=0;
                    //systick_ms=0;
                    //n1=0;
                    //n2=0;
                }
                
            }
						else if(buffer[0]==150)  //Set PID coefficients
            {							
							Wheel.Pid.set_all_coeff((float)buffer[1]/100, (float)buffer[2]/1000, (float)buffer[3]/1000000);
							//wait_ms(2000);
							uart.printf("Set OK\r\n");
							uart.printf("kp=%f, kd=%f, ki=%f\r\n",Wheel.Pid.kp, Wheel.Pid.kd, Wheel.Pid.ki);
						}
						else if(buffer[0]==151)  //Transmit PID coefficients
            {							
							uart.printf("kp=%f, kd=%f, ki=%f\r\n",Wheel.Pid.kp, Wheel.Pid.kd, Wheel.Pid.ki);
						}
						
            else if(buffer[0]==160)  //Set coefficients of feedback
            {							
							//Wheel.Pid.k_theta = buffer[1]*10;
							Wheel.Pid.dzeta_wheel = (float)(buffer[1])/10;
							Rotor.Pid.dzeta_rotor = (float)(buffer[2])/100;
							Wheel.Pid.linear_speed = buffer[3]*1;
							//wait_ms(2000);
							uart.printf("Set OK\r\n");
							//uart.printf("k_theta=%f, k_phi=%f, linear_speed=%f\r\n",Wheel.Pid.k_theta, Rotor.Pid.k_phi, Wheel.Pid.linear_speed);
							uart.printf("dzeta_wheel=%f, dzeta_rotor=%f, linear_speed=%f\r\n",Wheel.Pid.dzeta_wheel, Rotor.Pid.dzeta_rotor, Wheel.Pid.linear_speed);
						}
						else if(buffer[0]==161)  //Transmit feedback coefficients
            {							
							//uart.printf("k_theta=%f, k_phi=%f, linear_speed=%f\r\n",Wheel.Pid.k_theta, Rotor.Pid.k_phi, Wheel.Pid.linear_speed);
							uart.printf("dzeta_wheel=%f, dzeta_rotor=%f, linear_speed=%f\r\n",Wheel.Pid.dzeta_wheel, Rotor.Pid.dzeta_rotor, Wheel.Pid.linear_speed);
							myled3=!myled3;
						}
						else if(buffer[0]==170)  //Transmit feedback coefficients
            {							
							mpu9250.InitMPU(&uart);
						}
						else if(buffer[0]==180)  //Transmit feedback coefficients
            {							
							Rotor.Pid.coeff=(float)(buffer[1])/100;
							uart.printf("coeff=%f\r\n",Rotor.Pid.coeff);
						}
						
            //clear bufer
            for(i=0;i<NOM;i++)
            {
                buffer[i]=0;
            }

            flag_uart=0;    //clear flag
        }
}

void Debug_Printf(float* buffer)
{
	int i;
		
	for(i=0;i<200;i++)
	{
		uart.printf("%f\r\n", &buffer[i] );	
	}

}

