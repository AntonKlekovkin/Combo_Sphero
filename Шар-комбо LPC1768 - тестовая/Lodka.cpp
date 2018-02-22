#include "mbed.h"
#define NOM 12  //number of receiving bytes by UART

void Rx_Interrupt(void);
void time(void);
void obrabotka_bufera(void);
void str (float);
void motor1(int, float);
void motor2(int, float);
void motor3(int, float);
void IR_Enc_A1(void);
//void IR_Enc_B1(void);
void IR_Enc_A2(void);
//void IR_Enc_B2(void);
void IR_Enc_A3(void);
void sinchr(int , int , int, int, int, int);
void reset_pid(void);
void razgon(int, int,int, int,int,int, int);
void delay(int);

void M1(int, int, int, int );
void M2(int, int, int, int );
void M3(int, int, int, int );

void M2M3(int, int, int, int, int, int);
void M1M2M3(int, int, int, int, int, int, int, int);

void Trajectory1(int);
void Trajectory2(int);
///////////////////////////////////////////////////////////////////////////////////////////////////
// for UART's bufer 


char buffer[NOM];   //bufer for receiving by UART

char flag_uart=0;  //flag for UART, if 1 information get
int timeout=0;        //timeout for UART
int timeout_set=30;                         // setted value of timeout
int count_bufer=0; //counter of elements of UART's buffer
volatile uint32_t n1=0, n2=0, n3=0, n11=0;
volatile char cnt=0, n33=0, n22=0;
char metka=0;


///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// for motors
DigitalOut motor_dir1(p5);
DigitalOut motor_dir11(p6);
PwmOut motor_speed1(p26);

DigitalOut motor_dir2(p7);
DigitalOut motor_dir22(p8);
PwmOut motor_speed2(p25);

DigitalOut motor_dir3(p27);
DigitalOut motor_dir33(p28);
PwmOut motor_speed3(p24);

InterruptIn Enc_A1(p13);
//InterruptIn Enc_B1(p14);

InterruptIn Enc_A2(p15);
//InterruptIn Enc_B2(p16);

InterruptIn Enc_A3(p17);
//InterruptIn Enc_B3(p18);

// for small motors
DigitalOut small_motor_dir1(p29);
DigitalOut small_motor_dir11(p30);
PwmOut small_motor_speed1(p23);

DigitalOut small_motor_dir2(p11);
DigitalOut small_motor_dir22(p12);
PwmOut small_motor_speed2(p22);

int current_omega1=0, current_omega2=0, current_omega3=0;
int flag_move=0;

int flag_tr=0;
///////////////////////////////////////////////////////////////////////////////////////////////////
// for LED
DigitalOut myled1(LED1);
PwmOut myled3(LED3);
DigitalOut myled4(LED4);

PwmOut myled2(LED2);

Serial uart(p9, p10);  // tx, rx

Ticker tim;

// For encoders
volatile int zad_pulse1=0,zad_pulse2=0,zad_pulse3=0;       //number of pulse encoder in a 0,01 second
int per_chislo=10;
int  pulse_enc=12, dt=50;                             // Constants
int omega=0;

int dt_r=10;

/////////////////////////////////////////////////////////////////////////////////
// for PID
volatile float integral1=0,integral2=0,integral3=0;
volatile signed int e01=0, e02=0,e03=0;    //old errors
volatile signed int e1,e2,e3;                   //new errors

//volatile float u=0;


// for delay
int delay_ms=0;

// for Analog Inputs for pressure's sensors
AnalogIn pressure1(p20);
AnalogIn pressure2(p19);
float pr1_volt, pr2_volt, pr1, pr2;
int pr1_send, pr2_send;
	
	
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
    
    uart.printf("\r\n");
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
void delay(int t)
{
	delay_ms=t;
	while (delay_ms!=0);
}

void SysTick_Handler(void) 
{
    static int i1=0, i2=0;
            
    i1++;
    i2++;

    if(i1==500)
    {
      myled4 = !myled4;
      i1=0;
    }
		
		
       
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void time()
{

    static int i1=0, i2=0, i3=0;
	
            
    i1++;
    i2++;
	  //i3++;

    if(i1==100)
    {
      myled1 = !myled1;
      i1=0;
			
			cnt++;
			uart.putc(cnt);
      uart.putc(n22);
			n22=0;
    }
       
    if(delay_ms != 0)
		{
			delay_ms--;
		}
    
    if(i2==dt)
    {
        if(flag_move==1 )
        {
					//if(n1!=0 && n2!=0)
					//{
            sinchr(n1,zad_pulse1, n2,zad_pulse2, n3, zad_pulse3);
            //sinchr(n2,zad_pulse2, 2);
            
            
            n1=0;
					  n2=0;
						n3=0;
					//}
            
        }

        i2=0;
    }
		
// 		if(i3==100)
//     {
// 			cnt++;
// 			uart.putc(cnt);
//       uart.putc((uint8_t)n33);
// 			n33=0;
//       i3=0;
//     }
    
		
		
		
		
   //if we got something by UART4, we are waiting for timeout...
   if(count_bufer != 0)
   {
       timeout++;
       //if timeout is finished, we will get nothing 
       if(timeout>timeout_set) 
       {
           count_bufer=0;
           flag_uart=1;
           timeout=0;
           
           obrabotka_bufera();
       }
   }
}

////////////////////////////////////////////////////////////////////////// 
// Synchronization
//////////////////////////////////////////////////////////////////////////
void sinchr(int cnt1, int ust_znach1, int cnt2, int ust_znach2, int cnt3, int ust_znach3)
{

    float temp;
    float u=0;
    
    
    
				e1=ust_znach1-cnt1;
	
        //integral1+=(e1+e01)/2;

        u=0.3*e1 + 3*(e1-e01);//+ 0.01*integral1;
    
					
        temp=(1-motor_speed1.read())*1000;
        temp+=u;
    
        if(temp>1000)
        {
            temp=1000;
        }
        if(temp<=0)
        {
            temp=0;
        }
    
        motor_speed1.pulsewidth_us(1000-temp);
    
        e01=e1;
        
    
        e2=ust_znach2-cnt2;
        //e2=e;
        //integral2+=(e2+e02)/2;

        u=0.3*e2 + 6*(e2-e02);//+ 0.01*integral2;
    
				//u=u/10000;
				
        temp=(1-motor_speed2.read())*1000;
        temp+=u;
    
        if(temp>1000)
        {
            temp=1000;
        }
        if(temp<=0)
        {
            temp=0;                  
        }
    
        
        motor_speed2.pulsewidth_us(1000-temp);
    
        e02=e2;
				
				
				e3=ust_znach3-cnt3;
        //e3=e;
        //integral3+=(e3+e03)/2;

        u=0.3*e3 + 6*(e3-e03);//+ 0.01*integral3;
    
				//u=u/10000;
				
        temp=(1-motor_speed3.read())*1000;
        temp+=u;
    
        if(temp>1000)
        {
            temp=1000;
        }
        if(temp<=0)
        {
            temp=0;                  
        }
    
        
        motor_speed3.pulsewidth_us(1000-temp);
    
        e03=e3;
        
       
    
}

void reset_pid()
{
    integral1=integral2=0;
    e1=e2=e3=e01=e02=e03=0;
    n1=0;
    n2=0;
	  n3=0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
void IR_Enc_A1()
{
  n1++;
	n11++;
}


void IR_Enc_A2()
{
	static int c=0;
	c++;
	
	n2++;
	if(c==10)	
	{
		n22++;
		c=0;
	}
}

void IR_Enc_A3()
{
	n3++;
	n33++;
}


///////////////////////////////////////////////////////////////////////////
// analysis of UART's data
//////////////////////////////////////////////////////////////////////////
void obrabotka_bufera(void)
{
    int i=0;
    
    float speed1, speed2, speed3;
    int dir1, dir2, dir3, omega1, omega2, omega3;
    int time=10;
	
    if(flag_uart==1)            // if data is got by UART
    {
            if(buffer[0]==125)  // if we received speeds of motors
            {
                
                // rotor 1
                
                dir1=buffer[1];                
                
                omega1=buffer[2]<<8;             // rotational velocity of the 1 wheel, RPM (revolution per minute)
                omega1+=buffer[3];
                speed1=(float)omega1;
                zad_pulse1=(omega1*pulse_enc*per_chislo*dt)/(60*1000*0.6);   //calculate number of pulse of encoder for time dt

                //uart.putc((char)zad_pulse1);
                
                // rotor 2
                dir2=buffer[4];                
                omega2=buffer[5]<<8;             // rotational velocity of the 1 wheel, RPM (revolution per minute)
                omega2+=buffer[6];
                speed2=(float)omega2;                                                              
                zad_pulse2=(omega2*pulse_enc*per_chislo*dt)/(60*1000*0.6);   //calculate number of pulse of encoder for time dt

							  // rotor 3
                dir3=buffer[7];                
                omega3=buffer[8]<<8;             // rotational velocity of the 1 wheel, RPM (revolution per minute)
                omega3+=buffer[9];
                speed3=(float)omega3;                                                              
                zad_pulse3=(omega3*pulse_enc*per_chislo*dt)/(60*1000);   //calculate number of pulse of encoder for time dt
								
								
                time=buffer[10];   
                          
                if(buffer[11]==1)   //forward
                {
									  flag_move=1;
									cnt=0;
//                     motor1(dir1, speed1);
//                     motor2(dir2, speed2);
// 									  motor3(dir3, speed3);
                    
                    
                    
                    razgon(dir1, omega1, dir2, omega2, dir3, omega3, time);
                    
                    
                    //motor_speed1=1-(speed1/2);
									  //motor_speed2=1-(speed2/2);
										//motor_speed3=1-(speed3/2);
                    
                    //uart.putc(buffer[7]);
                    
                    
                    
                }
                else if(buffer[11]==0)  //stop
                {
                    
                    razgon(dir1, omega1, dir2, omega2, dir3, omega3, time);
                    //myled4=0;
                    
                    //flag_move=0;
                    //systick_ms=0;
                    
									  
// 										motor1(0, 0);
//                     motor2(0, 0);
// 									  motor3(0, 0);
                    
									//myled2=1-motor_speed1;
                    //myled3=1-motor_speed2;
                    
                    //reset_pid();
                }
                
            }
						
            else if(buffer[0]==130)  // if we received speeds of motors
            {
							if(buffer[1]==1)
							{
								small_motor_dir1=1;
								small_motor_dir11=0;
								small_motor_speed1=1;
							}
							else if(buffer[1]==2)
							{
								small_motor_dir1=0;
								small_motor_dir11=1;
								small_motor_speed1=1;
							}
							else if(buffer[1]==0)
							{
								small_motor_dir1=0;
								small_motor_dir11=0;
								small_motor_speed1=0;
							}
							else if(buffer[1]==11)
							{
								small_motor_dir2=1;
								small_motor_dir22=0;
								small_motor_speed2=1;
							}
							else if(buffer[1]==12)
							{
								small_motor_dir2=0;
								small_motor_dir22=1;
								small_motor_speed2=1;
							}
							else if(buffer[1]==10)
							{
								small_motor_dir2=0;
								small_motor_dir22=0;
								small_motor_speed2=0;
							}
							
							else if(buffer[1]==21)
							{
								small_motor_dir1=1;
								small_motor_dir11=0;
								small_motor_speed1=1;
								
								small_motor_dir2=1;
								small_motor_dir22=0;
								small_motor_speed2=1;
							}
							else if(buffer[1]==22)
							{
								small_motor_dir1=0;
								small_motor_dir11=1;
								small_motor_speed1=1;
								
								small_motor_dir2=0;
								small_motor_dir22=1;
								small_motor_speed2=1;
							}
							else if(buffer[1]==20)
							{
								small_motor_dir1=0;
								small_motor_dir11=0;
								small_motor_speed1=0;
								
								small_motor_dir2=0;
								small_motor_dir22=0;
								small_motor_speed2=0;
							}
						}
						else if(buffer[0]==150) 
						{
							uart.putc(buffer[0]+1);
							flag_tr=1;
						}
						else if(buffer[0]==151) 
						{
							uart.putc(buffer[0]+1);
							flag_tr=2;
						}
						else if(buffer[0]==152) 
						{
							uart.putc(buffer[0]+1);
							flag_tr=3;
						}
						else if(buffer[0]==153) 
						{
							uart.putc(buffer[0]+1);
							flag_tr=4;
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
//Enable motors without bluetooth

void M1(int dir, int speed, int time, int motion)
{
	flag_uart=1;
	
	buffer[0]=125;
	buffer[1]=dir;
	
	buffer[2]=speed>>8;             
  buffer[3]=speed & 0xff;
	
	buffer[10]=time;
	buffer[11]=motion;
	
	obrabotka_bufera();
}

void M2(int dir, int speed, int time, int motion)
{
	flag_uart=1;
	
	buffer[0]=125;
	buffer[4]=dir;
	
	buffer[5]=speed>>8;             
  buffer[6]=speed & 0xff;
	
	buffer[10]=time;
	buffer[11]=motion;
	
	obrabotka_bufera();
}

void M3(int dir, int speed, int time, int motion)
{
	flag_uart=1;
	
	buffer[0]=125;
	buffer[7]=dir;
	
	buffer[8]=speed>>8;             
  buffer[9]=speed & 0xff;
	
	buffer[10]=time;
	buffer[11]=motion;
	
	obrabotka_bufera();
}

void M2M3(int dir1, int speed1, int dir2, int speed2, int time, int motion)
{
	flag_uart=1;
	
	buffer[0]=125;
	
	buffer[4]=dir1;
	buffer[5]=speed1>>8;             
  buffer[6]=speed1 & 0xff;
	
	buffer[7]=dir2;
	buffer[8]=speed2>>8;             
  buffer[9]=speed2 & 0xff;
	
	buffer[10]=time;
	buffer[11]=motion;
	
	obrabotka_bufera();
}

void M1M2M3(int dir1, int speed1, int dir2, int speed2, int dir3, int speed3, int time, int motion)
{
	flag_uart=1;
	
	buffer[0]=125;
	
	buffer[1]=dir1;
	buffer[2]=speed1>>8;             
  buffer[3]=speed1 & 0xff;
	
	buffer[4]=dir2;
	buffer[5]=speed2>>8;             
  buffer[6]=speed2 & 0xff;
	
	buffer[7]=dir3;
	buffer[8]=speed3>>8;             
  buffer[9]=speed3 & 0xff;
	
	buffer[10]=time;
	buffer[11]=motion;
	
	obrabotka_bufera();
}


void Trajectory1(int time)
{
	M3(1, 1000, 20, 1);
	
	delay(time);  

	M3(1, 0, 60, 0);
}

void Trajectory2(int time)
{
	M1M2M3(1, 1000, 1, 1000,1, 1000, 20, 1);
	
	delay(time);  

	M1M2M3(1, 0, 1, 0,1, 0, 60, 0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void razgon(int dir1, int omega1, int dir2, int omega2, int dir3, int omega3, int t)
{
    int om_r1=0, om_r2=0, om_r3=0;
	  int delta_omega1=0, delta_omega2=0, delta_omega3=0; 
    int i=0;
    //float	a=0.0;
    
		myled4=1;
	
    delta_omega1=(omega1-current_omega1)/(t+1);
	  delta_omega2=(omega2-current_omega2)/(t+1);
	  delta_omega3=(omega3-current_omega3)/(t+1);
	
	  om_r1=current_omega1+delta_omega1;
	  om_r2=current_omega2+delta_omega2;
	  om_r3=current_omega3+delta_omega3;
	  
	  //a = (1-motor_speed1.read())*1000;
		//a=a;
	
    motor1(dir1,(1-motor_speed1.read())*1000);
		motor2(dir2,(1-motor_speed2.read())*1000);
		motor3(dir3,(1-motor_speed3.read())*1000);
	
	  zad_pulse1=(om_r1*pulse_enc*per_chislo*dt)/(60*1000*0.6);   //calculate number of pulse of encoder for time dt
    zad_pulse2=(om_r2*pulse_enc*per_chislo*dt)/(60*1000*0.6);   //calculate number of pulse of encoder for time dt
		zad_pulse3=(om_r3*pulse_enc*per_chislo*dt)/(60*1000);
	
	  
	
		delay(100);   
	
    for(i=0;i<t;i++)
    {
        om_r1+=delta_omega1;
        om_r2+=delta_omega2;
			  om_r3+=delta_omega3;
        
        zad_pulse1=(om_r1*pulse_enc*per_chislo*dt)/(60*1000*0.6);   //calculate number of pulse of encoder for time dt
        zad_pulse2=(om_r2*pulse_enc*per_chislo*dt)/(60*1000*0.6);   //calculate number of pulse of encoder for time dt
			  zad_pulse3=(om_r3*pulse_enc*per_chislo*dt)/(60*1000);
        
        delay(100);
    }
		
		zad_pulse1=(omega1*pulse_enc*per_chislo*dt)/(60*1000*0.6);   //calculate number of pulse of encoder for time dt
    zad_pulse2=(omega2*pulse_enc*per_chislo*dt)/(60*1000*0.6);   //calculate number of pulse of encoder for time dt
		zad_pulse3=(omega3*pulse_enc*per_chislo*dt)/(60*1000);
		
		current_omega1=omega1;
		current_omega2=omega2;
		current_omega3=omega3;
		
		if(current_omega1==0)
		{
			motor1(0, 0);
			delay(100);
			n1=0;
      n11=0;
			e1=e01=0;
			zad_pulse1=0;
		}
		
		if(current_omega2==0)
		{
			motor2(0, 0);
			delay(100);
			n2=0;
      e2=e02=0;   
			zad_pulse2=0;			
		}
		
		if(current_omega3==0)
		{
			motor3(0, 0);
			delay(100);
   		n3=0;
			e3=e03=0;
			zad_pulse3=0;
		}
		if(current_omega1==0 && current_omega2==0 && current_omega3==0)
		{
			flag_move=0;
      //reset_pid();
		}
		
		myled4=0;
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
    
    motor_speed1.pulsewidth_us(1000-speed);
    
    
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
		
		motor_speed2.pulsewidth_us(1000-speed);

    
}

void motor3(int dir, float speed)
{
    if(dir==1)
    {
        motor_dir3=1;
        motor_dir33=0;  
    }
    else if(dir==2)
    {
        motor_dir3=0;
        motor_dir33=1;  
    }
    else if(dir==0)
    {
        motor_dir3=0;
        motor_dir33=0;  
    } 
		
		motor_speed3.pulsewidth_us(1000-speed);

    
}


///////////////////////////////////////////////////////////////////////////////////////////////////
int main() 
{
	uint32_t a;

	//int irqnum;
	//NVIC_EnableIRQ(SysTick_IRQn);
	//a=SysTick_Config((SystemCoreClock / 1000));
	
// 	if(a==0)
// 	{
// 		myled4=1;
// 	}
	
    Enc_A1.fall(&IR_Enc_A1);
    //Enc_B1.fall(&IR_Enc_B1);
    
    //Enc_A1.rise(&IR_Enc_A1);
    //Enc_B1.rise(&IR_Enc_B1);
    
    Enc_A2.fall(&IR_Enc_A2);
    //Enc_B2.fall(&IR_Enc_B2);
    
    //Enc_A2.rise(&IR_Enc_A2);
    //Enc_B2.rise(&IR_Enc_B2);
    
		Enc_A3.fall(&IR_Enc_A3);
	
    tim.attach_us(&time, 1000);
    
    uart.baud(115200);
    uart.format(8, Serial::Even, 1);
    uart.attach(&Rx_Interrupt);
    
    uart.putc(1);
    uart.printf("OK ");
    
    motor_speed1.period_us(1000);
    motor_speed2.period_us(1000);
		motor_speed3.period_us(1000);
    
		a=SystemCoreClock;
// enum FIRST_LAST_IRQ {		
// 		first_IRQ_number = SysTick_IRQn, // WDT_IRQn
//     last_IRQ_number  = CANActivity_IRQn,
// };
// 		
//     for(irqnum = first_IRQ_number ; irqnum < last_IRQ_number + 1 ; irqnum++)
// 		{
//         NVIC_SetPriority((IRQn_Type)irqnum, 0);
// 		}	
		
		NVIC_SetPriority(TIMER0_IRQn, 0);
		NVIC_SetPriority(TIMER1_IRQn, 0);
		NVIC_SetPriority(TIMER2_IRQn, 0);
		NVIC_SetPriority(TIMER3_IRQn, 0);
		
		NVIC_SetPriority(UART0_IRQn, 10);
		NVIC_SetPriority(UART1_IRQn, 10);
		NVIC_SetPriority(UART2_IRQn, 10);
		NVIC_SetPriority(UART3_IRQn, 10);

		motor1(1, 0);
    motor2(1, 0);
	  motor3(1, 0);
	 
	 	
	 while(1) 
    {
			
// 			pr1_volt=pressure1.read()*3.3;
// 			pr2_volt=pressure2.read()*3.3;
// 			
// 			pr1=(pr1_volt-0.2)/(0.45);
// 			if(pr1<0) pr1=0;
// 			
// 			pr2=(pr2_volt-0.2)/(0.45);
// 			if(pr2<0) pr2=0;
// 			
// 			pr1_send=(int)(pr1*36);
// 			pr2_send=(int)(pr2*36);
// 			
// 			uart.putc(50);
// 			uart.putc(pr1_send);
// 			//uart.putc(' ');
// 			uart.putc(pr2_send);
			
								
        if(flag_tr==1)
				{
					Trajectory1(10000);
					flag_tr=0;
				}
				else if(flag_tr==2)
				{
					Trajectory2(10000);
					flag_tr=0;
				}
				else if(flag_tr==3)
				{
					Trajectory1(30000);
					flag_tr=0;
				}
				else if(flag_tr==4)
				{
					Trajectory2(30000);
					flag_tr=0;
				}
				
				delay(1000);
        
    }
}
