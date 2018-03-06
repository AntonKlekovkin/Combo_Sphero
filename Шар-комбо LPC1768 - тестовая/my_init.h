#include <lpc17xx.h>

extern InterruptIn Enc_A1(p29);
extern InterruptIn Enc_B1(p30);

extern InterruptIn Enc_A2(p11);
extern InterruptIn Enc_B2(p12);

void IR_Enc_A1()
{
     char A, B;
     
     A=Enc_A1;
     B=Enc_B1;

        if ((A==1)&&(B==0)) {Wheel.pulse_enc++;}
        if ((A==0)&&(B==1)) {Wheel.pulse_enc++;}

        if ((A==1)&&(B==1)) {Wheel.pulse_enc--;}
        if ((A==0)&&(B==0)) {Wheel.pulse_enc--;}
                
        //uart.printf("%d\r\n", Wheel.pulse_enc);
        
 }

 void IR_Enc_B1()
{
     char A, B;
     
     A=Enc_A1;
     B=Enc_B1;

     if ((B==1)&&(A==1)) {Wheel.pulse_enc++;}
     if ((B==0)&&(A==0)) {Wheel.pulse_enc++;}
     if ((B==1)&&(A==0)) {Wheel.pulse_enc--;}
     if ((B==0)&&(A==1)) {Wheel.pulse_enc--;}
     
		 //uart.printf("%d\r\n", Wheel.pulse_enc);
}

 void IR_Enc_A2()
 {
     char A, B;
     
     A=Enc_A2;
     B=Enc_B2;

        if ((A==1)&&(B==0)) {Rotor.pulse_enc--;}
        if ((A==0)&&(B==1)) {Rotor.pulse_enc--;}

        if ((A==1)&&(B==1)) {Rotor.pulse_enc++;}
        if ((A==0)&&(B==0)) {Rotor.pulse_enc++;}
        
        //uart.printf("%d\r\n", Rotor.pulse_enc);
        
 }

 void IR_Enc_B2()
 {
     char A, B;
     
     A=Enc_A2;
     B=Enc_B2;

     if ((B==1)&&(A==1)) {Rotor.pulse_enc--;}
     if ((B==0)&&(A==0)) {Rotor.pulse_enc--;}
     if ((B==1)&&(A==0)) {Rotor.pulse_enc++;}
     if ((B==0)&&(A==1)) {Rotor.pulse_enc++;}
     
     //uart.printf("%d\r\n", Rotor.pulse_enc);
 }
