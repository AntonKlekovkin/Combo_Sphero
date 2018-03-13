#include <lpc17xx.h>
#include <math.h>

class My_pid
{
 
    protected:
 
    public:
			
		float kp, kd, ki;								// coefficients of PID
		float integral, err, old_err;		// integral component, errors
		float setted_value, real_value;	// 
		float out;											// output of PID
		float min_out, max_out;					// minimum and maximum values of output
		
		float k_theta, k_phi;						//coefficients of article feedback
		float out_torque_wheel; 				// output wheel torque for article feedback
		float	out_torque_rotor;					// output rotor torque for article feedback
		float linear_speed;
		float dzeta_wheel, dzeta_rotor;
		float coeff;
		
		//constructor of the class
//		My_pid(float kp_set, float kd_set, float ki_set)
//		{
//			kp=kp_set;
//			kd=kd_set;
//			ki=ki_set;
//			
//			integral=0;
//			err=0;
//			old_err=0;
//			
//			setted_value=0;
//			real_value=0;
//			
//			set_min_max(-50,50);
//		}
		
		//constructor of the class
		My_pid()
		{
			
			kp=0.01;
			kd=0.00000;
			ki=0.00000;
			
			integral=0;
			err=0;
			old_err=0;
			
			setted_value=0;
			real_value=0;
			
			k_theta=40;
			k_phi=0;
			linear_speed=2;
			dzeta_wheel=2;
			dzeta_rotor=0.1;
			
			set_min_max(-50,50);
			coeff=1;
		}
		
		//Setting new parameters
		void set_min_max(float min, float max)
		{
			min_out=min;
			max_out=max;
		}
		void set_kp(float new_kp)
		{
			kp=new_kp;
		}
		void set_kd(float new_kd)
		{
			kd=new_kd;
		}
		void set_ki(float new_ki)
		{
			ki=new_ki;
		}
		void set_all_coeff(float new_kp, float new_kd, float new_ki)
		{
			kp=new_kp;
			kd=new_kd;
			ki=new_ki;
		}
		
		//calculate of control action
		float Calculate_pid(float real_value1)
		{
			real_value=real_value1;
			err=setted_value-real_value;
			
			integral+=(err+old_err)/2;
			
			out=kp*err + kd*(err-old_err)+ ki*integral;
			
			if(out<min_out)
			{
				out=min_out;
			}
			if(out>max_out)
			{
				out=max_out;
			}
			
			old_err=err;
			
			return out;
		}
		
		float Calculate_feedback(float dot_theta)
		{
			int K_max = 1000;
			
			if(dzeta_wheel==0)
			{
				return setted_value;
			}			
			
			out_torque_wheel=(2*K_max/3.1415926)*atan( (-dot_theta + linear_speed)*dzeta_wheel) ;
								
			//out_torque_wheel = k_theta*( -dot_theta) + linear_speed;
			return out_torque_wheel;		
		
		}
		
		float Calculate_feedback_rotor(float dot_theta, float dot_phi, float w2, float w3)
		{
			
			int K_max = 1000;			
			
			//out_torque_rotor=k_phi*( w2 + w3 - 1*dot_phi);
			
			out_torque_rotor = -(2*K_max/3.1415926)*atan( (w2 + 1*w3 + 0.0*dot_phi)*dzeta_rotor) ;
			
			if(out_torque_rotor>0)
			{
				out_torque_rotor*=coeff;
			}
				
			return out_torque_rotor;
		
		}
		
		void Reset_on_zero()
		{
			integral=0;
			real_value=0;
			setted_value=0;
			out=0;
			err=0;
			old_err=0;		
			out_torque_wheel=0;
		}
		

};
