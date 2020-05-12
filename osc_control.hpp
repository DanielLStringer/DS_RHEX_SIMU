/*

DS code

*/

#ifndef RHEX_DART_OSC_CONTROL
#define RHEX_DART_OSC_CONTROL

#include<rhex_dart/hopf_osc.hpp>
#include <rhex_dart/DS_rhex.hpp>

#include <string>
#include <fstream>
#include <streambuf>

namespace rhex_dart {
	
	class OscControl{
	public:
	
		using robot_t = std::shared_ptr<DS_Rhex>;
		
	
		double t_old;
		robot_t _robot;
		rhex_dart::HopfOsc _osc;
		
        OscControl() {}
		
		OscControl(robot_t robot)
			: _robot(robot)
		{	
					
			std::vector <double> UV_init (12,0);
			 // Initial Values
			UV_init[0]=1.5;
			UV_init[1]=0;			
			//UV_init[0]=0;
			//UV_init[1]=-1.5;			
			
			UV_init[2]=0;
			UV_init[3]=-1;
			//UV_init[2]=0;
			//UV_init[3]=-1.5;
		
			
			UV_init[4]=1.5;
			UV_init[5]=-1;
			
			UV_init[6]=1;
			UV_init[7]=0.5;			
			//UV_init[6]=1.5;  ////ERROR
			//UV_init[7]=-1;
			
			UV_init[8]=1;
			UV_init[9]=0;			
			//UV_init[8]=1;
			//UV_init[9]=-1.5;
			
			UV_init[10]=1;
			UV_init[11]=-1.5;
			//UV_init[10]=0;
			//UV_init[11]=-1;
			
			_osc.init_UV(UV_init);
		}
		
		void update(double t, std::vector <double> phases, double Ep){
			std::vector <double> velocities(6,0);
			double t_step= t-t_old;
			
			//std::cout<<"Time step of:   "<<t_step<<std::endl;
			
			velocities=_osc.update(t_step, phases, Ep);			
			
            size_t dof = _robot->skeleton()->getNumDofs();
			Eigen::VectorXd commands = Eigen::VectorXd::Zero(dof);
			  for(size_t i = 0; i < 6; ++i)
            {
                    commands[i] = 0.0;
            }
			
			//std::cout<<"DOFs:   "<<dof<<std::endl;
					commands[6]= velocities[0]; 
					commands[7]= velocities[2]; 
					commands[8]= velocities[4]; 
					commands[9]= velocities[1]; 
					commands[10]= velocities[3]; 
					commands[11]= velocities[5]; 
					
			_robot->skeleton()->setCommands(commands);
			t_old=t;
			
		}
			
			
	};
		
		
}

#endif //OSC_CONTROL