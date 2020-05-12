/*

DS code

*/

#ifndef RHEX_DART_ADAPT
#define RHEX_DART_ADAPT

#include <cmath>
#include <string>
#include <fstream>
#include <streambuf>

#define PI 3.14159265

namespace rhex_dart {

	class Adapt {
		public:
			
		Adapt() {			
		}
		
		
		
		/*std::vector <double> UV;
		
		void init_UV(std::vector <double> UV_init){
			UV=UV_init;
		}*/

		std::vector <double> fixedOut( ){						
			//Tripod gait
			//double Ep=0.5;
			std::vector<double> Ep_phases(7,0);
			
			Ep_phases[0]=0.5;			
			Ep_phases[1]=-1*PI;
			Ep_phases[2]=0;
			Ep_phases[3]=1*PI;
			Ep_phases[4]=0;
			Ep_phases[5]=-1*PI;
			Ep_phases[6]=1*PI;		
			
			return(Ep_phases);			
		}
		
		std::vector <double> fixedOut(std::vector<long unsigned int> contact, Eigen::Vector3d rot){		
			std::vector<double> Ep_phases(7,0);
			
			/*std::cout << "CONTACT:" << std::endl;	
			for (size_t i = (contact.size()-6); i < contact.size(); i++) {
				std::cout << contact[i] << " ";
			}
			std::cout << std::endl;*/
				
				
		
			std::cout<<"rot[0]: "<<rot[0]<<"  "<<"rot[1]: "<<rot[1]<<"  "<<"rot[2]: "<<rot[2]<<"  "<<std::endl;
			if(rot[2]>=(PI/2) || rot[2]<= (-PI/2)){
				fail_state_direction();
			}
						
					
			//Tripod gait
			/*Ep_phases[0]=0.5;			
			Ep_phases[1]=-1*PI;
			Ep_phases[2]=0;
			Ep_phases[3]=1*PI;
			Ep_phases[4]=0;
			Ep_phases[5]=-1*PI;
			Ep_phases[6]=1*PI;*/		
			
			//Quadruped gait
			/*Ep_phases[0]=2.0/3.0;			
			Ep_phases[1]=(-2.0/3.0)*PI;
			Ep_phases[2]=(-2.0/3.0)*PI;
			Ep_phases[3]=(4.0/3.0)*PI;
			Ep_phases[4]=(-2.0/3.0)*PI;
			Ep_phases[5]=(-2.0/3.0)*PI;
			Ep_phases[6]=(4.0/3.0)*PI;*/
			
			//Hill climbing (AS)
			/*Ep_phases[0]=0.5;			
			Ep_phases[1]=PI;
			Ep_phases[2]=-1*PI;
			Ep_phases[3]=PI;
			Ep_phases[4]=-1*PI;
			Ep_phases[5]=PI;
			Ep_phases[6]=-1*PI;	*/
			
			//Stair climbing (AS)
			Ep_phases[0]=0.5;			
			Ep_phases[1]=0;
			Ep_phases[2]=(-2.0/3.0)*PI;
			Ep_phases[3]=0;
			Ep_phases[4]=(-2.0/3.0)*PI;
			Ep_phases[5]=0;
			Ep_phases[6]=(4.0/3.0)*PI;

			//Wave gait			
			/*Ep_phases[0]=5.0/6.0;			
			Ep_phases[1]=-1*PI;
			Ep_phases[2]=(-1.0/3.0)*PI;
			Ep_phases[3]=1*PI;
			Ep_phases[4]=(-1.0/3.0)*PI;
			Ep_phases[5]=-1*PI;
			Ep_phases[6]=(5.0/3.0)*PI;*/
			
			return(Ep_phases);			
		}
		
		std::vector <double> InToOut(std::vector<long unsigned int> contact, Eigen::Vector3d rot){		
			std::vector<double> Ep_phases(7,0);
			
			/*std::cout << "CONTACT:" << std::endl;	
			for (size_t i = (contact.size()-6); i < contact.size(); i++) {
				std::cout << contact[i] << " ";
			}
			std::cout << std::endl;*/
				
				
		
			//std::cout<<"rot[0]: "<<rot[0]<<"  "<<"rot[1]: "<<rot[1]<<"  "<<"rot[2]: "<<rot[2]<<"  "<<std::endl;
			if(rot[2]>=(PI/2) || rot[2]<= (-PI/2)){
				fail_state_direction();
			}
			
			if(rot[0]<-(PI/24)|| rot[0]> (PI/24)){
				std::cout<<"rugged"<<std::endl;
				//tripod gait
				Ep_phases[0]=0.5;			
				Ep_phases[1]=-1*PI;
				Ep_phases[2]=0;
				Ep_phases[3]=1*PI;
				Ep_phases[4]=0;
				Ep_phases[5]=-1*PI;
				Ep_phases[6]=1*PI;
				return(Ep_phases);
			}
			
			//PI/12 is around 0.26
			//PI/6 is around 0.52
			
			//PI/24 is around 0.13
			if(rot[1]<-(PI/18)){
				//PI/9 is around 0.35
				if(rot[1]<-(PI/9)){
					if(rot[1]<-(PI/6)){
						std::cout<<"extreme slope"<<std::endl;
						//Stair climbing (AS)
						Ep_phases[0]=0.5;			
						Ep_phases[1]=0;
						Ep_phases[2]=(-2.0/3.0)*PI;
						Ep_phases[3]=0;
						Ep_phases[4]=(-2.0/3.0)*PI;
						Ep_phases[5]=0;
						Ep_phases[6]=(4.0/3.0)*PI;			
						return(Ep_phases);
					}	
					std::cout<<"stairs"<<std::endl;
					//Stair climbing (AS)
					Ep_phases[0]=0.5;			
					Ep_phases[1]=0;
					Ep_phases[2]=(-2.0/3.0)*PI;
					Ep_phases[3]=0;
					Ep_phases[4]=(-2.0/3.0)*PI;
					Ep_phases[5]=0;
					Ep_phases[6]=(4.0/3.0)*PI;			
					return(Ep_phases);
				}				
				std::cout<<"hill"<<std::endl;
				//Quadruped gait
				Ep_phases[0]=2.0/3.0;			
				Ep_phases[1]=(-2.0/3.0)*PI;
				Ep_phases[2]=(-2.0/3.0)*PI;
				Ep_phases[3]=(4.0/3.0)*PI;
				Ep_phases[4]=(-2.0/3.0)*PI;
				Ep_phases[5]=(-2.0/3.0)*PI;
				Ep_phases[6]=(4.0/3.0)*PI;
				return(Ep_phases);					
			}			
					
			//Tripod gait
			Ep_phases[0]=0.5;			
			Ep_phases[1]=-1*PI;
			Ep_phases[2]=0;
			Ep_phases[3]=1*PI;
			Ep_phases[4]=0;
			Ep_phases[5]=-1*PI;
			Ep_phases[6]=1*PI;		
			
			//Quadruped gait
			/*Ep_phases[0]=2.0/3.0;			
			Ep_phases[1]=(-2.0/3.0)*PI;
			Ep_phases[2]=(-2.0/3.0)*PI;
			Ep_phases[3]=(4.0/3.0)*PI;
			Ep_phases[4]=(-2.0/3.0)*PI;
			Ep_phases[5]=(-2.0/3.0)*PI;
			Ep_phases[6]=(4.0/3.0)*PI;*/
			
			//Hill climbing (AS)
			/*Ep_phases[0]=0.5;			
			Ep_phases[1]=PI;
			Ep_phases[2]=-1*PI;
			Ep_phases[3]=PI;
			Ep_phases[4]=-1*PI;
			Ep_phases[5]=PI;
			Ep_phases[6]=-1*PI;	*/
			
			//Stair climbing (AS)
			/*Ep_phases[0]=0.5;			
			Ep_phases[1]=0;
			Ep_phases[2]=(-2.0/3.0)*PI;
			Ep_phases[3]=0;
			Ep_phases[4]=(-2.0/3.0)*PI;
			Ep_phases[5]=0;
			Ep_phases[6]=(4.0/3.0)*PI;*/

			//Wave gait			
			/*Ep_phases[0]=5.0/6.0;			
			Ep_phases[1]=-1*PI;
			Ep_phases[2]=(-1.0/3.0)*PI;
			Ep_phases[3]=1*PI;
			Ep_phases[4]=(-1.0/3.0)*PI;
			Ep_phases[5]=-1*PI;
			Ep_phases[6]=(5.0/3.0)*PI;*/
			
			return(Ep_phases);			
		}
		
		void fitness(double covered_distance, double t, Eigen::Vector3d velocities){
			std::cout<<std::endl;
			std::cout<<"----------adapt fitness-----------"<<std::endl;
			std::cout<<"covered_distance:  "<<covered_distance<<std::endl;
			std::cout<<"time:  "<<t<<std::endl;
			std::cout<<"covered_distance/time:  "<<covered_distance/t<<std::endl;
			
			std::cout << "Avg COM Velocities" << std::endl;
			for (size_t i = 0; i < velocities.size(); i++) {
					std::cout << velocities[i] << " ";
			}
			std::cout << std::endl;
			std::cout<<"----------------------------------"<<std::endl<<std::endl;
		}
		
		void fail_state_colllision(){
			std::cout<<"error due to collision"<<std::endl; 
		}
		
		void fail_state_direction(){
			std::cout<<"error due to change in direction"<<std::endl; 
		}


	};
}

#endif //ADAPT
