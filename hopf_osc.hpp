/*

DS code

*/

#ifndef RHEX_DART_HOPF_OSC
#define RHEX_DART_HOPF_OSC

#include <cmath>

#include <string>
#include <fstream>
#include <streambuf>

#define PI 3.14159265

namespace rhex_dart {
	
	class HopfOsc {
		public:
		
		std::vector <double> UV;
		
		HopfOsc(){			
		}
		
		void init_UV(std::vector <double> UV_init){
			UV=UV_init;
		}			
		
		std::vector <double> del_U(std::vector <double> phi, double n, std::vector <double> UV){
			std::vector <double> del_U(5,0);//the feedback to U of an oscillator from all the other oscillators (needs to be 1 less than number of oscillators long)
			int y=0;
			for (unsigned x=0; x<12; x=x+2){     //x needs to be double than number of oscillators
				if(x!=n){
					del_U[y] = UV[x]*cos(phi[y]) - UV[x+1]*sin(phi[y]);
					y++;
				}
			}
			return del_U;
		}
	
		std::vector <double> del_V(std::vector <double> phi, double n, std::vector <double> UV){
			std::vector <double> del_V(5,0);//the feedback to U of an oscillator from all the other oscillators (needs to be 1 less than number of oscillators long)
			int y=0;
			for (unsigned x=0; x<12; x=x+2){     //x needs to be double than number of oscillators
				if(x!=n){
					del_V[y] = UV[x]*sin(phi[y]) + UV[x+1]*cos(phi[y]);
					y++;
				}
			}
			return del_V;
		}
	
		double CT_U(std::vector <double> del, double lam){
			double ct_U=0;//the feedback that is applied to U of each oscillator (needs to be number of oscillators long)
			for (unsigned n=0; n<5; n++){          //n needs to be 1 less than number of oscillators
				ct_U=ct_U+lam*del[n];
			}
			return ct_U;
		}

		double CT_V(std::vector <double> del, double lam){
			double ct_V=0;//the feedback that is applied to U of each oscillator (needs to be number of oscillators long)
			for (unsigned n=0; n<5; n++){          //n needs to be 1 less than number of oscillators
				ct_V=ct_V+lam*del[n];
			}
			return ct_V;

		}	
		
		
		//Function
		std::vector <double> func6(std::vector <double> UV, std::vector <double> phases, double Ep){
			double lam=1; //Lambda, coupling coefficient
			double A=1;  //Amplitude
			double Si=100; //Sigma, convergence speed
			std::vector <double> dUV(12);//the differentials
			std::vector <double> W(6);  //Oscillating frequency

			
			double N=270;  //Average rotating speed (RPM) 
			//double N=90;  //Average rotating speed (RPM)

			//
			//Finding the coupling feedback
			//
			std::vector <double> phi_1(5,0);     //vector of the phase differences from oscillator 1 (needs to be 1 less than number of oscillators long)
			std::vector <double> phi_2(5,0);     //vector of the phase differences from oscillator 2 (needs to be 1 less than number of oscillators long)
			std::vector <double> phi_3(5,0);     //vector of the phase differences from oscillator 3 (needs to be 1 less than number of oscillators long)
			std::vector <double> phi_4(5,0);     //vector of the phase differences from oscillator 4 (needs to be 1 less than number of oscillators long)
			std::vector <double> phi_5(5,0);     //vector of the phase differences from oscillator 5 (needs to be 1 less than number of oscillators long)
			std::vector <double> phi_6(5,0);     //vector of the phase differences from oscillator 6 (needs to be 1 less than number of oscillators long)

			//Calculating the different phases for the "phi"s, form give Phases
			//Phases from first oscillator
			phi_1[0]=phases[0];
			//std::cout<<"calculated(phi_1[0]):"<<phi_1[0]<<"   given:"<<phases[0]<<std::endl;
			phi_1[1]=phi_1[0]+phases[1];			
			//std::cout<<"calculated(phi_1[1]):"<<phi_1[1]<<std::endl;
			phi_1[2]=phi_1[1]+phases[2];
			//std::cout<<"calculated(phi_1[2]):"<<phi_1[2]<<std::endl;
			phi_1[3]=phi_1[2]+phases[3];
			//std::cout<<"calculated(phi_1[3]):"<<phi_1[3]<<std::endl;
			phi_1[4]=phi_1[3]+phases[4];
			//std::cout<<"calculated(phi_1[4]):"<<phi_1[4]<<std::endl;

			//Phases from second oscillator
			phi_2[0]=-phi_1[0];	
			//std::cout<<"calculated(phi_2[0]):"<<phi_2[0]<<std::endl;			
			phi_2[1]=phases[1];
			//std::cout<<"calculated(phi_2[1]):"<<phi_2[1]<<"   given:"<<phases[1]<<std::endl;
			phi_2[2]=phi_2[1]+phases[2];
			//std::cout<<"calculated(phi_2[2]):"<<phi_2[2]<<std::endl;
			phi_2[3]=phi_2[2]+phases[3];
			//std::cout<<"calculated(phi_2[3]):"<<phi_2[3]<<std::endl;
			phi_2[4]=phi_2[3]+phases[4];
			//std::cout<<"calculated(phi_2[4]):"<<phi_2[4]<<std::endl;

			//Phases from third oscillator
			phi_3[0]=-phi_1[1];
			//std::cout<<"calculated(phi_3[0]):"<<phi_3[0]<<std::endl;
			phi_3[1]=-phi_2[1];
			//std::cout<<"calculated(phi_3[1]):"<<phi_3[1]<<std::endl;
			phi_3[2]=phases[2];
			//std::cout<<"calculated(phi_3[2]):"<<phi_3[2]<<"   given:"<<phases[2]<<std::endl;
			phi_3[3]=phi_3[2]+phases[3];
			//std::cout<<"calculated(phi_3[3]):"<<phi_3[3]<<std::endl;
			phi_3[4]=phi_3[3]+phases[4];
			//std::cout<<"calculated(phi_3[4]):"<<phi_3[4]<<std::endl<<std::endl;

			//Phases from fourth oscillator
			phi_4[0]=-phi_1[2];
			//std::cout<<"calculated(phi_4[0]):"<<phi_4[0]<<std::endl;
			phi_4[1]=-phi_2[2];
			//std::cout<<"calculated(phi_4[1]):"<<phi_4[1]<<std::endl;
			phi_4[2]=-phi_3[2];
			//std::cout<<"calculated(phi_4[2]):"<<phi_4[2]<<std::endl;
			phi_4[3]=phases[3];
			//std::cout<<"calculated(phi_4[3]):"<<phi_4[3]<<"   given:"<<phases[3]<<std::endl;
			phi_4[4]=phi_4[3]+phases[4];
			//std::cout<<"calculated(phi_4[4]):"<<phi_4[4]<<std::endl<<std::endl;

			//Phases from fifth oscillator
			phi_5[0]=-phi_1[3];
			//std::cout<<"calculated(phi_5[0]):"<<phi_5[0]<<std::endl;
			phi_5[1]=-phi_2[3];
			//std::cout<<"calculated(phi_5[1]):"<<phi_5[1]<<std::endl;
			phi_5[2]=-phi_3[3];
			//std::cout<<"calculated(phi_5[2]):"<<phi_5[2]<<std::endl;
			phi_5[3]=-phi_4[3];
			//std::cout<<"calculated(phi_5[3]):"<<phi_5[3]<<std::endl;
			phi_5[4]=phases[4];
			//std::cout<<"calculated(phi_5[4]):"<<phi_5[4]<<"   given:"<<phases[4]<<std::endl<<std::endl;

			//Phases from sixth oscillator
			phi_6[0]=-phi_1[4];
			//std::cout<<"calculated:"<<phi_6[0]<<"   given:"<<phases[5]<<std::endl;
			phi_6[1]=-phi_2[4];			
			//std::cout<<"calculated(phi_6[1]):"<<phi_6[1]<<std::endl;
			phi_6[2]=-phi_3[4];			
			//std::cout<<"calculated(phi_6[2]):"<<phi_6[2]<<std::endl;
			phi_6[3]=-phi_4[4];
			//std::cout<<"calculated(phi_6[3]):"<<phi_6[3]<<std::endl;
			phi_6[4]=-phi_5[4];
			//std::cout<<"calculated(phi_6[4]):"<<phi_6[4]<<std::endl<<std::endl;

			std::vector <double> del_1U(5,0);  //the feedback to U of oscillator 1 from all the other oscillators (needs to be 1 less than number of oscillators long)
			std::vector <double> del_1V(5,0);  //the feedback to V of oscillator 1 from all the other oscillators (needs to be 1 less than number of oscillators long)
			std::vector <double> del_2U(5,0);  //the feedback to U of oscillator 2 from all the other oscillators (needs to be 1 less than number of oscillators long)
			std::vector <double> del_2V(5,0);  //the feedback to V of oscillator 2 from all the other oscillators (needs to be 1 less than number of oscillators long)
			std::vector <double> del_3U(5,0);  //the feedback to U of oscillator 3 from all the other oscillators (needs to be 1 less than number of oscillators long)
			std::vector <double> del_3V(5,0);  //the feedback to V of oscillator 3 from all the other oscillators (needs to be 1 less than number of oscillators long)
			std::vector <double> del_4U(5,0);  //the feedback to U of oscillator 4 from all the other oscillators (needs to be 1 less than number of oscillators long)
			std::vector <double> del_4V(5,0);  //the feedback to V of oscillator 4 from all the other oscillators (needs to be 1 less than number of oscillators long)
			std::vector <double> del_5U(5,0);  //the feedback to U of oscillator 5 from all the other oscillators (needs to be 1 less than number of oscillators long)
			std::vector <double> del_5V(5,0);  //the feedback to V of oscillator 5 from all the other oscillators (needs to be 1 less than number of oscillators long)
			std::vector <double> del_6U(5,0);  //the feedback to U of oscillator 6 from all the other oscillators (needs to be 1 less than number of oscillators long)
			std::vector <double> del_6V(5,0);  //the feedback to V of oscillator 6 from all the other oscillators (needs to be 1 less than number of oscillators long)

			//calculating the "del"s form the "phi"s
			del_1U=del_U(phi_1,0,UV);
			del_1V=del_V(phi_1,0,UV);
			del_2U=del_U(phi_2,2,UV);
			del_2V=del_V(phi_2,2,UV);
			del_3U=del_U(phi_3,4,UV);
			del_3V=del_V(phi_3,4,UV);
			del_4U=del_U(phi_4,6,UV);
			del_4V=del_V(phi_4,6,UV);
			del_5U=del_U(phi_5,8,UV);
			del_5V=del_V(phi_5,8,UV);
			del_6U=del_U(phi_6,10,UV);
			del_6V=del_V(phi_6,10,UV);

			std::vector <double> ct_u(6,0);  //the feedback that is applied to U of each oscillator (needs to be number of oscillators long)
			std::vector <double> ct_v(6,0);  //the feedback that is applied to V of each oscillator (needs to be number of oscillators long)

			//Adding all the "del"s for each oscillator, to give "ct_u" and "ct_v"
			ct_u[0]=CT_U(del_1U,lam);
			ct_u[1]=CT_U(del_2U,lam);
			ct_u[2]=CT_U(del_3U,lam);
			ct_u[3]=CT_U(del_4U,lam);
			ct_u[4]=CT_U(del_5U,lam);
			ct_u[5]=CT_U(del_6U,lam);

			ct_v[0]=CT_V(del_1V,lam);
			ct_v[1]=CT_V(del_2V,lam);
			ct_v[2]=CT_V(del_3V,lam);
			ct_v[3]=CT_V(del_4V,lam);
			ct_v[4]=CT_V(del_5V,lam);
			ct_v[5]=CT_V(del_6V,lam);

			//
			//Finding W, the frequency
			//
			int j=0;
			for(int i=1; i<12;i=i+2){
				if(UV[i]>=0){
					W[(j)]=N/((120.0)*(1-Ep));
				}else if(UV[i]<0){
					W[(j)]=N/((120.0)*Ep);
				}
				j++;
			}

			//
			//Find the differentials
			//
			for(int i=0; i<12;i=i+2){
			dUV[i]=((Si*UV[i]*(pow(A,2)-pow(UV[i],2)-pow(UV[i+1],2)))-(2*PI*W[(i/2)]*UV[i+1]))+(ct_u[i/2]);
			dUV[i+1]=((Si*UV[i+1]*(pow(A,2)-pow(UV[i],2)-pow(UV[i+1],2)))+(2*PI*W[(i/2)]*UV[i]))+(ct_v[i/2]);
			}

			return(dUV);
		}
		
		std::vector <double> update(double t_step, std::vector <double> Osc_phases, double Osc_Ep){
			//std::vector <double> dUV(12);			
			std::vector <double> dUV(12,0);
			
			std::vector <double> D_dot(6,0);
			double Phi_stance = 30;
			double Phi_swing = 360 - Phi_stance;
			double Phi_ST=Phi_stance*(PI/180.0);
			double Phi_SW=Phi_swing*(PI/180.0);
			
			dUV=func6(UV, Osc_phases, Osc_Ep);
			
			/*for(int m=1;m<12;m=m+2){
	    	   	if(UV[m]<=0){
	    	    	D_dot[((m-1)/2)]=(PI*(Phi_ST)/4)*cos((PI/2)*UV[(m-1)]*dUV[(m-1)]);
	    	    }else if(UV[m]>0){
	    	    	D_dot[((m-1)/2)]=(PI*(Phi_SW)/4)*cos((PI/2)*UV[(m-1)]*dUV[(m-1)]);
	    	    }
	    	}*/
			
			//Below is correct
			for(int m=1;m<12;m=m+2){
	    	   	if(UV[m]<=0){
	    	    	D_dot[((m-1)/2)]=dUV[(m-1)]*(PI*(Phi_ST)/4)*cos((PI/2)*UV[(m-1)]);
	    	    }else if(UV[m]>0){
	    	    	D_dot[((m-1)/2)]=-dUV[(m-1)]*(PI*(Phi_SW)/4)*cos((PI/2)*UV[(m-1)]);
	    	    }
	    	}
			
			for(int i=0;i<12;i++){
				UV[i]=UV[i]+t_step*(dUV[i]);
			}
			
			//std::vector <double> D(6);
			
			//double SWT=2*PI-Phi_ST;
			/*for(int m=1;m<12;m=m+2){
	    	    if(UV[m]<=0){
	    	    	D[((m-1)/2)]=(Phi_ST/2)*sin((PI/2)*UV[(m-1)])+(Phi_ST/2);
	    	    }else if(UV[m]>0){
	    	    	D[((m-1)/2)]=(-Phi_SW/2)*sin((PI/2)*UV[(m-1)])+(Phi_SW/2)+Phi_ST;
	    	    }
	    	}*/

	    	
			//returns the wanted velocities
			return(D_dot);
		}
	
		
	};
}

#endif //HOPF_OSC
