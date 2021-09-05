// controlDLL.cpp : Defines the entry point for the DLL application.
//
#include "servo.h"
#include "param.h"
#include "control.h"
//#include "UiAgent.h"
#include "PrVector.h"
#include "PrMatrix.h"
#include "Utils.h" // misc. utility functions, such as toRad, toDeg, etc.
#include <math.h>
#include <algorithm>
using std::min;
using std::max;
float time_step = 0;
float time_offset = 0;
int i = 0;
PrMatrix TrajectoryP2;
PrMatrix TrajectoryP3;
int num_points = 360; // number of via points, should be greater 4


void PrintDebug(GlobalVariables& gv);

// *******************************************************************
// Initialization functions
// *******************************************************************

void InitControl(GlobalVariables& gv) 
{
   // This code runs before the first servo loop
}

void PreprocessControl(GlobalVariables& gv)
{
   // This code runs on every servo loop, just before the control law
   
   
  // 
    if ((gv.dof == 3) || (gv.dof == 6)) {

        //get the correct joint angles depending on the current mode:
        double q1,q2,q3;
        if (gv.dof == 3) {
            q1 = gv.q[0];
            q2 = gv.q[1];
            q3 = gv.q[2];
        } else if (gv.dof == 6) {
            q1 = gv.q[1];
            q2 = gv.q[2];
            q3 = gv.q[4];
        }

        PrVector3 g123 = PrVector3(0,0,0); //Variable that holds the torque exerted by gravity for each joint

        //Compute g123 here!  
        float r1 = R2;
        float r2 = 0.189738;
        float r3 = R6;
        
        float l1 = L2;
        float l2 = L3;
        float l3 = L6;
        
        float m1 = M2;
        float m2 = M3+M4+M5;
        float m3 = M6;
        float g = -9.81;
        
        float c1 = cos(q1);
        float c12 = cos(q1+q2-(M_PI/2));
        float c123 = cos(q1+q2+q3-(M_PI/2));
        
        g123[2]=-r3*c123*m3*g;
        g123[1] = -((l2*c12+r3*c123)*m3 + r2*c12*m2)*g;
        g123[0] = -((l1*c1+ l2*c12 + r3*c123)*m3+ (l1*c1+ r2*c12)*m2 + r1*c1*m1)*g;
        
        
 
        //maps the torques to the right joint indices depending on the current mode:
        if (gv.dof == 3) {
            gv.G[0] = g123[0];
            gv.G[1] = g123[1];
            gv.G[2] = g123[2];
        } else if (gv.dof == 6) {
            gv.G[1] = g123[0];
            gv.G[2] = g123[1];
            gv.G[4] = g123[2];
        }
        // printing example, do not leave print inthe handed in solution 
//        printVariable(g123, "g123");
    } else {
        gv.G = PrVector(gv.G.size());
    }   
}

void PostprocessControl(GlobalVariables& gv) 
{
   // This code runs on every servo loop, just after the control law
}

void initFloatControl(GlobalVariables& gv) 
{
	
    // Control Initialization Code Here
}

void initOpenControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjholdControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initJholdControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjmoveControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initJmoveControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjgotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initJgotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjtrackControl(GlobalVariables& gv) 
{
	time_offset = gv.curTime;

	//gv.curTime = 0;
	spline.t0 = time_offset;
	spline.tf = time_offset + computeTf(gv);
	//spline.a2[0] = PrVector3(0,0,0)[0];
	//std::cout << "tf is: " << spline.tf << std::endl;
	//std::cout << "tf - offset is: " << spline.tf-time_offset << std::endl;


	spline.a0 = gv.q;
	spline.a1 = PrVector3(0,0,0);
	spline.a2[0] = (3 / pow(spline.tf-time_offset, 2)) * (gv.qd[0] - gv.q[0]);
	spline.a2[1] = (3 / pow(spline.tf-time_offset, 2)) * (gv.qd[1] - gv.q[1]);
	spline.a2[2] = (3 / pow(spline.tf-time_offset, 2)) * (gv.qd[2] - gv.q[2]);
	spline.a3[0] = ((-2) / pow(spline.tf-time_offset, 3)) * (gv.qd[0] - gv.q[0]);
	spline.a3[1] = ((-2) / pow(spline.tf-time_offset, 3)) * (gv.qd[1] - gv.q[1]);
	spline.a3[2] = ((-2) / pow(spline.tf-time_offset, 3)) * (gv.qd[2] - gv.q[2]);
	//std::cout << "gv.qd is " << gv.qd[0] <<" "<< gv.qd[1] <<" "<< gv.qd[2] << std::endl;
}

void initJtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNxtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initXtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initNholdControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initHoldControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNgotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initGotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initNtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initTrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initPfmoveControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initLineControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initProj1Control(GlobalVariables& gv) 
{
	//(5.5 , 55.4 , -60.8 )
	gv.qd = PrVector3(0.096, 0.967, -1.061);
	initNjtrackControl(gv);
}

void initProj2Control(GlobalVariables& gv) 
{
	time_offset = gv.curTime;
	float circle_time = 5.0;
	i = 0;
	PrVector offset = PrVector(3);	
	time_step = circle_time/ num_points; //columns: timestamp, x, z,alpha

	float angle_step = 2*M_PI/num_points;
	
	TrajectoryP2[0][0] = 0 +time_offset;
	TrajectoryP2[0][1] = 0.8;
	TrajectoryP2[0][2] = 0.35;
	TrajectoryP2[0][3] = 0;
	
	
	for(unsigned i = 1; i < num_points; i++){ 
		TrajectoryP2[i][0] = i *time_step +time_offset;
		TrajectoryP2[i][1] = (cos(angle_step*i)*offset[2]) + offset[0];
		TrajectoryP2[i][2] = (sin(angle_step*i)*offset[2]) + offset[1];
		TrajectoryP2[i][3] = 0; // endefector should stay upright
				  
	}
	
//	gv.curTime = 0;
//	i = 0;
//	PrVector offset = PrVector(3);
//	offset[0] = 0.6; // offset in x direction due circle center at (0.6,0.35)
//	offset[1] = 0.35; // offset in z direction
//	offset[2] = 0.2 ; //radius of the circle
//
//
//
//
//	float circle_time = 5; // due to angular velocity 2Pi/5sek
//
//	TrajectoryP2 = PrMatrix(num_points, 4); // rows = number of points, columns: timestamp, x, z,alpha
//	time_step = circle_time/ num_points;
//	float angle_step = 2*M_PI/num_points;
//
//	TrajectoryP2[0][0] = 0;
//	TrajectoryP2[0][1] = 0.8;
//	TrajectoryP2[0][2] = 0.35;
//	TrajectoryP2[0][3] = 0;
//
//
//	for(unsigned i = 1; i < num_points; i++){
//		TrajectoryP2[i][0] = i *time_step;
//		TrajectoryP2[i][1] = (cos(angle_step*i)*offset[2]) + offset[0];
//		TrajectoryP2[i][2] = (sin(angle_step*i)*offset[2]) + offset[1];
//		TrajectoryP2[i][3] = 0; // endefector should stay upright
//
//	}

}

void initProj3Control(GlobalVariables& gv) 
{
	time_offset = gv.curTime;
	i = 0;
	PrVector offset = PrVector(3);
	offset[0] = 0.6; // offset in x direction due circle center at (0.6,0.35)
	offset[1] = 0.35; // offset in z direction
	offset[2] = 0.2 ; //radius of the circle
	float tb = 5;
	float tc = 15;
	int numb_points_3 = num_points*3;
	

	
	
	float circle_time = 5; // due to angular velocity 2Pi/5sek
	
	TrajectoryP3 = PrMatrix(numb_points_3, 4); // rows = number of points, columns: timestamp, x, z,alpha
	time_step = circle_time/ num_points;
	//time_step = 1;
	float angle_step = 2*M_PI/num_points;
	
	TrajectoryP3[0][0] = 0 +time_offset;
	TrajectoryP3[0][1] = 0.8;
	TrajectoryP3[0][2] = 0.35;
	TrajectoryP3[0][3] = 0;
	float v = 2*M_PI/5.;
	float a = 2*M_PI/25.;
	float ub = 1./2. * a * pow((5),2);
	float uc = v*(19) + ub;
	PrMatrix TrajectoryAngle;
	TrajectoryAngle = PrMatrix(numb_points_3, 2); // rows = number of points, columns: timestamp, angle
	
	
	for(unsigned i = 1; i < numb_points_3; i++){ 
		if(i *time_step < 5){
			TrajectoryAngle[i][0] = i *time_step;
			TrajectoryAngle[i][1] = 1./2. * a * pow((time_step*i),2);
			}
		else if(i *time_step >= 5 and i *time_step <= 19){
			TrajectoryAngle[i][0] = i *time_step;
			TrajectoryAngle[i][1] = v*(time_step*i-5) + ub;
		}
		else{
			TrajectoryAngle[i][0] = i *time_step;
			TrajectoryAngle[i][1] = -1./2. * a * pow((time_step*i-19),2)+ v*(time_step*i-19) +uc;
		}
				  
	}
	
	for(unsigned i = 1; i < numb_points_3; i++){ 
		TrajectoryP3[i][0] = i *time_step +time_offset;
		TrajectoryP3[i][1] = (cos(TrajectoryAngle[i][1])*offset[2]) + offset[0];		
		TrajectoryP3[i][2] = (sin(TrajectoryAngle[i][1])*offset[2]) + offset[1];
		TrajectoryP3[i][3] = 0; // endefector should stay upright
				  
	}
}


// *******************************************************************
// Control laws
// *******************************************************************

void noControl(GlobalVariables& gv)
{
}

void floatControl(GlobalVariables& gv)
{   
	
	// gv.tau = ?
	//gv.tau[0] = -10;
	// this only works on the real robot unless the function is changed to use cout
	// the handed in solution must not contain any printouts
//	PrintDebug(gv);
	        //maps the torques to the right joint indices depending on the current mode:
	gv.tau = -gv.G;
}

void openControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void njholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void jholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void njmoveControl(GlobalVariables& gv)
{
	// Ex:B.1 P-controller
	gv.tau = -gv.kp*(gv.q-gv.qd);
}

void jmoveControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void njgotoControl(GlobalVariables& gv) 
{	
   gv.tau = -gv.kp*(gv.q-gv.qd)- gv.G; 
}

void jgotoControl(GlobalVariables& gv) 
{
   
   gv.tau = 	-gv.kp*(gv.q-gv.qd)
   		        -gv.kv*gv.dq
   		        - gv.G;
}

void njtrackControl(GlobalVariables& gv) 
{
	float cT = gv.curTime - time_offset; // same for all the calculations
	//q_desired calculated from the spline parameters
	gv.qd[0] = spline.a0[0] + spline.a1[0]*cT + spline.a2[0]*pow(cT,2) + spline.a3[0]*pow(cT,3);
	gv.qd[1] = spline.a0[1] + spline.a1[1]*cT + spline.a2[1]*pow(cT,2) + spline.a3[1]*pow(cT,3);
	gv.qd[2] = spline.a0[2] + spline.a1[2]*cT + spline.a2[2]*pow(cT,2) + spline.a3[2]*pow(cT,3);

	//desired derivative of q calculated from the spline parameters
	gv.dqd[0] = 2*spline.a2[0]*cT + 3*spline.a3[0]*pow(cT,2);
	gv.dqd[1] = 2*spline.a2[1]*cT + 3*spline.a3[1]*pow(cT,2);
	gv.dqd[2] = 2*spline.a2[2]*cT + 3*spline.a3[2]*pow(cT,2);

	//gv.qd = PrVector3(0.096, 0.967, -1.061);
	if ( gv.curTime > spline.tf ){ //
		floatControl(gv);
	}else{

		gv.tau = 	-gv.kp*(gv.q-gv.qd)
		   		-gv.kv*(gv.dq-gv.dqd)
		   		-gv.G;
		//std::cout << "curT  "<< gv.curTime << ":  "<< gv.qd[0] << ", " << gv.qd[1] << ", "<< gv.qd[2] << std::endl;
	}
	//std::cout << gv.dqd[0] << ", "<< gv.dqd[1] << ", "<< gv.dqd[2] << std::endl;
}

void jtrackControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void nxtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void xtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void nholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void holdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void ngotoControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void gotoControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void ntrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void trackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void pfmoveControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void lineControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void proj1Control(GlobalVariables& gv) 
{
   gv.xd[0] = 0.8;
		gv.xd[1] = 0.35;
		gv.xd[2] = 0;
		PrVector F = - gv.kp*(gv.x - gv.xd)- gv.kv*(gv.dx-gv.dxd);
      		gv.tau = gv.Jtranspose*F - gv.G;  // Remove this line when you implement proj1Control
}




	
	


void proj2Control(GlobalVariables& gv) 
{	
		
	if (gv.dof == 3) {
		if(gv.curTime <= TrajectoryP2[i][0]){
		
		gv.xd[0] = TrajectoryP2[i][1];
		gv.xd[1] = TrajectoryP2[i][2];
		gv.xd[2] = TrajectoryP2[i][3];
		PrVector F = - gv.kp*(gv.x - gv.xd)- gv.kv*(gv.dx-gv.dxd);
      		gv.tau = gv.Jtranspose*F - gv.G;
      		
       	}
       	else{
       		if(i < num_points-1){
       		i += 1;
       		gv.xd[0] = TrajectoryP2[i][1];
			gv.xd[1] = TrajectoryP2[i][2];
			gv.xd[2] = TrajectoryP2[i][3];
			PrVector F = - gv.kp*(gv.x - gv.xd)- gv.kv*(gv.dx-gv.dxd);
      			gv.tau = gv.Jtranspose*F - gv.G;

       		}
       		else{
       			gv.xd[0] = TrajectoryP2[0][1];
				gv.xd[1] = TrajectoryP2[0][2];
				gv.xd[2] = TrajectoryP2[0][3];
       			PrVector F = - gv.kp*(gv.x - gv.xd)- gv.kv*(gv.dx-gv.dxd);
      				gv.tau = gv.Jtranspose*F - gv.G;
       		}

       
       	}
       
       }

	else if (gv.dof == 6) {
		floatControl(gv);
	}

}

void proj3Control(GlobalVariables& gv) 
{
   if (gv.dof == 3) {
	   
		if(gv.curTime < TrajectoryP3[i][0]){
		
		gv.xd[0] = TrajectoryP3[i][1];
		gv.xd[1] = TrajectoryP3[i][2];
		gv.xd[2] = TrajectoryP3[i][3];
		
		PrVector F = - gv.kp*(gv.x - gv.xd)- gv.kv*(gv.dx-gv.dxd);
      		gv.tau = gv.Jtranspose*F - gv.G;
      		
       	}
       	else{
			
       		if(i < num_points*3-1){
       		i += 1;
       		gv.xd[0] = TrajectoryP3[i][1];
			gv.xd[1] = TrajectoryP3[i][2];
			gv.xd[2] = TrajectoryP3[i][3];
			PrVector F = - gv.kp*(gv.x - gv.xd)- gv.kv*(gv.dx-gv.dxd);
      			gv.tau = gv.Jtranspose*F - gv.G;
       		}
			else {
       			gv.xd[0] = TrajectoryP3[0][1];
				gv.xd[1] = TrajectoryP3[0][2];
				gv.xd[2] = TrajectoryP3[0][3];
       			PrVector F = - gv.kp*(gv.x - gv.xd)- gv.kv*(gv.dx-gv.dxd);
      				gv.tau = gv.Jtranspose*F - gv.G;
       		}
       		

       
       	}
       
       }

	else if (gv.dof == 6) {
		floatControl(gv);
	}
}

// *******************************************************************
// Debug function
// *******************************************************************

void PrintDebug(GlobalVariables& gv)
{
   // Replace this code with any debug information you'd like to get
   // when you type "pdebug" at the prompt.
   //PrVector F = - gv.kp*(gv.x - gv.xd);
   
   std::cout << TrajectoryP2[2][2] << std::endl;
   printf( "This sample code prints the torque and mass\n" );
   //std::cout << gv.Jtranspose << std::endl;
   gv.tau.display( "tau" );
   gv.A.display( "A" );
   //gv.Jtranspose.display("J");
}

#ifdef WIN32
// *******************************************************************
// XPrintf(): Replacement for printf() which calls ui->VDisplay()
// whenever the ui object is available.  See utility/XPrintf.h.
// *******************************************************************

int XPrintf( const char* fmt, ... )
{
  int returnValue;
  va_list argptr;
  va_start( argptr, fmt );

  returnValue = vprintf( fmt, argptr );
 

  va_end( argptr );
  return returnValue;
}
#endif //#ifdef WIN32

/********************************************************

END OF DEFAULT STUDENT FILE 

ADD HERE ALL STUDENT DEFINED AND AUX FUNCTIONS 

*******************************************************/
