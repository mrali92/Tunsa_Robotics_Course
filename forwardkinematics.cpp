/*

 Please fill in the function stubs provided in this file

 For achieving full points on this exercise, explain the implemented equations
 in the accompanying pdf. Especially:
   - which parameters were used for computation?
   - were equations simplified? What was the original equation?
   - when introducing additional variables, what do they contain?


 * Do not use additional includes
 * Do not include/link additional files
 * Do not change the predefined function signature
 * Do not change the header file

 Tip: use the main() function to test your implementation.

*/

#include "forwardkinematics.hpp" //Have a look at this header file, it declares the class ForwardKinematicsPuma2D
#include <cmath>                 //use sin, cos from cmath for your computation
#include <iostream>
using namespace std;

#ifdef UNITTEST
    #define main STUDENTS_MAIN
#endif



/*
Convenience function to print out a homogenous transform
*/
void print_HTransform(HTransform tf)
{
    std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    std::cout.precision(3);
    cout << "\n---------------------------\n";

    cout << tf[0][0]<<"  "<<tf[0][1]<<"  "<<tf[0][2]<<"  "<<tf[0][3]<<endl;
    cout << tf[1][0]<<"  "<<tf[1][1]<<"  "<<tf[1][2]<<"  "<<tf[1][3]<<endl;
    cout << tf[2][0]<<"  "<<tf[2][1]<<"  "<<tf[2][2]<<"  "<<tf[2][3]<<endl;
    cout << tf[3][0]<<"  "<<tf[3][1]<<"  "<<tf[3][2]<<"  "<<tf[3][3]<<endl;
    cout <<   "---------------------------\n";
}

/*
Convenience function to print out a 3x3 Jacobian matrix
*/
void print_Jacobian(float Jacobian[3][3])
{
    std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    std::cout.precision(3);
    cout << "\n+++++++++++++++++++++++++++\n";
    cout << Jacobian[0][0]<<"  "<<Jacobian[0][1]<<"  "<<Jacobian[0][2]<<endl;
    cout << Jacobian[1][0]<<"  "<<Jacobian[1][1]<<"  "<<Jacobian[1][2]<<endl;
    cout << Jacobian[2][0]<<"  "<<Jacobian[2][1]<<"  "<<Jacobian[2][2]<<endl;
    cout <<   "+++++++++++++++++++++++++++\n";
}


/*
Convenience function to print out a position
*/
void print_Position(float F[3])
{
    std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    std::cout.precision(3);
    cout << "\nooooooooooooooooooooooooooo\n";
    cout << F[0]<<"  "<<F[1]<<"  "<< F[2]<<endl;
    cout <<   "ooooooooooooooooooooooooooo\n";
}


/*
already implemented

Set the robot's joint values and recompute the forward kinematics.

a1, a2 and a3 are assumed to be given in radians!
*/
void ForwardKinematicsPuma2D::setJoints(float a1, float a2, float a3)
{
    //store joint angles
    angles[0] = a1;
    angles[1] = a2;
    angles[2] = a3;
    c1 = cos(angles[0]);
    s1 = sin(angles[0]);
    c2 = cos(angles[1]);
    s2 = sin(angles[1]);
    c3 = cos(angles[2]);
    s3 = sin(angles[2]);
    c21 = cos(angles[0] - angles[1]);
    s21 = sin(angles[0] - angles[1]);
    //recompute the dependent variables
    computeDH();
    computeT0_1();
    computeT1_2();
    computeT2_3();
    computeT3_E();
    computeT0_E();
    computeF();
    computeJ();
}




/***********************************************/
/******************EDIT BELOW ******************/
/***********************************************/



/*
updates the variable T0_1

HT between frames 0 and 1
*/
void ForwardKinematicsPuma2D::computeT0_1()
{

   // compute from
   // angles[0], angles[1], angles[2], l1, l2, and l3

   //row vector
   T0_1[0][0] = c1;
   T0_1[0][1] = -s1;
   T0_1[0][2] = 0.0;
   T0_1[0][3] = 0.0;

   //row vector
   T0_1[1][0] = s1;
   T0_1[1][1] = c1;
   T0_1[1][2] = 0.0;
   T0_1[1][3] = 0.0;

   //row vector
   T0_1[2][0] = 0.0;
   T0_1[2][1] = 0.0;
   T0_1[2][2] = 1.0;
   T0_1[2][3] = 0.0;

   //row vector
   T0_1[3][0] = 0.0;
   T0_1[3][1] = 0.0;
   T0_1[3][2] = 0.0;
   T0_1[3][3] = 1.0;
}


/*
updates the variable T1_2

HT between frames 1 and 2
*/
void ForwardKinematicsPuma2D::computeT1_2()
{

   // compute from
   // angles[0], angles[1], angles[2], l1, l2, and l3

   //row vector
   T1_2[0][0] = c2;
   T1_2[0][1] = -s2;
   T1_2[0][2] = 0.0;
   T1_2[0][3] = l1;

   //row vector
   T1_2[1][0] = -s2;
   T1_2[1][1] = -c2;
   T1_2[1][2] = 0.0;
   T1_2[1][3] = 0.0;

   //row vector
   T1_2[2][0] = 0.0;
   T1_2[2][1] = 0.0;
   T1_2[2][2] = -1.0;
   T1_2[2][3] = 0.0;

   //row vector
   T1_2[3][0] = 0.0;
   T1_2[3][1] = 0.0;
   T1_2[3][2] = 0.0;
   T1_2[3][3] = 1.0;
}

/*
updates the variable T2_3

HT between frames 2 and 3
*/
void ForwardKinematicsPuma2D::computeT2_3()
{

   // compute from
   // angles[0], angles[1], angles[2], l1, l2, and l3

   //row vector
   T2_3[0][0] = c3;
   T2_3[0][1] = -s3;
   T2_3[0][2] = 0.0;
   T2_3[0][3] = l2;

   //row vector
   T2_3[1][0] = s3;
   T2_3[1][1] = c3;
   T2_3[1][2] = 0.0;
   T2_3[1][3] = 0.0;

   //row vector
   T2_3[2][0] = 0.0;
   T2_3[2][1] = 0.0;
   T2_3[2][2] = 1.0;
   T2_3[2][3] = 0.0;

   //row vector
   T2_3[3][0] = 0.0;
   T2_3[3][1] = 0.0;
   T2_3[3][2] = 0.0;
   T2_3[3][3] = 1.0;
}


/*
updates the variable T3_E

HT between frame 3 and EF
*/
void ForwardKinematicsPuma2D::computeT3_E()
{

   // compute from
   // angles[0], angles[1], angles[2], l1, l2, and l3

   //row vector
   T3_E[0][0] = 1.0;
   T3_E[0][1] = 0.0;
   T3_E[0][2] = 0.0;
   T3_E[0][3] = l3;

   //row vector
   T3_E[1][0] = 0.0;
   T3_E[1][1] = 1.0;
   T3_E[1][2] = 0.0;
   T3_E[1][3] = 0.0;

   //row vector
   T3_E[2][0] = 0.0;
   T3_E[2][1] = 0.0;
   T3_E[2][2] = 1.0;
   T3_E[2][3] = 0.0;

   //row vector
   T3_E[3][0] = 0.0;
   T3_E[3][1] = 0.0;
   T3_E[3][2] = 0.0;
   T3_E[3][3] = 1.0;
}


/*

This function updates the variable T0_E

HT between base frame and EF frame

*/
void ForwardKinematicsPuma2D::computeT0_E()
{

   // compute from
   // angles[0], angles[1], angles[2], l1, l2, and l3

   //row vector
   T0_E[0][0] = c21*c3;
   T0_E[0][1] = c3*(s21 - c21);
   T0_E[0][2] = 0.0;
   T0_E[0][3] = c21*(c3*l3 + l2) + s21*s3*l3 + c1*l1;

   //row vector
   T0_E[1][0] = s21*c3;
   T0_E[0][1] = -c3*(s21 + c21);
   T0_E[1][2] = 0.0;
   T0_E[1][3] = s21*(c3*l3 + l2) - s21*s3*l3 + s1*l1;

   //row vector
   T0_E[2][0] = 0.0;
   T0_E[2][1] = 0.0;
   T0_E[2][2] =-1.0;
   T0_E[2][3] = 0.0;

   //row vector
   T0_E[3][0] = 0.0;
   T0_E[3][1] = 0.0;
   T0_E[3][2] = 0.0;
   T0_E[3][3] = 1.0;
}



/*
This function updates the variables ee_x, ee_y, ee_alpha

Forward kinematics function derived from the HT

*/
void ForwardKinematicsPuma2D::computeF()
{

   // compute from
   // angles[0], angles[1], angles[2], l1, l2, and l3


   F[0] = c21*(c3*l3 + l2) + s21*s3*l3 + c1*l1; //x
   F[1] = s21*(c3*l3 + l2) - s21*s3*l3 + s1*l1; //y
   F[2] = -atan2(c21,s21); //alpha
 //  F[2] = angles[1]- angles[2]; //alpha
}


/*
This function updates the variable J

Jacobian matrix of the forward kinematics
*/
void ForwardKinematicsPuma2D::computeJ()
{

   // compute from
   // angles[0], angles[1], angles[2], l1, l2, and l3

   //row vector
   J[0][0] = -s21*(c3*l3 + l2) + c21*s3*l3 - s1*l1;
   J[0][1] =  s21*(c3*l3 + l2) - c21*s3*l3;
   J[0][2] = -s3*c21*l3 + c3*s21*l3;

   //row vector
   J[1][0] = c21*(c3*l3 + l2) + s21*s3*l3 - c1*l1;
   J[1][1] = c21*(c3*l3 + l2) - s21*s3*l3;
   J[1][2] = -s3*c21*l3 - c3*c21*l3;

   //row vector
   J[2][0] = -1.0;
   J[2][1] =  1.0;
   J[2][2] =  0.0;
}

/* This function computes the DH params
   and sets the internal member variable DH
DH parameters for the 3-DOF RRR
*/
void ForwardKinematicsPuma2D::computeDH()
{
   // compute from
   // angles[0], angles[1], angles[2], l1, l2, and l3


   //row vector
   DH[0][0] = 0.0; 
   DH[0][1] = 0.0; 
   DH[0][2] = 0.0; 
   DH[0][3] = angles[0];

   //row vector
   DH[1][0] = M_PI;
   DH[1][1] = l1;
   DH[1][2] = 0.0;
   DH[1][3] = angles[1];

   //row vector
   DH[2][0] = 0.0; 
   DH[2][1] = l2;
   DH[2][2] = 0.0; 
   DH[2][3] = angles[2];

   //row vector
   DH[3][0] = 0.0;
   DH[3][1] = l3;
   DH[3][2] = 0.0;
   DH[3][3] = 0.0;
}


/*
Example code to test your functions:

You are free to change main() as you like
*/
int main()
{
 ForwardKinematicsPuma2D* fk = new ForwardKinematicsPuma2D();
// fk->setJoints(0.0,0.0,0.0); //example, try out different values
 fk->setJoints(M_PI/4,-M_PI/4,M_PI/4);
 cout << "********************Testing Transforms**************"<<endl;
 print_HTransform(fk->T0_1);
 print_HTransform(fk->T1_2);
 print_HTransform(fk->T2_3);
 print_HTransform(fk->T3_E);
 print_HTransform(fk->T0_E);
 cout << "********************Testing F***********************"<<endl;
 print_Position(fk->F);
 cout << "********************Testing J***********************"<<endl;
 print_Jacobian(fk->J);
 return 0;
}
