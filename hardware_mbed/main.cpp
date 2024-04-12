#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "BezierCurve.h"
#include "MotorShield.h" 
#include "HardwareSetup.h"
#include "Matrix.h"
#include "MatrixMath.h"


// NOTE: To track when ball hits the tips, we can look at change in velocity. velocity usually spikes up greatly
// CHECK UPDATES TO CODE:  detecting ball
// changing set point
// changing k

//added flags llike ballCollided, ballLaunched,etc
#define BEZIER_ORDER_FOOT    7
#define NUM_INPUTS (12 + 2*(BEZIER_ORDER_FOOT+1))
#define NUM_OUTPUTS 19

#define PULSE_TO_RAD (2.0f*3.14159f / 1200.0f)

// Initializations
Serial pc(USBTX, USBRX);    // USB Serial Terminal
ExperimentServer server;    // Object that lets us communicate with MATLAB
Timer t;                    // Timer to measure elapsed time of experiment

QEI encoderA(PE_9,PE_11, NC, 1200, QEI::X4_ENCODING);  // MOTOR A ENCODER (no index, 1200 counts/rev, Quadrature encoding) arm 1
QEI encoderB(PA_5, PB_3, NC, 1200, QEI::X4_ENCODING);  // MOTOR B ENCODER (no index, 1200 counts/rev, Quadrature encoding) hand 1
QEI encoderC(PC_6, PC_7, NC, 1200, QEI::X4_ENCODING);  // MOTOR C ENCODER (no index, 1200 counts/rev, Quadrature encoding) arm 2
QEI encoderD(PD_12, PD_13, NC, 1200, QEI::X4_ENCODING);// MOTOR D ENCODER (no index, 1200 counts/rev, Quadrature encoding) hand 2

MotorShield motorShield(24000); //initialize the motor shield with a period of 12000 ticks or ~20kHZ
Ticker currentLoop;

Matrix MassMatrix(2,2);
Matrix Jacobian(2,2);
Matrix JacobianT(2,2);
Matrix InverseMassMatrix(2,2);
Matrix temp_product(2,2);
Matrix Lambda(2,2);

// Variables for q1(arm joint) (for both arms)
float current1;
float current_des1 = 0;
float prev_current_des1 = 0;
float current_int1 = 0;
float angle1;
float velocity1;
float duty_cycle1;
float angle1_init;

// Variables for q2 (hand joint) (for both arms)
float current2;
float current_des2 = 0;
float prev_current_des2 = 0;
float current_int2 = 0;
float angle2;
float velocity2;
float duty_cycle2;
float angle2_init;

// Fixed kinematic parameters (EDIT THESE FOR OUR PROJECT!!!!!)
// TODO: change these
const float l_OA=0.045; // make it half the size of the arm 
const float l_AB=0.093; // 
const float l_BC=0.107; // 
const float m1 =0.2684; 
const float m2 =.0338;

const float I1 = 0.0002189;  
const float I2 = 0.0000613;  

const float l_A_m1=0.0836;
const float l_B_m2=0.0484;

const float N = 18.75;
const float Ir = 0.0035/pow(N,2);


// Timing parameters
float current_control_period_us = 200.0f;     // 5kHz current control loop
float impedance_control_period_us = 1000.0f;  // 1kHz impedance control loop
float start_period, traj_period, end_period;

// Control parameters
float current_Kp = 4.0f;         
float current_Ki = 0.4f;           
float current_int_max = 3.0f;       
float duty_max;      
float K_xx;
float K_yy;
float K_xy;
float D_xx;
float D_xy;
float D_yy;

// Model parameters
float supply_voltage = 12;     // motor supply voltage
float R = 2.0f;                // motor resistance
float k_t = 0.18f;             // motor torque constant
float nu = 0.0005;             // motor viscous friction


// Flags for detecting robot state
bool ballCollided = false;
bool ballLaunched = false;



// Current control interrupt function
void CurrentLoop()
{
    // This loop sets the motor voltage commands using PI current controllers with feedforward terms.
    
    //use the motor shield as follows:
    //motorShield.motorAWrite(DUTY CYCLE, DIRECTION), DIRECTION = 0 is forward, DIRECTION =1 is backwards.
        

    // Calculations and motor commands for arm joint
    current1 = -(((float(motorShield.readCurrentA())/65536.0f)*30.0f)-15.0f);           // measure current
    velocity1 = encoderA.getVelocity() * PULSE_TO_RAD;                                  // measure velocity        
    float err_c1 = current_des1 - current1;                                             // current errror
    current_int1 += err_c1;                                                             // integrate error
    current_int1 = fmaxf( fminf(current_int1, current_int_max), -current_int_max);      // anti-windup
    float ff1 = R*current_des1 + k_t*velocity1;                                         // feedforward terms
    duty_cycle1 = (ff1 + current_Kp*err_c1 + current_Ki*current_int1)/supply_voltage;   // PI current controller
    
    float absDuty1 = abs(duty_cycle1);
    if (absDuty1 > duty_max) {
        duty_cycle1 *= duty_max / absDuty1;
        absDuty1 = duty_max;
    }    
    if (duty_cycle1 < 0) { // backwards
        motorShield.motorAWrite(absDuty1, 1);
        //motorShield.motorCWrite(absDuty1, 0); // opposite arm goes in reverse direction based on motor mounts
    } else { // forwards
        motorShield.motorAWrite(absDuty1, 0);
        //motorShield.motorCWrite(absDuty1, 1); // opposite arm goes in reverse direction based on motor mounts 
    }             
    prev_current_des1 = current_des1; 
    



    // Calculations and motor commands for hand joint

    current2     = -(((float(motorShield.readCurrentB())/65536.0f)*30.0f)-15.0f);       // measure current
    velocity2 = encoderB.getVelocity() * PULSE_TO_RAD;                                  // measure velocity  
    float err_c2 = current_des2 - current2;                                             // current error
    current_int2 += err_c2;                                                             // integrate error
    current_int2 = fmaxf( fminf(current_int2, current_int_max), -current_int_max);      // anti-windup   
    float ff2 = R*current_des2 + k_t*velocity2;                                         // feedforward terms
    duty_cycle2 = (ff2 + current_Kp*err_c2 + current_Ki*current_int2)/supply_voltage;   // PI current controller
    
    float absDuty2 = abs(duty_cycle2);
    if (absDuty2 > duty_max) {
        duty_cycle2 *= duty_max / absDuty2;
        absDuty2 = duty_max;
    }    
    if (duty_cycle2 < 0) { // backwards
        motorShield.motorBWrite(absDuty2, 1);
        //motorShield.motorDWrite(absDuty2, 0); // reverse direction based on motor mount orientation
    } else { // forwards
        motorShield.motorBWrite(absDuty2, 0);
        //motorShield.motorDWrite(absDuty2, 1); //reverse direction based on motor mount orientation
    }             
    prev_current_des2 = current_des2; 
    
}

int main (void)
{
    
    // Object for 7th order Cartesian foot trajectory
    BezierCurve rDesFoot_bez(2,BEZIER_ORDER_FOOT);
    
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();
    
    // Continually get input from MATLAB and run experiments
    float input_params[NUM_INPUTS];
    pc.printf("%f",input_params[0]);
    
    while(1) {
        
        // If there are new inputs, this code will run
        if (server.getParams(input_params,NUM_INPUTS)) {
            
                        
            // Get inputs from MATLAB          
            start_period                = input_params[0];    // First buffer time, before trajectory
            traj_period                 = input_params[1];    // Trajectory time/length
            end_period                  = input_params[2];    // Second buffer time, after trajectory
    
            angle1_init                 = input_params[3];    // Initial angle for q1 (rad)
            angle2_init                 = input_params[4];    // Initial angle for q2 (rad)

            K_xx                        = input_params[5];    // Foot stiffness N/m
            K_yy                        = input_params[6];    // Foot stiffness N/m
            K_xy                        = input_params[7];    // Foot stiffness N/m
            D_xx                        = input_params[8];    // Foot damping N/(m/s)
            D_yy                        = input_params[9];    // Foot damping N/(m/s)
            D_xy                        = input_params[10];   // Foot damping N/(m/s)
            duty_max                    = input_params[11];   // Maximum duty factor
          
            // Get foot trajectory points
            float foot_pts[2*(BEZIER_ORDER_FOOT+1)];
            for(int i = 0; i<2*(BEZIER_ORDER_FOOT+1);i++) {
                
              foot_pts[i] = input_params[12+i];    
            }
            rDesFoot_bez.setPoints(foot_pts);
            
            // Attach current loop interrupt
            currentLoop.attach_us(CurrentLoop,current_control_period_us);
                        
            // Setup experiment
            t.reset();
            t.start();
            encoderA.reset();
            encoderB.reset();
            encoderC.reset();
            encoderD.reset();

            motorShield.motorAWrite(0, 0); //turn motor A off
            motorShield.motorBWrite(0, 0); //turn motor B off
                         
            // Run experiment
            while( t.read() < start_period + traj_period + end_period) { 
                 
                // Read encoders to get motor states
                angle1 = encoderA.getPulses() *PULSE_TO_RAD + angle1_init;       
                velocity1 = encoderA.getVelocity() * PULSE_TO_RAD;
                 
                angle2 = encoderB.getPulses() * PULSE_TO_RAD + angle2_init;       
                velocity2 = encoderB.getVelocity() * PULSE_TO_RAD;           

                // NEW:
                // Based on observations, when the ball lands on hand tips, velocity of
                // corresponding motor spikes, and spikes more than due to the repeating disturbance from 
                // ethernet cable interference
                if (abs(velocity2) > 7.5 && ballLaunched == false && t.read() > 2)
                {
                    ballCollided = true;
                }

                if (ballCollided && abs(velocity2) < 1) // checking if ball is in process of getting caught
                {
                    // Now switch the set point. 
                    for(int i = 0; i<2*(BEZIER_ORDER_FOOT+1);i++)
                    {
                        if (i%2==0)
                        {
                            foot_pts[i] = input_params[12+i] + .03;
                        }
                        else if (i % 2 == 1)
                        {
                            foot_pts[i] = input_params[12+i] - .1;
                        }    
                    }
                    rDesFoot_bez.setPoints(foot_pts);
                    ballCollided = false; // ball should leave arms
                    ballLaunched = true;
                }



                
                const float th1 = angle1;
                const float th2 = angle2;
                const float dth1= velocity1;
                const float dth2= velocity2;
 
                // Calculate the Jacobian
                float Jx_th1 = - l_BC*sin(th1 + th2) - l_AB*sin(th1);
                float Jx_th2 = -l_BC*sin(th1 + th2);
                float Jy_th1 =  l_BC*cos(th1 + th2) + l_AB*cos(th1);
                float Jy_th2 = l_BC*cos(th1 + th2);
                                
                // Calculate the forward kinematics (position and velocity)
                float xFoot = l_OA + l_BC*cos(th1 + th2) + l_AB*cos(th1);
                float yFoot = l_BC*sin(th1 + th2) + l_AB*sin(th1);
                float dxFoot = dth1 * Jx_th1 + dth2 * Jx_th2;  
                float dyFoot = dth1 * Jy_th1 + dth2 * Jy_th2;      
  

                // Set gains based on buffer and traj times, then calculate desired x,y from Bezier trajectory at current time if necessary
                float teff  = 0;
                float vMult = 0;
                if( t < start_period) {
                    if (K_xx > 0 || K_yy > 0) {
                        K_xx = 100; 
                        K_yy = 100; 
                        D_xx = 5;  
                        D_yy = 5;  
                        K_xy = 0;
                        D_xy = 0;
                    }
                    teff = 0;
                }
                else if (t < start_period + traj_period)
                {
                    K_xx = input_params[5];  // Foot stiffness N/m
                    K_yy = input_params[6];  // Foot stiffness N/m
                    K_xy = input_params[7];  // Foot stiffness N/m
                    D_xx = input_params[8];  // Foot damping N/(m/s)
                    D_yy = input_params[9];  // Foot damping N/(m/s)
                    D_xy = input_params[10]; // Foot damping N/(m/s)
                    teff = (t-start_period);
                    vMult = 1;
                }
                else
                {
                    teff = traj_period;
                    vMult = 0;
                }
                
                // if ball launched
                if (ballLaunched)
                {
                    K_yy = K_yy*2;
                }



                // Get desired foot positions and velocities
                float rDesFoot[2] , vDesFoot[2];
                rDesFoot_bez.evaluate(teff/traj_period,rDesFoot);
                rDesFoot_bez.evaluateDerivative(teff/traj_period,vDesFoot);
                vDesFoot[0]/=traj_period;
                vDesFoot[1]/=traj_period;
                vDesFoot[0]*=vMult;
                vDesFoot[1]*=vMult;
                
                

                // Calculate error variables
                float e_x = rDesFoot[0] - xFoot;
                float e_y = rDesFoot[1] - yFoot;
                float de_x = vDesFoot[0] - dxFoot;
                float de_y = vDesFoot[1] - dyFoot;
        
                // Calculate virtual force on foot
                float fx = K_xx*e_x + K_xy * e_y + D_xx * de_x + D_xy*de_y;
                float fy = K_xy*e_x + K_yy * e_y + D_xy * de_x + D_yy*de_y;
                
                // Calculate mass matrix elements

            
                float M11 = I1 + I2 + Ir +Ir*pow(N, 2)+ m2*pow(l_AB,2) + 2*m2*cos(th2)*l_AB*l_B_m2 + m1*pow(l_A_m1,2) + m2*pow(l_B_m2,2) ;
                float M12 = I2 + Ir*N+ m2*pow(l_B_m2,2)+ l_AB*m2*cos(th2)*l_B_m2 ;
                float M22 = Ir*pow(N,2) + m2*pow(l_B_m2,2) + I2;



                // Populate mass matrix
                MassMatrix.Clear();
                MassMatrix << M11 << M12
                           << M12 << M22;
                // Populate Jacobian matrix
                Jacobian.Clear();
                Jacobian << Jx_th1 << Jx_th2
                         << Jy_th1 << Jy_th2;
                
                // Once you have copied the elements of the mass matrix, uncomment the following section
                
                // Calculate Lambda matrix
               JacobianT = MatrixMath::Transpose(Jacobian);
               InverseMassMatrix = MatrixMath::Inv(MassMatrix);
               temp_product = Jacobian*InverseMassMatrix*JacobianT;
               Lambda = MatrixMath::Inv(temp_product); 
                
                // Pull elements of Lambda matrix
               float L11 = Lambda.getNumber(1,1);
               float L12 = Lambda.getNumber(1,2);
               float L21 = Lambda.getNumber(2,1);
               float L22 = Lambda.getNumber(2,2);               
                                
                                
                                
                // Set desired currents             
                current_des1 = (L11 * Jx_th1 * fx + L12 * Jy_th1 * fy)/k_t;          
                current_des2 = (L21 * Jx_th2 * fx + L22 * Jy_th2 * fy)/k_t;   
                
                


                // Form output to send to MATLAB     
                float output_data[NUM_OUTPUTS];
                // current time
                output_data[0] = t.read();
                // motor 1 state
                output_data[1] = angle1;
                output_data[2] = velocity1;  
                output_data[3] = current1;
                output_data[4] = current_des1;
                output_data[5] = duty_cycle1;
                // motor 2 state
                output_data[6] = angle2;
                output_data[7] = velocity2;
                output_data[8] = current2;
                output_data[9] = current_des2;
                output_data[10]= duty_cycle2;
                // foot state
                output_data[11] = xFoot;
                output_data[12] = yFoot;
                output_data[13] = dxFoot;
                output_data[14] = dyFoot;
                output_data[15] = rDesFoot[0];
                output_data[16] = rDesFoot[1];
                output_data[17] = vDesFoot[0];
                output_data[18] = vDesFoot[1];
                
                // Send data to MATLAB
                server.sendData(output_data,NUM_OUTPUTS);

                wait_us(impedance_control_period_us);   
            }
            
            // Cleanup after experiment
            server.setExperimentComplete();
            currentLoop.detach();
            motorShield.motorAWrite(0, 0); //turn motor A off
            motorShield.motorBWrite(0, 0); //turn motor B off
        
        } // end if
        
    } // end while
    
} // end main
