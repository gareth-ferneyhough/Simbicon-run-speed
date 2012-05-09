/*
 *   This is an implementation of the planar character animation system presented in "SIMBICON: Simple Biped Locomotion Control"
 *   by Kangkang Yin, Kevin Loken and Michiel van de Panne. The purpose of this applet is to provide a simple demo to the aforementioned
 *   system.
 *
 */


import java.awt.image.BufferedImage;
import javax.swing.Timer;

public class Simbicon{// extends java.applet.Applet 
    //implements MouseListener, MouseMotionListener, KeyListener{
    Bip7 bip7 = new Bip7();
    Ground gnd = new Ground();
    private float Dt = 0.00005f;
    private float DtDisp = 0.0054f;
    private float timeEllapsed = 0;
        
    //and the controller
    Controller con;
    
    
    float Md, Mdd;
    
    
    float DesVel = 0;
    
    //if this variable is set to true, the simulation will be running, otherwise it won't
    boolean simFlag = false;
    
    // GF
    float last_foot_location;
    int last_state;
    int iteration_number;
    int the_cost_function;
    // GF
    
    boolean shouldPanY = false;
	private float total_steps_length;
	private int total_steps;
    
    
    public void init(	float torso0, float torso1, float torso2, 
    					float rhip0,  float rhip1,  float rhip2,
    					float rknee0, float rknee1, float rknee2,
    					//float lhip0,  float lhip1,  float lhip2, dont need
    					float lknee0, float lknee1, float lknee2,
    					float rankle0,float rankle1,float rankle2,
    					float lankle0,float lankle1,float lankle2,
    					float transTime
    				) 
    {       
        //GF
    	float lhip0 = 0f;
    	float lhip1 = 0f;
    	float lhip2 = 0f;
    	
        iteration_number  = 0;
        the_cost_function = 0;
        total_steps_length = 0;
        total_steps = 0;
        
        
        //initialize the biped to a valid state:
        float[] state = {0.463f, 0.98f, 0.898f, -0.229f, 0.051f, 0.276f, -0.221f, -1.430f, -0.217f, 0.086f, 0.298f, -3.268f, -0.601f, 3.167f, 0.360f, 0.697f, 0.241f, 3.532f};
        bip7.setState(state);
        
        int delay = 1; //milliseconds      
        
        initComponents();
        con = new Controller();
        con.addWalkingController();
        con.addRunningController();
        con.addCrouchWalkController();
              
        // GF
        
        // manually set controller params
        con.state[4].transTime = transTime;
        con.state[4].setThThDThDD(0, torso0,  torso1,  torso2 );		// torso
        con.state[4].setThThDThDD(1, rhip0,   rhip1,   rhip2  );		// rhip
        con.state[4].setThThDThDD(2, rknee0,  rknee1,  rknee2 );		// rknee
        con.state[4].setThThDThDD(3, lhip0,   lhip1,   lhip2  );		// lhip
        con.state[4].setThThDThDD(4, lknee0,  lknee1,  lknee2 );		// lknee
        con.state[4].setThThDThDD(5, rankle0, rankle1, rankle2);		// rankle
        con.state[4].setThThDThDD(6, lankle0, lankle1, lankle2);		// lankle
        
        // switch legs
        con.state[6].transTime = transTime;
        con.state[6].setThThDThDD(0, torso0,  torso1,  torso2 );		// torso
        con.state[6].setThThDThDD(1, lhip0,   lhip1,   lhip2  );		// rhip
        con.state[6].setThThDThDD(2, lknee0,  lknee1,  lknee2 );		// rknee
        con.state[6].setThThDThDD(3, rhip0,   rhip1,   rhip2  );		// lhip
        con.state[6].setThThDThDD(4, rknee0,  rknee1,  rknee2 );		// lknee
        con.state[6].setThThDThDD(5, lankle0, lankle1, lankle2);		// rankle
        con.state[6].setThThDThDD(6, rankle0, rankle1, rankle2);		// lankle
        
        con.desiredGroupNumber = 1; 
        simFlag = true;
        
        //bip7.PushTime = 20f;
        //bip7.PushForce = push_force;
        
        while(true){
        	if (!bip7.lostControl)  
        		runLoop();
        	else {
        		System.out.println("lost control");
        		System.exit(-1);
        	}
        }
        // GF
    }
    
    
    public float boundRange(float value, float min, float max) {
        if (value<min) 
            value=min;
        
        if (value>max) 
            value=max;
        return value;
    }

    //////////////////////////////////////////////////////////
    //  PROC: wPDtorq()
    //  DOES: computes requires torque to move a joint wrt world frame
    //////////////////////////////////////////////////////////
    public void wPDtorq(float torq[], int joint, float dposn, float kp, float kd, boolean world){
        float joint_posn = bip7.State[4 + joint*2];
        float joint_vel = bip7.State[4 + joint*2 + 1];
        if (world) {                   // control wrt world frame? (virtual)
            joint_posn += bip7.State[4];    // add body tilt
            joint_vel  += bip7.State[5];    // add body angular velocity
        }
        torq[joint] = kp*(dposn - joint_posn) - kd*joint_vel;
    }

    //////////////////////////////////////////////////////////
    // PROC:  jointLimit()
    // DOES:  enforces joint limits
    //////////////////////////////////////////////////////////
    public float jointLimit(float torq, int joint){
            float kpL=800;
            float kdL = 80;
            float minAngle = con.jointLimit[0][joint];
            float maxAngle = con.jointLimit[1][joint];
            float currAngle  = bip7.State[4 + joint*2];
            float currOmega = bip7.State[4 + joint*2 + 1];

            if (currAngle<minAngle)
                    torq = kpL*(minAngle - currAngle) - kdL*currOmega;
            else if (currAngle>maxAngle)
                    torq = kpL*(maxAngle - currAngle) - kdL*currOmega;
        return torq;
    }
	

    //////////////////////////////////////////////////////////
    //	PROC:	bip7WalkFsm(torq)
    //	DOES:	walking control FSM
    //////////////////////////////////////////////////////////
    public void bip7WalkFsm(float torq[]){
            int torsoIndex  = 0;
            int rhipIndex   = 1;
            int rkneeIndex  = 2;
            int lhipIndex   = 3;
            int lkneeIndex  = 4;
            int rankleIndex = 5;
            int lankleIndex = 6;
            boolean worldFrame[] = {
                    false,  // torso
                    true,   // rhip
                    false,  // rknee
                    true,   // lhip
                    false,  // lknee
                    false,  // rankle
                    false   // lankle
            };

            con.stateTime += Dt;
            ConState s = con.state[con.fsmState];

            computeMdMdd();
            for (int n=0; n<7; n++) {         // compute target angles for each joint
                    float target  = s.th[n] + Md*s.thd[n] + Mdd*s.thdd[n];         // target state + fb actions
                    target = boundRange(target, con.targetLimit[0][n], con.targetLimit[1][n]);    // limit range of target angle
                    wPDtorq(torq, n, target, con.kp[n], con.kd[n], worldFrame[n]);  // compute torques
            }

            con.advance(bip7);   	// advance FSM to next state if needed
    }

    //////////////////////////////////////////////////////////
    //	PROC:	bip7Control(torq)
    //	DOES:	calculates some primitive controlling torques
    //////////////////////////////////////////////////////////
    public void bip7Control(float torq[]){
            int body=0, stanceHip, swingHip;
            float fallAngle = 60;

            for (int n=0; n<7; n++)
                torq[n] = 0;

            // The following applies the control FSM.
            // As part of this, it computes the virtual fb torque for the
            // body, as implemented by a simple PD controller wrt to the world up vector
            if (!bip7.lostControl)  
                bip7WalkFsm(torq);

        // now change torq[body], which is virtual, 
        // to include a FEL feed-forward component

          // compute stance leg torque based upon body and swing leg
            if (con.state[con.fsmState].leftStance) {
                    stanceHip = 3;   // left hip
                    swingHip  = 1;   // right hip
            } else {
                    stanceHip = 1;   // right hip
                    swingHip  = 3;   // left hip
            }

            if (!con.state[con.fsmState].poseStance) {
                    torq[stanceHip] = -torq[body] - torq[swingHip];
            }
            torq[0] = 0;         // no external torque allowed !

            for (int n=1; n<7; n++) {  
                  torq[n] = boundRange(torq[n], con.torqLimit[0][n], con.torqLimit[1][n]);   // torq limits
                  jointLimit(torq[n],n);		                                     // apply joint limits
            }
    }    
    
    
    public void computeMdMdd(){
        float stanceFootX = bip7.getStanceFootXPos(con);
        Mdd = bip7.State[1] - DesVel;          // center-of-mass velocity error
        Md = bip7.State[0] - stanceFootX;      // center-of-mass position error
    }        
    
    public void initComponents(){
        //now we'll make a little of a GUI to let the user pause the simulation, etc.
    	// nottt
    }
    
    public void resetSimulation(){
                //toggle the sim flag
                bip7.resetBiped();

        	con.stateTime = 0;
        	con.fsmState = 0;
                con.currentGroupNumber = 0;
                con.desiredGroupNumber = 0;
                    
    }
    
    public void runLoop(){
        if (simFlag == false)
            return;
       
        //we'll run this a few times since the timer doesn't fire fast enough
        for (int i=0;i<200;i++){
            bip7.computeGroundForces(gnd);
            bip7Control(bip7.t);
            bip7.runSimulationStep(Dt);
            
            timeEllapsed += Dt;
            if (timeEllapsed>DtDisp){
             	
            	// GF
            	int state = con.fsmState;
            	if (state == 6 && last_state != 6){
            		float new_foot_location = bip7.getStanceFootXPos(con);
            		
            		// lets get up to speed? maybeee
            		if (iteration_number > 100)
            			updateStepLength(new_foot_location - last_foot_location);
            		
            		//System.out.println(new_foot_location - last_foot_location);
            		last_foot_location = new_foot_location; 
            	}   
            	last_state = con.fsmState;
                timeEllapsed = 0;
            }
        }
        iteration_number++;
        if(iteration_number >= 600){
        	int cost = calcCostFunction(iteration_number);
        	System.out.println(cost);
        	System.exit(cost);
        }
    }


	private void updateStepLength(float f) {
		total_steps_length += f;
		total_steps += 1;
	}


	private int calcCostFunction(int iteration_number) {
		float desired_avg_step_length = 2.0f;
		float avg_step_length = total_steps_length / total_steps;
		
		float step_error = (desired_avg_step_length - avg_step_length)*(desired_avg_step_length - avg_step_length);
				
		return (int) (step_error*10000);
	}
	
	
}