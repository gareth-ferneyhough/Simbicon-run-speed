/*
 * Bip7.java
 *
 * Created on May 22, 2007, 6:22 PM
 *
 * To change this template, choose Tools | Template Manager
 * and open the template in the editor.
 */
import java.awt.Graphics;
import java.awt.Point;
import java.awt.Color;
import java.awt.Polygon;
import java.awt.geom.Point2D;
import java.awt.BasicStroke;
import java.awt.Graphics2D;

/**
 * This class acts as a container for the biped's state ( a 7-link planar biped) and implements some methods that are pertinent to
 * the biped (i.e. draw biped, etc).
 * @author Stelian
 */
public class Bip7 {
    //this is the number of states that the biped needs in order to be fully characterized:  - 7*2 for each link + 2*(root translation/vel)
    public final static int nrStates = 18;
    
    //this is the state of the biped
    public float State[] = new float[nrStates];
    //and we'll keep a copy of the default biped state, so that we can restart from it
    public float CopyState[] = new float[nrStates];
    
    
    //this is the array of accelerations of the biped - 9 accelerations: 7, 1 for each link, and 2 for CM x and y translational components
    public float ac[] = new float[9];
    //this is the array of joint torques that will be applied to the joints: 7 links, each of them has a joint actuator
    public float t[] = new float[7];
    //and this is the array of external forces that will be applied to the monitor points - can't remember how many mon points, so we'll just allocate space for plenty
    public float Fext[] = new float[50];
    //and this is the array of monitor point positions
    public float MonState[] = new float[50];
    //and this is an array that stores some info on the foot states - used to compute the forces due to the ground
    public float FootState[] = new float[50];
    //and this is how many monitor states we have
    final public static int NMON = 12;
    
    //this variable keeps track of whether or not we lost control of the biped
    boolean lostControl = false;
    
    //this variable is used to indicate if the biped is being pushed at the moment or not
    float PushTime; 
    //and the value of the push
    float PushForce;
    
    /** Creates a new instance of Bip7 */
    public Bip7() {
        for (int i=0;i<nrStates;i++)
            CopyState[i] = 0;
    }

    /**
        this method should be called to reset the biped
     */
    public void resetBiped(){
        //reset the biped here - we need a new state, FootState, etc... - look to see what else is needed
	for (int i=0; i<NMON*3; i++) {
		FootState[i] = 0;
	}
        for (int i=0;i<nrStates;i++){
            State[i] = CopyState[i];
        }
        lostControl = false;
        
    }
    
    
    float getStanceFootXPos(Controller con){
            int iLHeel = 8;   // x-coord of left-heel monitor point
            int iRHeel = 0;   // x-coord of right-heel monitor point
            if (con.state[con.fsmState].leftStance)
                    return MonState[iLHeel];
            else
                    return MonState[iRHeel];
    }

   
    public void loseControl(){
        lostControl = true;
    }
    
    
    public void runSimulationStep(float Dt){
        //it is assumed that when this step is called, the external forces and joint torques have already been computed
	t[0] = 0;							// no external torque allowed !
	dynBip7Eqns(State);	// compute accelerations
	bip7Integrate(Dt);				// integrate to update state
    }
    
    public void bip7Integrate(float Dt){
        //this method is used to update the positions and velocities for the biped using forward Euler
	for (int n=0; n<nrStates; n+=2) {
		State[n] += State[n+1]*Dt + 0.5*ac[n/2]*Dt*Dt;  // update position
		State[n+1] += ac[n/2]*Dt;                       // update velocity
	}
	if (State[4]>Math.PI)
		State[4] -= 2.0*Math.PI;
	if (State[4]<-Math.PI)
		State[4] += 2.0*Math.PI;    
        

        
        if (State[4]>2 || State[4]<-2 || State[6]>2.3 || State[6]<-2.3 || State[10]>2.3 || State[10]<-2.3)
            loseControl();
        
        
    }
    
    
    public void computeGroundForces(Ground gnd){
	int i1,i2,i3;	             // indices
	int pt;
        myPoint2D f = new myPoint2D();
        
	monBip7(State,MonState);      // monitor-point state computation
	for (pt=0; pt<NMON; pt++) {
		i1 = pt*4;				 // index for Mstate
		i2 = pt*3;		         // index for FootState
		i3 = pt*2;				 // index for fext

		float xMon = MonState[i1];
		float gndHeight = gnd.gndHeight( xMon );
		if (MonState[i1+2]>gndHeight) {		  // not in contact?
			FootState[i2] = 0.0f;
			Fext[i3]=0;
			Fext[i3+1]=0;
		} else {					/* if in contact */
			bip7_PointForce(i1,i2,gndHeight,f);
			if (f.y>0) {
				Fext[i3]   = f.x;
				Fext[i3+1] = f.y;
			} else {
				Fext[i3] = 0;
				Fext[i3+1] = 0;
			}
		}
	}

	if (PushTime>0.0) {
		PushTime -= 0.00005;     // subtract from duration
		pt = 10;
		i3 = pt*2;				 // index for fext
		Fext[i3] = PushForce;    // x-component
	} 

    }
    
    /** This method is used to compute penalty forces that come about due to ground contact at the monitor states */
    public void bip7_PointForce(int i1,int i2, float gndHeight, myPoint2D f){
        float KpGnd = 100000;
        float KdGnd = 6000;
	float x  = MonState[i1];
	float dx = MonState[i1+1];
	float y  = MonState[i1+2];
	float dy = MonState[i1+3];
	
	if (FootState[i2]==0.0) {		// first contact?
		int tempIndex = i2;
		//check and see if it is one of the foot points. If it is, the thing didn't fall over yet.
		if (tempIndex!=0 && tempIndex!=3 && tempIndex!=6 && tempIndex!=9)
			loseControl();

		FootState[i2] = 1.0f;		// mark as in contact 
		FootState[i2+1] = x;	        // save x coord
		FootState[i2+2] = gndHeight;	// save y coord 
	}
	f.x = (FootState[i2+1]-x)*KpGnd - dx*KdGnd;
	f.y = (FootState[i2+2]-y)*KpGnd - dy*KdGnd;
}
    
    
    public void clearJointTorques(){
        for (int i=0;i<7;i++)
            t[i] = 0;
    }
    
    public void clearExternalForces(){
        for (int i=0;i<50;i++)
            Fext[i] = 0;
    }
    
    /** This method sets the state of the biped */
    public void setState(float[] newState){
        for (int i=0;i<nrStates;i++){
            State[i] = newState[i];
            CopyState[i] = newState[i];
        }
    }
 
    /** This method draws a black rectangle at the origin - plays the role of a joint.*/
    public void drawJoint(Graphics g, Matrix3x3 transform){
        float jointSize = 0.01f;
        Polygon p = new Polygon();
        myPoint2D pp = new myPoint2D();

        // draw R knee joint
        pp = transform.multiplyBy(new myPoint2D(-jointSize, -jointSize));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(-jointSize, jointSize));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(jointSize, jointSize));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(jointSize, -jointSize));  p.addPoint((int)pp.x, -(int)pp.y);
        g.drawPolygon(p);
        g.fillPolygon(p);
    }
    
    float signum(float x){
        if (x<0)
            return -1;
        if (x>0)
            return 1;
        return 0;
    
    }
    
    
    /** this method is used to draw the biped */
    public void drawBiped(Graphics g, Matrix3x3 transform){
        Polygon p = new Polygon();
        myPoint2D pp = new myPoint2D();
        Matrix3x3 tArrow = transform.multiplyBy(Matrix3x3.getTranslationMatrix(MonState[40],MonState[42]));
        
        
        transform = transform.multiplyBy(Matrix3x3.getTranslationMatrix(State[0],State[2]));
        transform = transform.multiplyBy(Matrix3x3.getRotationMatrix(State[4]));

        Matrix3x3 mCopy = transform;
        transform = transform.multiplyBy(Matrix3x3.getRotationMatrix(State[10]));

       // draw the left upper leg
        pp = transform.multiplyBy(new myPoint2D(-0.04, 0.02));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(0.04, 0.02));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(0.04, -0.45));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(-0.04, -0.45));  p.addPoint((int)pp.x, -(int)pp.y);
        g.setColor(new Color(100,100,100));
        g.fillPolygon(p);
        g.setColor(new Color(0,0,0));
        g.drawPolygon(p);        
        p.reset();
        
	// draw hip joint
        drawJoint(g, transform);
        
        transform = transform.multiplyBy(Matrix3x3.getTranslationMatrix(0,-0.45f));

        // draw the left lower leg
        transform = transform.multiplyBy(Matrix3x3.getRotationMatrix(State[12]));

        pp = transform.multiplyBy(new myPoint2D(-0.025, 0.02));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(0.025, 0.02));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(0.025, -0.45));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(-0.025, -0.45));  p.addPoint((int)pp.x, -(int)pp.y);
        g.setColor(new Color(100,100,100));
        g.fillPolygon(p);
        g.setColor(new Color(0,0,0));
        g.drawPolygon(p);        
        p.reset();
        
        //draw the L knee Joint
        drawJoint(g, transform);

       // draw L foot
        transform = transform.multiplyBy(Matrix3x3.getTranslationMatrix(0,-0.45f));
        transform = transform.multiplyBy(Matrix3x3.getRotationMatrix(State[16]));
        pp = transform.multiplyBy(new myPoint2D(-0.04, 0.0));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(-0.04, -0.04));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(0.12, -0.04));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(0.12, 0.0));  p.addPoint((int)pp.x, -(int)pp.y);
        g.setColor(new Color(100,100,100));
        g.fillPolygon(p);
        g.setColor(new Color(0,0,0));
        g.drawPolygon(p);        
        p.reset();

        
        // draw R ankle joint
        drawJoint(g, transform);

        transform = mCopy;

        //draw the link here - eliminate some visual artifacts...
        pp = transform.multiplyBy(new myPoint2D(-0.05, 0.0));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(0.05, 0.0));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(0.05, 0.48));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(-0.05, 0.48));  p.addPoint((int)pp.x, -(int)pp.y);
        g.setColor(new Color(200,200,200));
        g.fillPolygon(p);
        g.setColor(new Color(0,0,0));
        g.drawPolygon(p);        
        p.reset();
        
        
        transform = transform.multiplyBy(Matrix3x3.getRotationMatrix(State[6]));

       // draw link 2 (right upper leg)
        pp = transform.multiplyBy(new myPoint2D(-0.04, 0.02));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(0.04, 0.02));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(0.04, -0.45));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(-0.04, -0.45));  p.addPoint((int)pp.x, -(int)pp.y);
        g.setColor(new Color(255,255,255));
        g.fillPolygon(p);
        g.setColor(new Color(0,0,0));
        g.drawPolygon(p);        
        p.reset();
        
	// draw hip joint
        drawJoint(g, transform);
        
        transform = transform.multiplyBy(Matrix3x3.getTranslationMatrix(0,-0.45f));

        // draw link 3 (right lower leg)
        transform = transform.multiplyBy(Matrix3x3.getRotationMatrix(State[8]));

        pp = transform.multiplyBy(new myPoint2D(-0.025, 0.02));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(0.025, 0.02));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(0.025, -0.45));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(-0.025, -0.45));  p.addPoint((int)pp.x, -(int)pp.y);
        g.setColor(new Color(255,255,255));
        g.fillPolygon(p);
        g.setColor(new Color(0,0,0));
        g.drawPolygon(p);        
        p.reset();
        
        //draw the R knee Joint
        drawJoint(g, transform);

       // draw R foot
        transform = transform.multiplyBy(Matrix3x3.getTranslationMatrix(0,-0.45f));
        transform = transform.multiplyBy(Matrix3x3.getRotationMatrix(State[14]));
        pp = transform.multiplyBy(new myPoint2D(-0.04, 0.0));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(-0.04, -0.04));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(0.12, -0.04));  p.addPoint((int)pp.x, -(int)pp.y);
        pp = transform.multiplyBy(new myPoint2D(0.12, 0.0));  p.addPoint((int)pp.x, -(int)pp.y);
        g.setColor(new Color(255,255,255));
        g.fillPolygon(p);
        g.setColor(new Color(0,0,0));
        g.drawPolygon(p);        
        p.reset();

        
        // draw R ankle joint
        drawJoint(g, transform);
        
        
        //now we'll draw an arrow for the push, if we have any...
        if (PushTime>0){
            g.setColor(new Color(0,0,0));
            Graphics2D g2 = (Graphics2D)g;
            BasicStroke bs = new BasicStroke(3);
            g2.setStroke(bs);

            monBip7(State,MonState);
            myPoint2D p1 = new myPoint2D();
            myPoint2D p2 = new myPoint2D();
            
            p1 = tArrow.multiplyBy(new myPoint2D(0, 0));
            p2 = tArrow.multiplyBy(new myPoint2D(-PushForce * 0.01, 0));
            g.drawLine((int)p1.x,-(int)p1.y,(int)p2.x,-(int)p2.y);
            
            p2 = tArrow.multiplyBy(new myPoint2D(-signum(PushForce) * 0.05, 0.05));
            g.drawLine((int)p1.x,-(int)p1.y,(int)p2.x,-(int)p2.y);
            
            p2 = tArrow.multiplyBy(new myPoint2D(-signum(PushForce) * 0.05, -0.05));
            g.drawLine((int)p1.x,-(int)p1.y,(int)p2.x,-(int)p2.y);
            

            bs = new BasicStroke(1);
            g2.setStroke(bs);
        }
        
        

    }

    
    
    
    
    
    
    
/**
 *   CAUTION: the code below was automatically generated and is not meant to be readable.
 *   Read at own risk!!! You might get a headache though - I did...
 *   --------------------------------------------------------------------------------------
 */    
    
    
private static float G  = (-9.800f);
private static float MSIZE= 9;
private static float MASS7= 1.000000f;
private static float INER7= 0.010000f;
private static float LENCM7= 0.044721f;
private static float A_CM7= -0.463648f;
private static float LENMON4= 0.126491f;
private static float A_M4= -0.321751f;
private static float LENMON3= 0.056569f;
private static float A_M3= -2.356194f;
private static float LENAT7= 0.450000f;
private static float A_AT7= -1.570796f;
private static float MASS6= 1.000000f;
private static float INER6= 0.010000f;
private static float LENCM6= 0.044721f;
private static float A_CM6= -0.463648f;
private static float LENMON2= 0.126491f;
private static float A_M2= -0.321751f;
private static float LENMON1= 0.056569f;
private static float A_M1= -2.356194f;
private static float LENAT6= 0.450000f;
private static float A_AT6= -1.570796f;
private static float MASS5= 4.000000f;
private static float INER5= 0.069600f;
private static float LENCM5= 0.225000f;
private static float A_CM5= -1.570796f;
private static float LENMON8= 0.040000f;
private static float A_M8= 0.000000f;
private static float LENMON7= 0.040000f;
private static float A_M7= 3.141593f;
private static float LENAT5= 0.450000f;
private static float A_AT5= -1.570796f;
private static float MASS4= 5.000000f;
private static float INER4= 0.088500f;
private static float LENCM4= 0.225000f;
private static float A_CM4= -1.570796f;
private static float LENAT4= 0.000000f;
private static float A_AT4= 0.000000f;
private static float MASS3= 4.000000f;
private static float INER3= 0.069600f;
private static float LENCM3= 0.225000f;
private static float A_CM3= -1.570796f;
private static float LENMON6= 0.040000f;
private static float A_M6= 0.000000f;
private static float LENMON5= 0.040000f;
private static float A_M5= 3.141593f;
private static float LENAT3= 0.450000f;
private static float A_AT3= -1.570796f;
private static float MASS2= 5.000000f;
private static float INER2= 0.088500f;
private static float LENCM2= 0.225000f;
private static float A_CM2= -1.570796f;
private static float LENAT2= 0.000000f;
private static float A_AT2= 0.000000f;
private static float MASS1= 70.000000f;
private static float INER1= 1.475000f;
private static float LENCM1= 0.240000f;
private static float A_CM1= 1.570796f;
private static float LENMON12= 0.482597f;
private static float A_M12= 1.467004f;
private static float LENMON11= 0.482597f;
private static float A_M11= 1.674589f;
private static float LENMON10= 0.050000f;
private static float A_M10= 0.000000f;
private static float LENMON9= 0.050000f;
private static float A_M9= 3.141593f;
private static float LENAT1= 0.000000f;
private static float A_AT1= 0.000000f;

/**
 *  This method computes and stores the accelerations for the biped in the variable ac. Inputs to this method are the current state,
 *  the torques at the joints and the external forces applied at the monitor points.
 */
public void dynBip7Eqns(float st[]){
        int MAX_EQNS = 20;
        float b[] = new float[MAX_EQNS];
        float A[][] = new float[MAX_EQNS][MAX_EQNS];
        float x[] = new float[MAX_EQNS];
	float th1;
	float dth1;
	float cm_angle1;
	float sin_cm1;
	float cos_cm1;
	float m_angle12;
	float sin_m12;
	float cos_m12;
	float m_angle11;
	float sin_m11;
	float cos_m11;
	float m_angle10;
	float sin_m10;
	float cos_m10;
	float m_angle9;
	float sin_m9;
	float cos_m9;
	float th2;
	float dth2;
	float cm_angle2;
	float sin_cm2;
	float cos_cm2;
	float at_angle2;
	float sin_at2;
	float cos_at2;
	float th3;
	float dth3;
	float cm_angle3;
	float sin_cm3;
	float cos_cm3;
	float at_angle3;
	float sin_at3;
	float cos_at3;
	float m_angle6;
	float sin_m6;
	float cos_m6;
	float m_angle5;
	float sin_m5;
	float cos_m5;
	float th6;
	float dth6;
	float cm_angle6;
	float sin_cm6;
	float cos_cm6;
	float at_angle6;
	float sin_at6;
	float cos_at6;
	float m_angle2;
	float sin_m2;
	float cos_m2;
	float m_angle1;
	float sin_m1;
	float cos_m1;
	float th4;
	float dth4;
	float cm_angle4;
	float sin_cm4;
	float cos_cm4;
	float at_angle4;
	float sin_at4;
	float cos_at4;
	float th5;
	float dth5;
	float cm_angle5;
	float sin_cm5;
	float cos_cm5;
	float at_angle5;
	float sin_at5;
	float cos_at5;
	float m_angle8;
	float sin_m8;
	float cos_m8;
	float m_angle7;
	float sin_m7;
	float cos_m7;
	float th7;
	float dth7;
	float cm_angle7;
	float sin_cm7;
	float cos_cm7;
	float at_angle7;
	float sin_at7;
	float cos_at7;
	float m_angle4;
	float sin_m4;
	float cos_m4;
	float m_angle3;
	float sin_m3;
	float cos_m3;
	float k[] = new float[245];

	t[0] = 0.0f;
	th1 = 		 st[4];
	dth1 = 		 st[5];
	cm_angle1 = 		 A_CM1
		+th1;
	sin_cm1 = 		 (float)Math.sin  (cm_angle1);
	cos_cm1 = 		 (float)Math.cos (cm_angle1);
	m_angle12 = 		 A_M12
		+th1;
	sin_m12 = 		 (float)Math.sin  (m_angle12);
	cos_m12 = 		 (float)Math.cos (m_angle12);
	m_angle11 = 		 A_M11
		+th1;
	sin_m11 = 		 (float)Math.sin  (m_angle11);
	cos_m11 = 		 (float)Math.cos (m_angle11);
	m_angle10 = 		 A_M10
		+th1;
	sin_m10 = 		 (float)Math.sin  (m_angle10);
	cos_m10 = 		 (float)Math.cos (m_angle10);
	m_angle9 = 		 A_M9
		+th1;
	sin_m9 = 		 (float)Math.sin  (m_angle9);
	cos_m9 = 		 (float)Math.cos (m_angle9);
	th2 = 		 th1
		+st[6];
	dth2 = 		 dth1
		+st[7];
	cm_angle2 = 		 A_CM2
		+th2;
	sin_cm2 = 		 (float)Math.sin  (cm_angle2);
	cos_cm2 = 		 (float)Math.cos (cm_angle2);
	at_angle2 = 		 A_AT2
		+th1;
	sin_at2 = 		 (float)Math.sin  (at_angle2);
	cos_at2 = 		 (float)Math.cos (at_angle2);
	th3 = 		 th2
		+st[8];
	dth3 = 		 dth2
		+st[9];
	cm_angle3 = 		 A_CM3
		+th3;
	sin_cm3 = 		 (float)Math.sin  (cm_angle3);
	cos_cm3 = 		 (float)Math.cos (cm_angle3);
	at_angle3 = 		 A_AT3
		+th2;
	sin_at3 = 		 (float)Math.sin  (at_angle3);
	cos_at3 = 		 (float)Math.cos (at_angle3);
	m_angle6 = 		 A_M6
		+th3;
	sin_m6 = 		 (float)Math.sin  (m_angle6);
	cos_m6 = 		 (float)Math.cos (m_angle6);
	m_angle5 = 		 A_M5
		+th3;
	sin_m5 = 		 (float)Math.sin  (m_angle5);
	cos_m5 = 		 (float)Math.cos (m_angle5);
	th6 = 		 th3
		+st[14];
	dth6 = 		 dth3
		+st[15];
	cm_angle6 = 		 A_CM6
		+th6;
	sin_cm6 = 		 (float)Math.sin  (cm_angle6);
	cos_cm6 = 		 (float)Math.cos (cm_angle6);
	at_angle6 = 		 A_AT6
		+th3;
	sin_at6 = 		 (float)Math.sin  (at_angle6);
	cos_at6 = 		 (float)Math.cos (at_angle6);
	m_angle2 = 		 A_M2
		+th6;
	sin_m2 = 		 (float)Math.sin  (m_angle2);
	cos_m2 = 		 (float)Math.cos (m_angle2);
	m_angle1 = 		 A_M1
		+th6;
	sin_m1 = 		 (float)Math.sin  (m_angle1);
	cos_m1 = 		 (float)Math.cos (m_angle1);
	th4 = 		 th1
		+st[10];
	dth4 = 		 dth1
		+st[11];
	cm_angle4 = 		 A_CM4
		+th4;
	sin_cm4 = 		 (float)Math.sin  (cm_angle4);
	cos_cm4 = 		 (float)Math.cos (cm_angle4);
	at_angle4 = 		 A_AT4
		+th1;
	sin_at4 = 		 (float)Math.sin  (at_angle4);
	cos_at4 = 		 (float)Math.cos (at_angle4);
	th5 = 		 th4
		+st[12];
	dth5 = 		 dth4
		+st[13];
	cm_angle5 = 		 A_CM5
		+th5;
	sin_cm5 = 		 (float)Math.sin  (cm_angle5);
	cos_cm5 = 		 (float)Math.cos (cm_angle5);
	at_angle5 = 		 A_AT5
		+th4;
	sin_at5 = 		 (float)Math.sin  (at_angle5);
	cos_at5 = 		 (float)Math.cos (at_angle5);
	m_angle8 = 		 A_M8
		+th5;
	sin_m8 = 		 (float)Math.sin  (m_angle8);
	cos_m8 = 		 (float)Math.cos (m_angle8);
	m_angle7 = 		 A_M7
		+th5;
	sin_m7 = 		 (float)Math.sin  (m_angle7);
	cos_m7 = 		 (float)Math.cos (m_angle7);
	th7 = 		 th5
		+st[16];
	dth7 = 		 dth5
		+st[17];
	cm_angle7 = 		 A_CM7
		+th7;
	sin_cm7 = 		 (float)Math.sin  (cm_angle7);
	cos_cm7 = 		 (float)Math.cos (cm_angle7);
	at_angle7 = 		 A_AT7
		+th5;
	sin_at7 = 		 (float)Math.sin  (at_angle7);
	cos_at7 = 		 (float)Math.cos (at_angle7);
	m_angle4 = 		 A_M4
		+th7;
	sin_m4 = 		 (float)Math.sin  (m_angle4);
	cos_m4 = 		 (float)Math.cos (m_angle4);
	m_angle3 = 		 A_M3
		+th7;
	sin_m3 = 		 (float)Math.sin  (m_angle3);
	cos_m3 = 		 (float)Math.cos (m_angle3);
	k[0] = 		-dth1*dth1*LENCM1*cos_cm1;
	k[1] = 		-sin_cm1*LENCM1;
	k[2] = 		-dth1*dth1*LENCM1*sin_cm1;
	k[3] = 		 cos_cm1*LENCM1;
	k[4] = 		-dth1*dth1*LENAT2*cos_at2;
	k[5] = 		-sin_at2*LENAT2;
	k[6] = 		-dth1*dth1*LENAT2*sin_at2;
	k[7] = 		 cos_at2*LENAT2;
	k[8] = 		-dth2*dth2*LENCM2*cos_cm2;
	k[9] = 		-sin_cm2*LENCM2;
	k[10] = 		-sin_cm2*LENCM2;
	k[11] = 		-dth2*dth2*LENCM2*sin_cm2;
	k[12] = 		 cos_cm2*LENCM2;
	k[13] = 		 cos_cm2*LENCM2;
	k[14] = 		-dth2*dth2*LENAT3*cos_at3;
	k[15] = 		-sin_at3*LENAT3;
	k[16] = 		-sin_at3*LENAT3;
	k[17] = 		-dth2*dth2*LENAT3*sin_at3;
	k[18] = 		 cos_at3*LENAT3;
	k[19] = 		 cos_at3*LENAT3;
	k[20] = 		-dth3*dth3*LENCM3*cos_cm3;
	k[21] = 		-sin_cm3*LENCM3;
	k[22] = 		-sin_cm3*LENCM3;
	k[23] = 		-sin_cm3*LENCM3;
	k[24] = 		-dth3*dth3*LENCM3*sin_cm3;
	k[25] = 		 cos_cm3*LENCM3;
	k[26] = 		 cos_cm3*LENCM3;
	k[27] = 		 cos_cm3*LENCM3;
	k[28] = 		-dth3*dth3*LENAT6*cos_at6;
	k[29] = 		-sin_at6*LENAT6;
	k[30] = 		-sin_at6*LENAT6;
	k[31] = 		-sin_at6*LENAT6;
	k[32] = 		-dth3*dth3*LENAT6*sin_at6;
	k[33] = 		 cos_at6*LENAT6;
	k[34] = 		 cos_at6*LENAT6;
	k[35] = 		 cos_at6*LENAT6;
	k[36] = 		-dth6*dth6*LENCM6*cos_cm6;
	k[37] = 		-sin_cm6*LENCM6;
	k[38] = 		-sin_cm6*LENCM6;
	k[39] = 		-sin_cm6*LENCM6;
	k[40] = 		-sin_cm6*LENCM6;
	k[41] = 		-dth6*dth6*LENCM6*sin_cm6;
	k[42] = 		 cos_cm6*LENCM6;
	k[43] = 		 cos_cm6*LENCM6;
	k[44] = 		 cos_cm6*LENCM6;
	k[45] = 		 cos_cm6*LENCM6;
	k[46] = 		-dth1*dth1*LENAT4*cos_at4;
	k[47] = 		-sin_at4*LENAT4;
	k[48] = 		-dth1*dth1*LENAT4*sin_at4;
	k[49] = 		 cos_at4*LENAT4;
	k[50] = 		-dth4*dth4*LENCM4*cos_cm4;
	k[51] = 		-sin_cm4*LENCM4;
	k[52] = 		-sin_cm4*LENCM4;
	k[53] = 		-dth4*dth4*LENCM4*sin_cm4;
	k[54] = 		 cos_cm4*LENCM4;
	k[55] = 		 cos_cm4*LENCM4;
	k[56] = 		-dth4*dth4*LENAT5*cos_at5;
	k[57] = 		-sin_at5*LENAT5;
	k[58] = 		-sin_at5*LENAT5;
	k[59] = 		-dth4*dth4*LENAT5*sin_at5;
	k[60] = 		 cos_at5*LENAT5;
	k[61] = 		 cos_at5*LENAT5;
	k[62] = 		-dth5*dth5*LENCM5*cos_cm5;
	k[63] = 		-sin_cm5*LENCM5;
	k[64] = 		-sin_cm5*LENCM5;
	k[65] = 		-sin_cm5*LENCM5;
	k[66] = 		-dth5*dth5*LENCM5*sin_cm5;
	k[67] = 		 cos_cm5*LENCM5;
	k[68] = 		 cos_cm5*LENCM5;
	k[69] = 		 cos_cm5*LENCM5;
	k[70] = 		-dth5*dth5*LENAT7*cos_at7;
	k[71] = 		-sin_at7*LENAT7;
	k[72] = 		-sin_at7*LENAT7;
	k[73] = 		-sin_at7*LENAT7;
	k[74] = 		-dth5*dth5*LENAT7*sin_at7;
	k[75] = 		 cos_at7*LENAT7;
	k[76] = 		 cos_at7*LENAT7;
	k[77] = 		 cos_at7*LENAT7;
	k[78] = 		-dth7*dth7*LENCM7*cos_cm7;
	k[79] = 		-sin_cm7*LENCM7;
	k[80] = 		-sin_cm7*LENCM7;
	k[81] = 		-sin_cm7*LENCM7;
	k[82] = 		-sin_cm7*LENCM7;
	k[83] = 		-dth7*dth7*LENCM7*sin_cm7;
	k[84] = 		 cos_cm7*LENCM7;
	k[85] = 		 cos_cm7*LENCM7;
	k[86] = 		 cos_cm7*LENCM7;
	k[87] = 		 cos_cm7*LENCM7;
	k[88] = 		-Fext[2]
		-Fext[0];
	k[89] = 		-Fext[3]
		-Fext[1]
		-MASS6*G;
	k[90] = 		-Fext[10]
		-Fext[8];
	k[91] = 		-Fext[11]
		-Fext[9]
		-MASS3*G;
	k[92] = 		-MASS2*G;
	k[93] = 		-Fext[6]
		-Fext[4];
	k[94] = 		-Fext[7]
		-Fext[5]
		-MASS7*G;
	k[95] = 		-Fext[14]
		-Fext[12];
	k[96] = 		-Fext[15]
		-Fext[13]
		-MASS5*G;
	k[97] = 		-MASS4*G;
	k[98] = 		-Fext[22]
		-Fext[20]
		-Fext[18]
		-Fext[16];
	k[99] = 		-Fext[23]
		-Fext[21]
		-Fext[19]
		-Fext[17]
		-MASS1*G;
	k[100] = 		 MASS7*G*LENCM7*cos_cm7
		+t[6]
		-Fext[6]*LENMON4*sin_m4
		+Fext[7]*LENMON4*cos_m4
		-Fext[4]*LENMON3*sin_m3
		+Fext[5]*LENMON3*cos_m3;
	k[101] = 		-MASS7*LENCM7*cos_cm7;
	k[102] = 		 MASS7*LENCM7*sin_cm7;
	k[103] = 		 MASS6*G*LENCM6*cos_cm6
		+t[5]
		-Fext[2]*LENMON2*sin_m2
		+Fext[3]*LENMON2*cos_m2
		-Fext[0]*LENMON1*sin_m1
		+Fext[1]*LENMON1*cos_m1;
	k[104] = 		-MASS6*LENCM6*cos_cm6;
	k[105] = 		 MASS6*LENCM6*sin_cm6;
	k[106] = 		 MASS5*G*LENCM5*cos_cm5
		+t[4]
		-t[6]
		-Fext[14]*LENMON8*sin_m8
		+Fext[15]*LENMON8*cos_m8
		-Fext[12]*LENMON7*sin_m7
		+Fext[13]*LENMON7*cos_m7;
	k[107] = 		-LENAT7*cos_at7;
	k[108] = 		 LENAT7*sin_at7;
	k[109] = 		-MASS5*LENCM5*cos_cm5;
	k[110] = 		 MASS5*LENCM5*sin_cm5;
	k[111] = 		 MASS4*G*LENCM4*cos_cm4
		+t[3]
		-t[4];
	k[112] = 		-LENAT5*cos_at5;
	k[113] = 		 LENAT5*sin_at5;
	k[114] = 		-MASS4*LENCM4*cos_cm4;
	k[115] = 		 MASS4*LENCM4*sin_cm4;
	k[116] = 		 MASS3*G*LENCM3*cos_cm3
		+t[2]
		-t[5]
		-Fext[10]*LENMON6*sin_m6
		+Fext[11]*LENMON6*cos_m6
		-Fext[8]*LENMON5*sin_m5
		+Fext[9]*LENMON5*cos_m5;
	k[117] = 		-LENAT6*cos_at6;
	k[118] = 		 LENAT6*sin_at6;
	k[119] = 		-MASS3*LENCM3*cos_cm3;
	k[120] = 		 MASS3*LENCM3*sin_cm3;
	k[121] = 		 MASS2*G*LENCM2*cos_cm2
		+t[1]
		-t[2];
	k[122] = 		-LENAT3*cos_at3;
	k[123] = 		 LENAT3*sin_at3;
	k[124] = 		-MASS2*LENCM2*cos_cm2;
	k[125] = 		 MASS2*LENCM2*sin_cm2;
	k[126] = 		 MASS1*G*LENCM1*cos_cm1
		+t[0]
		-t[1]
		-t[3]
		-Fext[22]*LENMON12*sin_m12
		+Fext[23]*LENMON12*cos_m12
		-Fext[20]*LENMON11*sin_m11
		+Fext[21]*LENMON11*cos_m11
		-Fext[18]*LENMON10*sin_m10
		+Fext[19]*LENMON10*cos_m10
		-Fext[16]*LENMON9*sin_m9
		+Fext[17]*LENMON9*cos_m9;
	k[127] = 		-LENAT4*cos_at4;
	k[128] = 		 LENAT4*sin_at4;
	k[129] = 		-LENAT2*cos_at2;
	k[130] = 		 LENAT2*sin_at2;
	k[131] = 		-MASS1*LENCM1*cos_cm1;
	k[132] = 		 MASS1*LENCM1*sin_cm1;
	k[133] = 		 k[47]
		+k[58];
	k[134] = 		 k[46]
		+k[56];
	k[135] = 		 k[57]
		+k[72];
	k[136] = 		 k[133]
		+k[73];
	k[137] = 		 k[134]
		+k[70];
	k[138] = 		 k[71]
		+k[80];
	k[139] = 		 k[135]
		+k[81];
	k[140] = 		 k[136]
		+k[82];
	k[141] = 		 k[137]
		+k[78];
	k[142] = 		 MASS7*k[79];
	k[143] = 		 MASS7*k[138];
	k[144] = 		 MASS7*k[139];
	k[145] = 		 MASS7*k[140];
	k[146] = 		 MASS7*k[141]
		+k[93];
	k[147] = 		 k[57]
		+k[64];
	k[148] = 		 k[133]
		+k[65];
	k[149] = 		 k[134]
		+k[62];
	k[150] = 		 MASS5
		+MASS7;
	k[151] = 		 MASS5*k[63]
		+k[143];
	k[152] = 		 MASS5*k[147]
		+k[144];
	k[153] = 		 MASS5*k[148]
		+k[145];
	k[154] = 		 MASS5*k[149]
		+k[95]
		+k[146];
	k[155] = 		 k[47]
		+k[52];
	k[156] = 		 k[46]
		+k[50];
	k[157] = 		 MASS4
		+k[150];
	k[158] = 		 MASS4*k[51]
		+k[152];
	k[159] = 		 MASS4*k[155]
		+k[153];
	k[160] = 		 MASS4*k[156]
		+k[154];
	k[161] = 		 k[5]
		+k[16];
	k[162] = 		 k[4]
		+k[14];
	k[163] = 		 k[15]
		+k[30];
	k[164] = 		 k[161]
		+k[31];
	k[165] = 		 k[162]
		+k[28];
	k[166] = 		 k[29]
		+k[38];
	k[167] = 		 k[163]
		+k[39];
	k[168] = 		 k[164]
		+k[40];
	k[169] = 		 k[165]
		+k[36];
	k[170] = 		 MASS6*k[37];
	k[171] = 		 MASS6*k[166];
	k[172] = 		 MASS6*k[167];
	k[173] = 		 MASS6*k[168];
	k[174] = 		 MASS6*k[169]
		+k[88];
	k[175] = 		 k[15]
		+k[22];
	k[176] = 		 k[161]
		+k[23];
	k[177] = 		 k[162]
		+k[20];
	k[178] = 		 MASS3
		+MASS6;
	k[179] = 		 MASS3*k[21]
		+k[171];
	k[180] = 		 MASS3*k[175]
		+k[172];
	k[181] = 		 MASS3*k[176]
		+k[173];
	k[182] = 		 MASS3*k[177]
		+k[90]
		+k[174];
	k[183] = 		 k[5]
		+k[10];
	k[184] = 		 k[4]
		+k[8];
	k[185] = 		 MASS2
		+k[178];
	k[186] = 		 MASS2*k[9]
		+k[180];
	k[187] = 		 MASS2*k[183]
		+k[181];
	k[188] = 		 MASS2*k[184]
		+k[182];
	k[189] = 		 k[49]
		+k[61];
	k[190] = 		 k[48]
		+k[59];
	k[191] = 		 k[60]
		+k[76];
	k[192] = 		 k[189]
		+k[77];
	k[193] = 		 k[190]
		+k[74];
	k[194] = 		 k[75]
		+k[85];
	k[195] = 		 k[191]
		+k[86];
	k[196] = 		 k[192]
		+k[87];
	k[197] = 		 k[193]
		+k[83];
	k[198] = 		 MASS7*k[84];
	k[199] = 		 MASS7*k[194];
	k[200] = 		 MASS7*k[195];
	k[201] = 		 MASS7*k[196];
	k[202] = 		 MASS7*k[197]
		+k[94];
	k[203] = 		 k[60]
		+k[68];
	k[204] = 		 k[189]
		+k[69];
	k[205] = 		 k[190]
		+k[66];
	k[206] = 		 MASS5
		+MASS7;
	k[207] = 		 MASS5*k[67]
		+k[199];
	k[208] = 		 MASS5*k[203]
		+k[200];
	k[209] = 		 MASS5*k[204]
		+k[201];
	k[210] = 		 MASS5*k[205]
		+k[96]
		+k[202];
	k[211] = 		 k[49]
		+k[55];
	k[212] = 		 k[48]
		+k[53];
	k[213] = 		 MASS4
		+k[206];
	k[214] = 		 MASS4*k[54]
		+k[208];
	k[215] = 		 MASS4*k[211]
		+k[209];
	k[216] = 		 MASS4*k[212]
		+k[97]
		+k[210];
	k[217] = 		 k[7]
		+k[19];
	k[218] = 		 k[6]
		+k[17];
	k[219] = 		 k[18]
		+k[34];
	k[220] = 		 k[217]
		+k[35];
	k[221] = 		 k[218]
		+k[32];
	k[222] = 		 k[33]
		+k[43];
	k[223] = 		 k[219]
		+k[44];
	k[224] = 		 k[220]
		+k[45];
	k[225] = 		 k[221]
		+k[41];
	k[226] = 		 MASS6*k[42];
	k[227] = 		 MASS6*k[222];
	k[228] = 		 MASS6*k[223];
	k[229] = 		 MASS6*k[224];
	k[230] = 		 MASS6*k[225]
		+k[89];
	k[231] = 		 k[18]
		+k[26];
	k[232] = 		 k[217]
		+k[27];
	k[233] = 		 k[218]
		+k[24];
	k[234] = 		 MASS3
		+MASS6;
	k[235] = 		 MASS3*k[25]
		+k[227];
	k[236] = 		 MASS3*k[231]
		+k[228];
	k[237] = 		 MASS3*k[232]
		+k[229];
	k[238] = 		 MASS3*k[233]
		+k[91]
		+k[230];
	k[239] = 		 k[7]
		+k[13];
	k[240] = 		 k[6]
		+k[11];
	k[241] = 		 MASS2
		+k[234];
	k[242] = 		 MASS2*k[12]
		+k[236];
	k[243] = 		 MASS2*k[239]
		+k[237];
	k[244] = 		 MASS2*k[240]
		+k[92]
		+k[238];

	A[0][0] = 		 MASS1
		+k[185]
		+k[157];
	A[0][1] = 		0;
	A[0][2] = 		 k[142];
	A[0][3] = 		 k[170];
	A[0][4] = 		 k[151];
	A[0][5] = 		 k[158];
	A[0][6] = 		 k[179];
	A[0][7] = 		 k[186];
	A[0][8] = 		 MASS1*k[1]
		+k[187]
		+k[159];
	A[1][0] = 		0;
	A[1][1] = 		 MASS1
		+k[241]
		+k[213];
	A[1][2] = 		 k[198];
	A[1][3] = 		 k[226];
	A[1][4] = 		 k[207];
	A[1][5] = 		 k[214];
	A[1][6] = 		 k[235];
	A[1][7] = 		 k[242];
	A[1][8] = 		 MASS1*k[3]
		+k[243]
		+k[215];
	A[2][0] = 		 k[102];
	A[2][1] = 		 k[101];
	A[2][2] = 		 k[102]*k[79]
		+k[101]*k[84]
		-INER7;
	A[2][3] = 		0;
	A[2][4] = 		 k[102]*k[138]
		+k[101]*k[194]
		-INER7;
	A[2][5] = 		 k[102]*k[139]
		+k[101]*k[195]
		-INER7;
	A[2][6] = 		0;
	A[2][7] = 		0;
	A[2][8] = 		 k[102]*k[140]
		+k[101]*k[196]
		-INER7;
	A[3][0] = 		 k[105];
	A[3][1] = 		 k[104];
	A[3][2] = 		0;
	A[3][3] = 		 k[105]*k[37]
		+k[104]*k[42]
		-INER6;
	A[3][4] = 		0;
	A[3][5] = 		0;
	A[3][6] = 		 k[105]*k[166]
		+k[104]*k[222]
		-INER6;
	A[3][7] = 		 k[105]*k[167]
		+k[104]*k[223]
		-INER6;
	A[3][8] = 		 k[105]*k[168]
		+k[104]*k[224]
		-INER6;
	A[4][0] = 		 k[110]
		+k[108]*MASS7;
	A[4][1] = 		 k[109]
		+k[107]*MASS7;
	A[4][2] = 		 k[108]*k[142]
		+k[107]*k[198];
	A[4][3] = 		0;
	A[4][4] = 		 k[110]*k[63]
		+k[109]*k[67]
		+k[108]*k[143]
		+k[107]*k[199]
		-INER5;
	A[4][5] = 		 k[110]*k[147]
		+k[109]*k[203]
		+k[108]*k[144]
		+k[107]*k[200]
		-INER5;
	A[4][6] = 		0;
	A[4][7] = 		0;
	A[4][8] = 		 k[110]*k[148]
		+k[109]*k[204]
		+k[108]*k[145]
		+k[107]*k[201]
		-INER5;
	A[5][0] = 		 k[115]
		+k[113]*k[150];
	A[5][1] = 		 k[114]
		+k[112]*k[206];
	A[5][2] = 		 k[113]*k[142]
		+k[112]*k[198];
	A[5][3] = 		0;
	A[5][4] = 		 k[113]*k[151]
		+k[112]*k[207];
	A[5][5] = 		 k[115]*k[51]
		+k[114]*k[54]
		+k[113]*k[152]
		+k[112]*k[208]
		-INER4;
	A[5][6] = 		0;
	A[5][7] = 		0;
	A[5][8] = 		 k[115]*k[155]
		+k[114]*k[211]
		+k[113]*k[153]
		+k[112]*k[209]
		-INER4;
	A[6][0] = 		 k[120]
		+k[118]*MASS6;
	A[6][1] = 		 k[119]
		+k[117]*MASS6;
	A[6][2] = 		0;
	A[6][3] = 		 k[118]*k[170]
		+k[117]*k[226];
	A[6][4] = 		0;
	A[6][5] = 		0;
	A[6][6] = 		 k[120]*k[21]
		+k[119]*k[25]
		+k[118]*k[171]
		+k[117]*k[227]
		-INER3;
	A[6][7] = 		 k[120]*k[175]
		+k[119]*k[231]
		+k[118]*k[172]
		+k[117]*k[228]
		-INER3;
	A[6][8] = 		 k[120]*k[176]
		+k[119]*k[232]
		+k[118]*k[173]
		+k[117]*k[229]
		-INER3;
	A[7][0] = 		 k[125]
		+k[123]*k[178];
	A[7][1] = 		 k[124]
		+k[122]*k[234];
	A[7][2] = 		0;
	A[7][3] = 		 k[123]*k[170]
		+k[122]*k[226];
	A[7][4] = 		0;
	A[7][5] = 		0;
	A[7][6] = 		 k[123]*k[179]
		+k[122]*k[235];
	A[7][7] = 		 k[125]*k[9]
		+k[124]*k[12]
		+k[123]*k[180]
		+k[122]*k[236]
		-INER2;
	A[7][8] = 		 k[125]*k[183]
		+k[124]*k[239]
		+k[123]*k[181]
		+k[122]*k[237]
		-INER2;
	A[8][0] = 		 k[132]
		+k[130]*k[185]
		+k[128]*k[157];
	A[8][1] = 		 k[131]
		+k[129]*k[241]
		+k[127]*k[213];
	A[8][2] = 		 k[128]*k[142]
		+k[127]*k[198];
	A[8][3] = 		 k[130]*k[170]
		+k[129]*k[226];
	A[8][4] = 		 k[128]*k[151]
		+k[127]*k[207];
	A[8][5] = 		 k[128]*k[158]
		+k[127]*k[214];
	A[8][6] = 		 k[130]*k[179]
		+k[129]*k[235];
	A[8][7] = 		 k[130]*k[186]
		+k[129]*k[242];
	A[8][8] = 		 k[132]*k[1]
		+k[131]*k[3]
		+k[130]*k[187]
		+k[129]*k[243]
		+k[128]*k[159]
		+k[127]*k[215]
		-INER1;
	b[0] = 		-MASS1*k[0]
		-k[98]
		-k[188]
		-k[160];
	b[1] = 		-MASS1*k[2]
		-k[99]
		-k[244]
		-k[216];
	b[2] = 		-k[102]*k[141]
		-k[101]*k[197]
		-k[100];
	b[3] = 		-k[105]*k[169]
		-k[104]*k[225]
		-k[103];
	b[4] = 		-k[110]*k[149]
		-k[109]*k[205]
		-k[108]*k[146]
		-k[107]*k[202]
		-k[106];
	b[5] = 		-k[115]*k[156]
		-k[114]*k[212]
		-k[111]
		-k[113]*k[154]
		-k[112]*k[210];
	b[6] = 		-k[120]*k[177]
		-k[119]*k[233]
		-k[118]*k[174]
		-k[117]*k[230]
		-k[116];
	b[7] = 		-k[125]*k[184]
		-k[124]*k[240]
		-k[121]
		-k[123]*k[182]
		-k[122]*k[238];
	b[8] = 		-k[132]*k[0]
		-k[131]*k[2]
		-k[130]*k[188]
		-k[129]*k[244]
		-k[128]*k[160]
		-k[127]*k[216]
		-k[126];
	MyLinAlg.lin_eq_solv(A,b,x,9);
	ac[0] = x[0];
	ac[1] = x[1];
	ac[8] = x[2];
	ac[7] = x[3];
	ac[6] = x[4];
	ac[5] = x[5];
	ac[4] = x[6];
	ac[3] = x[7];
	ac[2] = x[8];
    }

/** This method computes the position of the monitor points, in world coordinates given the current state.*/
public void monBip7(float state[], float mstate[]){
	float th1;
	float dth1;
	float xo1;
	float yo1;
	float vxo1;
	float vyo1;
	float m_angle12;
	float sin_m12;
	float cos_m12;
	float m_angle11;
	float sin_m11;
	float cos_m11;
	float m_angle10;
	float sin_m10;
	float cos_m10;
	float m_angle9;
	float sin_m9;
	float cos_m9;
	float th2;
	float dth2;
	float at_angle2;
	float sin_at2;
	float cos_at2;
	float xo2;
	float yo2;
	float vxo2;
	float vyo2;
	float th3;
	float dth3;
	float at_angle3;
	float sin_at3;
	float cos_at3;
	float xo3;
	float yo3;
	float vxo3;
	float vyo3;
	float m_angle6;
	float sin_m6;
	float cos_m6;
	float m_angle5;
	float sin_m5;
	float cos_m5;
	float th6;
	float dth6;
	float at_angle6;
	float sin_at6;
	float cos_at6;
	float xo6;
	float yo6;
	float vxo6;
	float vyo6;
	float m_angle2;
	float sin_m2;
	float cos_m2;
	float m_angle1;
	float sin_m1;
	float cos_m1;
	float th4;
	float dth4;
	float at_angle4;
	float sin_at4;
	float cos_at4;
	float xo4;
	float yo4;
	float vxo4;
	float vyo4;
	float th5;
	float dth5;
	float at_angle5;
	float sin_at5;
	float cos_at5;
	float xo5;
	float yo5;
	float vxo5;
	float vyo5;
	float m_angle8;
	float sin_m8;
	float cos_m8;
	float m_angle7;
	float sin_m7;
	float cos_m7;
	float th7;
	float dth7;
	float at_angle7;
	float sin_at7;
	float cos_at7;
	float xo7;
	float yo7;
	float vxo7;
	float vyo7;
	float m_angle4;
	float sin_m4;
	float cos_m4;
	float m_angle3;
	float sin_m3;
	float cos_m3;

	th1 = 		 state[4];
	dth1 = 		 state[5];
	xo1 = 		 state[0];
	yo1 = 		 state[2];
	vxo1 = 		 state[1];
	vyo1 = 		 state[3];
	m_angle12 = 		 A_M12
		+th1;
	sin_m12 = 		 (float)Math.sin  (m_angle12);
	cos_m12 = 		 (float)Math.cos (m_angle12);
	mstate[44] = 		 LENMON12*cos_m12
		+xo1;
	mstate[46] = 		 LENMON12*sin_m12
		+yo1;
	mstate[45] = 		-LENMON12*sin_m12*dth1
		+vxo1;
	mstate[47] = 		 LENMON12*cos_m12*dth1
		+vyo1;
	m_angle11 = 		 A_M11
		+th1;
	sin_m11 = 		 (float)Math.sin  (m_angle11);
	cos_m11 = 		 (float)Math.cos (m_angle11);
	mstate[40] = 		 LENMON11*cos_m11
		+xo1;
	mstate[42] = 		 LENMON11*sin_m11
		+yo1;
	mstate[41] = 		-LENMON11*sin_m11*dth1
		+vxo1;
	mstate[43] = 		 LENMON11*cos_m11*dth1
		+vyo1;
	m_angle10 = 		 A_M10
		+th1;
	sin_m10 = 		 (float)Math.sin  (m_angle10);
	cos_m10 = 		 (float)Math.cos (m_angle10);
	mstate[36] = 		 LENMON10*cos_m10
		+xo1;
	mstate[38] = 		 LENMON10*sin_m10
		+yo1;
	mstate[37] = 		-LENMON10*sin_m10*dth1
		+vxo1;
	mstate[39] = 		 LENMON10*cos_m10*dth1
		+vyo1;
	m_angle9 = 		 A_M9
		+th1;
	sin_m9 = 		 (float)Math.sin  (m_angle9);
	cos_m9 = 		 (float)Math.cos (m_angle9);
	mstate[32] = 		 LENMON9*cos_m9
		+xo1;
	mstate[34] = 		 LENMON9*sin_m9
		+yo1;
	mstate[33] = 		-LENMON9*sin_m9*dth1
		+vxo1;
	mstate[35] = 		 LENMON9*cos_m9*dth1
		+vyo1;
	th2 = 		 th1
		+state[6];
	dth2 = 		 dth1
		+state[7];
	at_angle2 = 		 A_AT2
		+th1;
	sin_at2 = 		 (float)Math.sin  (at_angle2);
	cos_at2 = 		 (float)Math.cos (at_angle2);
	xo2 = 		 LENAT2*cos_at2
		+xo1;
	yo2 = 		 LENAT2*sin_at2
		+yo1;
	vxo2 = 		-LENAT2*sin_at2*dth1
		+vxo1;
	vyo2 = 		 LENAT2*cos_at2*dth1
		+vyo1;
	th3 = 		 th2
		+state[8];
	dth3 = 		 dth2
		+state[9];
	at_angle3 = 		 A_AT3
		+th2;
	sin_at3 = 		 (float)Math.sin  (at_angle3);
	cos_at3 = 		 (float)Math.cos (at_angle3);
	xo3 = 		 LENAT3*cos_at3
		+xo2;
	yo3 = 		 LENAT3*sin_at3
		+yo2;
	vxo3 = 		-LENAT3*sin_at3*dth2
		+vxo2;
	vyo3 = 		 LENAT3*cos_at3*dth2
		+vyo2;
	m_angle6 = 		 A_M6
		+th3;
	sin_m6 = 		 (float)Math.sin  (m_angle6);
	cos_m6 = 		 (float)Math.cos (m_angle6);
	mstate[20] = 		 LENMON6*cos_m6
		+xo3;
	mstate[22] = 		 LENMON6*sin_m6
		+yo3;
	mstate[21] = 		-LENMON6*sin_m6*dth3
		+vxo3;
	mstate[23] = 		 LENMON6*cos_m6*dth3
		+vyo3;
	m_angle5 = 		 A_M5
		+th3;
	sin_m5 = 		 (float)Math.sin  (m_angle5);
	cos_m5 = 		 (float)Math.cos (m_angle5);
	mstate[16] = 		 LENMON5*cos_m5
		+xo3;
	mstate[18] = 		 LENMON5*sin_m5
		+yo3;
	mstate[17] = 		-LENMON5*sin_m5*dth3
		+vxo3;
	mstate[19] = 		 LENMON5*cos_m5*dth3
		+vyo3;
	th6 = 		 th3
		+state[14];
	dth6 = 		 dth3
		+state[15];
	at_angle6 = 		 A_AT6
		+th3;
	sin_at6 = 		 (float)Math.sin  (at_angle6);
	cos_at6 = 		 (float)Math.cos (at_angle6);
	xo6 = 		 LENAT6*cos_at6
		+xo3;
	yo6 = 		 LENAT6*sin_at6
		+yo3;
	vxo6 = 		-LENAT6*sin_at6*dth3
		+vxo3;
	vyo6 = 		 LENAT6*cos_at6*dth3
		+vyo3;
	m_angle2 = 		 A_M2
		+th6;
	sin_m2 = 		 (float)Math.sin  (m_angle2);
	cos_m2 = 		 (float)Math.cos (m_angle2);
	mstate[4] = 		 LENMON2*cos_m2
		+xo6;
	mstate[6] = 		 LENMON2*sin_m2
		+yo6;
	mstate[5] = 		-LENMON2*sin_m2*dth6
		+vxo6;
	mstate[7] = 		 LENMON2*cos_m2*dth6
		+vyo6;
	m_angle1 = 		 A_M1
		+th6;
	sin_m1 = 		 (float)Math.sin  (m_angle1);
	cos_m1 = 		 (float)Math.cos (m_angle1);
	mstate[0] = 		 LENMON1*cos_m1
		+xo6;
	mstate[2] = 		 LENMON1*sin_m1
		+yo6;
	mstate[1] = 		-LENMON1*sin_m1*dth6
		+vxo6;
	mstate[3] = 		 LENMON1*cos_m1*dth6
		+vyo6;
	th4 = 		 th1
		+state[10];
	dth4 = 		 dth1
		+state[11];
	at_angle4 = 		 A_AT4
		+th1;
	sin_at4 = 		 (float)Math.sin  (at_angle4);
	cos_at4 = 		 (float)Math.cos (at_angle4);
	xo4 = 		 LENAT4*cos_at4
		+xo1;
	yo4 = 		 LENAT4*sin_at4
		+yo1;
	vxo4 = 		-LENAT4*sin_at4*dth1
		+vxo1;
	vyo4 = 		 LENAT4*cos_at4*dth1
		+vyo1;
	th5 = 		 th4
		+state[12];
	dth5 = 		 dth4
		+state[13];
	at_angle5 = 		 A_AT5
		+th4;
	sin_at5 = 		 (float)Math.sin  (at_angle5);
	cos_at5 = 		 (float)Math.cos (at_angle5);
	xo5 = 		 LENAT5*cos_at5
		+xo4;
	yo5 = 		 LENAT5*sin_at5
		+yo4;
	vxo5 = 		-LENAT5*sin_at5*dth4
		+vxo4;
	vyo5 = 		 LENAT5*cos_at5*dth4
		+vyo4;
	m_angle8 = 		 A_M8
		+th5;
	sin_m8 = 		 (float)Math.sin  (m_angle8);
	cos_m8 = 		 (float)Math.cos (m_angle8);
	mstate[28] = 		 LENMON8*cos_m8
		+xo5;
	mstate[30] = 		 LENMON8*sin_m8
		+yo5;
	mstate[29] = 		-LENMON8*sin_m8*dth5
		+vxo5;
	mstate[31] = 		 LENMON8*cos_m8*dth5
		+vyo5;
	m_angle7 = 		 A_M7
		+th5;
	sin_m7 = 		 (float)Math.sin  (m_angle7);
	cos_m7 = 		 (float)Math.cos (m_angle7);
	mstate[24] = 		 LENMON7*cos_m7
		+xo5;
	mstate[26] = 		 LENMON7*sin_m7
		+yo5;
	mstate[25] = 		-LENMON7*sin_m7*dth5
		+vxo5;
	mstate[27] = 		 LENMON7*cos_m7*dth5
		+vyo5;
	th7 = 		 th5
		+state[16];
	dth7 = 		 dth5
		+state[17];
	at_angle7 = 		 A_AT7
		+th5;
	sin_at7 = 		 (float)Math.sin  (at_angle7);
	cos_at7 = 		 (float)Math.cos (at_angle7);
	xo7 = 		 LENAT7*cos_at7
		+xo5;
	yo7 = 		 LENAT7*sin_at7
		+yo5;
	vxo7 = 		-LENAT7*sin_at7*dth5
		+vxo5;
	vyo7 = 		 LENAT7*cos_at7*dth5
		+vyo5;
	m_angle4 = 		 A_M4
		+th7;
	sin_m4 = 		 (float)Math.sin  (m_angle4);
	cos_m4 = 		 (float)Math.cos (m_angle4);
	mstate[12] = 		 LENMON4*cos_m4
		+xo7;
	mstate[14] = 		 LENMON4*sin_m4
		+yo7;
	mstate[13] = 		-LENMON4*sin_m4*dth7
		+vxo7;
	mstate[15] = 		 LENMON4*cos_m4*dth7
		+vyo7;
	m_angle3 = 		 A_M3
		+th7;
	sin_m3 = 		 (float)Math.sin  (m_angle3);
	cos_m3 = 		 (float)Math.cos (m_angle3);
	mstate[8] = 		 LENMON3*cos_m3
		+xo7;
	mstate[10] = 		 LENMON3*sin_m3
		+yo7;
	mstate[9] = 		-LENMON3*sin_m3*dth7
		+vxo7;
	mstate[11] = 		 LENMON3*cos_m3*dth7
		+vyo7;
    }
   
    
}
