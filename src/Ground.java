/*
 * Ground.java
 *
 * Created on June 6, 2007, 10:11 PM
 *
 * To change this template, choose Tools | Template Manager
 * and open the template in the editor.
 */
import java.awt.BasicStroke;
import java.awt.Graphics;
import java.awt.BasicStroke;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Color;
import java.awt.Polygon;
import java.awt.geom.Point2D;

/**
 *
 * @author Stel-l
 */
public class Ground {
    public float Gx[] = new float[1000];
    public float Gy[] = new float[1000];
    public static boolean DrawTics = true;
    
    int npts;
    
    
    /** Creates a new instance of Ground */
    public Ground() {
        getFlatGround();
//        getComplexTerrain();
    }

    void getFlatGround(){
	npts=2;
	Gx[0] = -100;
	Gx[1] = 100;
	Gy[0] = 0;
        Gy[1] = 0;        
    }
    
    void getComplexTerrain(){
        npts=15;
        Gx[0] = -10;
        Gy[0] = 0;
        Gx[1] = 0;
        Gy[1] = 0;

        Gx[2] = 1.79f;
        Gy[2] = 0;
        Gx[3] = 1.8f;
        Gy[3] = -0.2f;
        Gx[4] = 3.0f;
        Gy[4] = -0.2f;
        Gx[5] = 3.01f;
        Gy[5] = -0.4f;
        Gx[6] = 5.2f;
        Gy[6] = -0.4f;
        Gx[7] = 5.21f;
        Gy[7] = -0.6f;
        Gx[8] = 7;
        Gy[8] = -0.6f;
        Gx[9] = 7.01f;
        Gy[9] = -0.6f;
        Gx[10] = 9;
        Gy[10] = -0.8f;
        Gx[11] = 11;
        Gy[11] = -0.59f;
        Gx[12] = 12;
        Gy[12] = -0.59f;
        Gx[13] = 14;
        Gy[13] = -0.8f;
        Gx[14] = 1000;
        Gy[14] = -0.8f;
    
    }
    
    
    float gndHeight(float x){
	int n;
	float x1,x2,y1,y2;

	for (n=0; n<npts-1; n++) {
		if (Gx[n+1]>=x) break;
	}
	x1 = Gx[n];
	x2 = Gx[n+1];
	y1 = Gy[n];
	y2 = Gy[n+1];
	if (npts==0 || n==(npts-1)) return 0;
	float y = y1 + (x-x1)*(y2-y1)/(x2-x1);   // interpolate
	return y;        
    }
    
    
    void drawTics(Graphics g, Matrix3x3 transform, float ticSpacing, int nTics, float ticLength, int ticLW){
        myPoint2D p1 = new myPoint2D();
        myPoint2D p2 = new myPoint2D();        
	for (int n=0; n<nTics; n++) {
		float x1 = n*ticSpacing;
		float y1 = gndHeight(x1);
		float x2 = -1*x1;
		float y2 = gndHeight(x2);
                p1 = transform.multiplyBy(new myPoint2D(x1,y1));
                p2 = transform.multiplyBy(new myPoint2D(x1,y1-ticLength));
                g.drawLine((int)p1.x, -(int)p1.y, (int)p2.x,-(int)p2.y);
                p1 = transform.multiplyBy(new myPoint2D(x2,y2));
                p2 = transform.multiplyBy(new myPoint2D(x2,y2-ticLength));
                g.drawLine((int)p1.x, -(int)p1.y, (int)p2.x,-(int)p2.y);
	}
    }
    
    void draw(Graphics g, Matrix3x3 transform){
        // draw ground
        myPoint2D p1 = new myPoint2D();
        myPoint2D p2 = new myPoint2D();
        
        // draw the left upper leg
        g.setColor(new Color(40,40,40));
        Graphics2D g2 = (Graphics2D)g;
        BasicStroke bs = new BasicStroke(3);
        g2.setStroke(bs);
        p1 = transform.multiplyBy(new myPoint2D(Gx[0],Gy[0]));

        for (int n=1; n<npts; n++) {
		p2 = transform.multiplyBy(new myPoint2D(Gx[n],Gy[n]));
                g.drawLine((int)p1.x, -(int)p1.y, (int)p2.x,-(int)p2.y);
                p1 = p2;
        }

	if (DrawTics)
		drawTics(g, transform, 1.0f, 300, 0.08f, 3);     // draw major tics        
    
        BasicStroke bs2 = new BasicStroke(1);
        g2.setStroke(bs2);        
    }
    
}
