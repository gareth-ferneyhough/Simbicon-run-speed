/*
 * myPoint2D.java
 *
 * Created on May 22, 2007, 9:57 PM
 *
 * To change this template, choose Tools | Template Manager
 * and open the template in the editor.
 */

/**
 *
 * @author Stel-l
 */
public class myPoint2D {
    public float x;
    public float y;
    /** Creates a new instance of myPoint2D */
    public myPoint2D() {
        x = 0;
        y = 0;
    }
    
    public myPoint2D(float myX, float myY){
        x = myX;
        y = myY;
    }

    public myPoint2D(double myX, double myY){
        x = (float)myX;
        y = (float)myY;
    }    
    
}
