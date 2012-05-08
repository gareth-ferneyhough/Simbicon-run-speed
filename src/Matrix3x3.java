/*
 * Matrix3x3.java
 *
 * Created on May 22, 2007, 6:39 PM
 *
 * To change this template, choose Tools | Template Manager
 * and open the template in the editor.
 */

import java.math.*;



/**
 *
 * @author Stel-l
 */
public class Matrix3x3 {
    float data[][] = new float[3][3];
    
    /** Creates a new instance of Matrix3x3 */
    public Matrix3x3() {
        for (int i=0;i<3;i++)
            for (int j=0;j<3;j++)
                if (i == j)
                    data[i][j] = 1;
                else
                    data[i][j] = 0;
    }
    
    /** This method returns a rotation matrix - rotate about the z axis by x rad */
    public static Matrix3x3 getRotationMatrix(float x){
        Matrix3x3 result = new Matrix3x3();
        result.data[0][0] = (float)Math.cos(x);
        result.data[0][1] = (float)-Math.sin(x);
        result.data[1][0] = (float)Math.sin(x);
        result.data[1][1] = (float)Math.cos(x);
        
        return result;
    }

    /** This method returns a translation matrix */
    public static Matrix3x3 getTranslationMatrix(float x, float y){
        Matrix3x3 result = new Matrix3x3();
        result.data[0][2] = x;
        result.data[1][2] = y;
        return result;
    }    
    
    /** This method returns a scaling matrix */
    public static Matrix3x3 getScalingMatrix(float s){
        Matrix3x3 result = new Matrix3x3();
        result.data[0][0] = s;
        result.data[1][1] = s;
        return result;
    }
    
    /** This method multiplies the point that is passed in by the current matrix and returns the resulting point */
    public myPoint2D multiplyBy(myPoint2D a){
        myPoint2D result = new myPoint2D();
        result.x = data[0][0] * a.x + data[0][1] * a.y + data[0][2];
        result.y = data[1][0] * a.x + data[1][1] * a.y + data[1][2];
        
        return result;
    }
    
    public Matrix3x3 multiplyBy(Matrix3x3 a){
        Matrix3x3 result = new Matrix3x3();
        for (int i=0;i<3;i++)
            for (int j=0;j<3;j++){
                result.data[i][j] = 0;
                for (int k=0;k<3;k++)
                    result.data[i][j] += this.data[i][k] * a.data[k][j];
            }
        return result;
    }
    
}
