/*
 * MyLinAlg.java
 *
 * Created on June 7, 2007, 12:08 AM
 *
 * To change this template, choose Tools | Template Manager
 * and open the template in the editor.
 */

/**
 *
 * @author Stel-l
 */
public class MyLinAlg {
    
    /** Creates a new instance of MyLinAlg */
    public MyLinAlg() {
    }


/************************************************************
	PROC:	lin_eq_solv(A,b,x,n)
	DOES:	Solves a set of n linear equations
************************************************************/

public static int lin_eq_solv(float a[][],float b[],float x[],int n)
{
	int indx[] = new int[40];
	float d = 0;
	int n1;
	int flag;

	flag = ludcmp(a,n,indx,d);
	if (flag==0) return(0);
	lubksb(a,n,indx,b);
	for (n1=0; n1<n; n1++)
		x[n1] = b[n1];
	return(1);
}

/************************************************************
	PROC:	ludcmp()
	DOES:	replaces a matrix by the LU decomposition of itself
************************************************************/

public static float TINY  = 1.0e-20f;

public static int ludcmp(float[][] a, int n, int indx[], float d)
{
	int i,j,k;
	float sum;
	int imax = 0;
	float big,dum,temp;
	float vv[] = new float[n];

	d = 1.0f;
	for (i=0; i<n; i++) {
		big = 0.0f;
		for (j=0; j<n; j++)
			if ((temp=Math.abs(a[i][j]))  > big) big=temp;
		if (big==0.0) {
			nrerror("Singular matrix in routine LUDCMP");
			return(0);
		}
		vv[i] = 1.0f/big;
	}
	for (j=0; j<n; j++) {
		for (i=0; i<j; i++) {
			sum = a[i][j];
			for (k=0; k<i; k++) sum -= a[i][k]*a[k][j];
			a[i][j] = sum;
		}
		big = 0.0f;
		for (i=j; i<n; i++) {
			sum= a[i][j];
			for (k=0; k<j; k++)
				sum -= a[i][k]*a[k][j];
			a[i][j] = sum;
			if ((dum=vv[i]*Math.abs(sum))>=big) {
				big = dum;
				imax = i;
			}
		}
		if (j!=imax) {
			for (k=0; k<n; k++) {
				dum = a[imax][k];
				a[imax][k] = a[j][k];
				a[j][k] = dum;
			}
			d = -(d);
			vv[imax] = vv[j];
		}
		indx[j] = imax;
		if (a[j][j]==0.0) a[j][j]=TINY;
		if (j!=n-1) {
			dum = 1.0f/(a[j][j]);
			for (i=j+1; i<n; i++) a[i][j] *= dum;
		}
	}

	return(1);
}

/************************************************************
	PROC:	lubksb()
	DOES:	does back-substitution for LU decomp
************************************************************/

public static void lubksb(float a[][],int n,int indx[],float b[])
{
	int i,ii=-1,ip,j;
	float sum;

	for (i=0; i<n; i++) {
		ip=indx[i];
		sum=b[ip];
		b[ip]=b[i];
		if (ii!=-1)
			for (j=ii; j<=i-1; j++) sum -= a[i][j]*b[j];
		else if (sum!=0) ii=i;
		b[i] = sum;
	}
	for (i=n-1; i>=0; i--) {
		sum=b[i];
		for (j=i+1; j<n; j++) sum -= a[i][j]*b[j];
		b[i] = sum/a[i][i];
	}
}


/************************************************************
	PROC:	nrerror(str)
	DOES:	numerical recipe error
************************************************************/
private static void nrerror(String str){
        System.out.println("nrerror: " + str);
        System.exit(0);
}

    
    
}
