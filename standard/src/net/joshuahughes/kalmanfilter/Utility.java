package net.joshuahughes.kalmanfilter;

import java.util.Arrays;
import java.util.Random;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.LUDecomposition;

public class Utility {
	public static Random rand = new Random(9832927l);
    public static double[][] sum( double[][] s0, double[][] s1 )
    {
        double[][] sum = new double[s0.length][s0[0].length];
        for(int x=0;x<sum.length;x++)
            for(int y=0;y<sum[0].length;y++)
                sum[x][y] = s0[x][y] + s1[x][y];
        return sum;
    }
    public static double[][] inverse( double[][] matrix )
    {
        return new LUDecomposition( new Array2DRowRealMatrix( matrix ) ).getSolver( ).getInverse( ).getData( );
    }
    public static double[][] diagonal(int dim,double value)
    {
        double[][] diagonal = new double[dim][dim];
        for(int index=0;index<dim;index++)
            diagonal[index][index] = value;
        return diagonal;
    }
    public static double[][] identity(int dim)
    {
    	return diagonal(dim,1);
    }
    public static double[][] difference( double[][] s0, double[][] s1 )
    {
        double[][] difference = new double[s0.length][s0[0].length];
        for(int x=0;x<difference.length;x++)
            for(int y=0;y<difference[0].length;y++)
                difference[x][y] = s0[x][y] - s1[x][y];
        return difference;
    }
    public static double[][] product( double[][] m1,double[][] m2)
    {
        double sum = 0;
        int dim1 = m1.length;
        int midDim = m2.length;
        int dim2 = m2[0].length;
        double[][] product = new double[dim1][dim2];
        for (int c = 0; c < dim1; c++) {
            for (int d = 0; d < dim2; d++) {
              for (int k = 0; k < midDim; k++) {
                sum = sum + m1[c][k]*m2[k][d];
              }
       
              product[c][d] = sum;
              sum = 0;
            }
        }
        return product;
    }

    
    public static double[][] getRk( int dim1, int dim2 )
    {
        int iDim = dim1*dim2;
        double[][] identity = new double[iDim][iDim];
        for(int index=0;index<iDim;index++)
            identity[index][index] = .01*rand.nextDouble();
        return identity;
    }
    
    public static double[][] getH( int dim1, int dim2 )
    {
        int hDim = dim1*dim2;
        double[][] H = new double[hDim][hDim];
        for(int index=0;index<hDim;index++)
            H[index][index] = 1;
        return H;
    }
    public static double[][] getzk( int tgtCnt, int stateCount, double[][] data )
    {
        int dim = stateCount*tgtCnt;
        double[][] zk = new double[dim][1];
        //skip over
        int xIndex = 0;
        int index = 0;
        while(xIndex+stateCount<zk.length && index+stateCount<data[0].length)
        {
            for(int i=0;i<stateCount;i++)
            {
                zk[xIndex++][0] = data[0][i+index];
            }
            index+=2*stateCount;
        }
        return zk;
    }
    public static double[][] getPk1k1( int tgtCnt, int stateCount, double[][] zk )
    {
        int dim = stateCount*tgtCnt;
        double[][] xk1k1 = new double[1][dim];
        //skip over
        int xIndex = 0;
        int startIndex=0;
        int index = startIndex;
        while(xIndex+stateCount<xk1k1[0].length && index+stateCount<zk.length)
        {
            for(int i=0;i<stateCount;i++)
            {
                xk1k1[0][xIndex++] = zk[0][i+stateCount+index];
            }
            index+=2*stateCount;
        }
        return xk1k1;
    }
    public static double[][] getFk( int tgtCnt,int stateCount, double dt )
    {
        int dim = stateCount*tgtCnt;
        double[][] Fk = new double[dim][dim];
        for(int i=0;i<dim;i++)
        {
            Fk[i][i] = 1;
            if(i<dim-2 && i%4<2)
            {
                Fk[i][i+2] = dt;
            }
        }
        return Fk;
    }
    public static double[][] transpose( double[][] matrix )
    {
        double[][] transpose = new double[matrix[0].length][matrix.length];
        for(int x=0;x<transpose.length;x++)
            for(int y=0;y<transpose[0].length;y++)
                transpose[x][y] = matrix[y][x];
        return transpose;
    }
    public static void print(double[][] matrix,String... strings)
    {
        System.out.println("-------------");
        for(String s : strings)
            System.out.println(s);
        System.out.println(matrix.length+"\t"+matrix[0].length);
        for(double[] array : matrix)
            System.out.println(Arrays.toString( array ));
    }
	public static double[][] diagonal(double... values) {
		double[][] diagonal = new double[values.length][values.length];
		for(int index=0;index<diagonal.length;index++)
			diagonal[index][index] = values[index];
		return diagonal;
	}

}
