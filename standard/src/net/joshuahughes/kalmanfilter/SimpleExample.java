package net.joshuahughes.kalmanfilter;

import java.util.Arrays;
import java.util.Random;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.LUDecomposition;

import net.joshuahughes.kalmanfilter.source.SimpleExamplePositionSource;
import net.joshuahughes.kalmanfilter.source.Source;
import net.joshuahughes.kalmanfilter.source.Source.Data;
import net.joshuahughes.kalmanfilter.target.JDialogTarget;

public class SimpleExample
{
	static Random rand = new Random(437583478);//bad1,bad2
	public static void main(String[] args) throws Exception
	{
		int maxTargets = 2;
		int timeCount = 1000;
		int stateCount = 4;

		JDialogTarget target = new JDialogTarget(timeCount, timeCount);
		Source source = new SimpleExamplePositionSource(timeCount, maxTargets, stateCount, true);

		// Using https://en.wikipedia.org/wiki/Kalman_filter#Details
		double tk1=Double.NaN;
		double[][] xk1k1 = null;
		double[][] Pk1k1 = null;
		double[][] I = getIdentity(maxTargets,stateCount);
		double[][] Qk1 = getQk1(maxTargets,stateCount);
		double[][] Rk = getRk(maxTargets,stateCount);
		double[][] Hk = getHk(maxTargets,stateCount);
		double[][] HkT = transpose(Hk);
		
		for(Data data : source)
		{
			double tk = data.time;
			double[][] zk = transpose(new double[][]{data.measurements});
			target.receive(data);
			if(Double.isNaN(tk1)) // initialize xk_0k_0 and Pk_0k_0 with first measurements
			{
				xk1k1 = zk;
				Pk1k1 = source.getPk0k0();
			}
			else //otherwise 
			{
				double[][] Fk = source.getFk(tk-tk1);
				double[][] xkk1 = product(Fk,xk1k1);
				double[][] Pkk1 = sum(product(product(Fk,Pk1k1),transpose(Fk)),Qk1);
				//Skip if no measurements  *** this needs to be tested ***
				if(zk.length>0)
				{
					double[][] yk = difference(zk,product(Hk,xkk1));
					double[][] Sk = sum(product(Hk,product(Pkk1,HkT)),Rk);
					double[][] Kk = product(Pkk1,product(HkT,inverse(Sk)));
					double[][] xkk = sum(xkk1,product(Kk,yk));
					double[][] Pkk = product(difference(I,product(Kk,Hk)),Pkk1);
					xk1k1 = xkk;
					Pk1k1 = Pkk;
				}
			}
			tk1 = tk;
			target.receive(xk1k1,Pk1k1);
			Thread.sleep(10);
		}
	}
	private static double[][] sum( double[][] s0, double[][] s1 )
	{
		double[][] sum = new double[s0.length][s0[0].length];
		for(int x=0;x<sum.length;x++)
			for(int y=0;y<sum[0].length;y++)
				sum[x][y] = s0[x][y] + s1[x][y];
		return sum;
	}
	private static double[][] inverse( double[][] matrix )
	{
		return new LUDecomposition( new Array2DRowRealMatrix( matrix ) ).getSolver( ).getInverse( ).getData( );
	}
	private static double[][] getQk1( int dim1, int dim2 )
	{
		int iDim = dim1*dim2;
		double[][] Qk1 = new double[iDim][iDim];
		for(int index=0;index<iDim;index++)
			Qk1[index][index] = .000001;
		return Qk1;
	}
	private static double[][] getRk( int dim1, int dim2 )
	{
		int iDim = dim1*dim2;
		double[][] identity = new double[iDim][iDim];
		for(int index=0;index<iDim;index++)
			identity[index][index] = 100;
		return identity;
	}
	private static double[][] getIdentity( int dim1, int dim2 )
	{
		int iDim = dim1*dim2;
		double[][] identity = new double[iDim][iDim];
		for(int index=0;index<iDim;index++)
			identity[index][index] = 1;
		return identity;
	}
	private static double[][] getHk( int dim1, int dim2 )
	{
		int hDim = dim1*dim2;
		double[][] H = new double[hDim][hDim];
		for(int index=0;index<hDim;index++)
			H[index][index] = 1;
		return H;
	}
	private static double[][] difference( double[][] s0, double[][] s1 )
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
		//this ensures the second dim of m1 is equal to the first dim of m2
		int midDim = Math.max(m1[0].length, m2.length);
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
	public static double[][] getzk( int tgtCnt, int stateCount, double[][] data )
	{
		int dim = stateCount*tgtCnt;
		double[][] zk = new double[dim][1];
		//skip over
		int xIndex = 0;
		int index = 0;
		while(xIndex-stateCount<zk.length && index+stateCount<data[0].length)
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
		int index = 0;
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
}
