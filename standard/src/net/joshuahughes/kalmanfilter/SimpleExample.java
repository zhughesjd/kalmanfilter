package net.joshuahughes.kalmanfilter;

import java.awt.Color;
import java.util.Arrays;
import java.util.Random;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.LUDecomposition;

import net.joshuahughes.kalmanfilter.receiver.JDialogReceiver;

public class SimpleExample
{
	static Random rand = new Random(437583478);//bad1,bad2
	public static boolean useVelocityMeasure = true;
	public static void main(String[] args) throws Exception
	{
		int maxTargets = 2;
		int timeCount = 1000;
		double oxx = 40;
		double oxy = oxx;
		double ovx = 0;
		double ovy = 0;
		double oa1 = 0;
		double oa2 = 0;
		double on1 = 0;
		double on2 = 0;
		JDialogReceiver receiver = new JDialogReceiver(timeCount, timeCount);

		double[][] truth = generate(timeCount);
		double[][] zks = perturb(truth,0,oxx,oxy,ovx,ovy,oa1,oa2,on1,on2,oxx,oxy,ovx,ovy,oa1,oa2,on1,on2);
		int stateCount = truth[0].length/maxTargets/2;
		// Using https://en.wikipedia.org/wiki/Kalman_filter#Details
		double tk1=Double.NaN;
		double[][] xk1k1 = null;
		double[][] Pk1k1 = null;
		double[][] I = getIdentity(maxTargets,stateCount);
		double[][] Qk1 = getQk1(maxTargets,stateCount);
		double[][] Rk = getRk(maxTargets,stateCount);
		double[][] Hk = getH(maxTargets,stateCount);
		double[][] HkT = transpose(Hk);

		for(int index=0;index<zks.length;index++)
		{
			double[] data = zks[index];
			double tk = data[0];
			double[][] zk = getzk(data);
			receiver.receive(truth[index][1],truth[index][2],Color.yellow);
			receiver.receive(truth[index][9],truth[index][10],Color.white);
			receiver.receive(zk[0][0],zk[0][1],Color.blue);
			receiver.receive(zk[0][8],zk[0][9],Color.cyan);
			if(Double.isNaN(tk1))
			{
				xk1k1 = getzk(maxTargets,stateCount,zk);
				Pk1k1 = getPk1k1(maxTargets,stateCount,zk);
				tk1 = tk;
				continue;
			}
			double[][] Fk =getFk(maxTargets,stateCount,tk-tk1);
			double[][] xkk1 = product(Fk,xk1k1);
			double[][] Pkk1 = sum(product(product(Fk,Pk1k1),transpose(Fk)),Qk1);
			//Skip if no measurements
			if(zk.length>0)
			{
				zk = getzk(maxTargets,stateCount,zk);
				double[][] yk = difference(zk,product(Hk,xkk1));
				double[][] Sk = sum(product(Hk,product(Pkk1,HkT)),Rk);
				double[][] Kk = product(Pkk1,product(HkT,inverse(Sk)));
				double[][] xkk = sum(xkk1,product(Kk,yk));
				double[][] Pkk = product(difference(I,product(Kk,Hk)),Pkk1);
				xk1k1 = xkk;
				Pk1k1 = Pkk;
			}
			tk1 = tk;
			receiver.receive(xk1k1[0][0],xk1k1[1][0],Color.red);
			receiver.receive(xk1k1[4][0],xk1k1[5][0],Color.green);
			Thread.sleep(10);
		}
	}
	public static double[][] perturb(double[][] data,double... sigmas)
	{
		double[][] perturb = new double[data.length][data[0].length];
		for(int x=0;x<perturb.length;x++)
			for(int y=0;y<perturb[0].length;y++)
				perturb[x][y] = data[x][y] + sigmas[y]*rand.nextGaussian();
		return perturb;
	}
	public static double[][] generate(int timeCount) {
		double[][] generate = new double[timeCount][];
		double vx00 = useVelocityMeasure?0:0;
		double vy00 = useVelocityMeasure?1:0;

		double vx01 = useVelocityMeasure?1:0;
		double vy01 = useVelocityMeasure?1:0;
		
		double vx10 = useVelocityMeasure?1:0;
		double vy10 = useVelocityMeasure?0:0;
		
		double vx11 = useVelocityMeasure?1:0;
		double vy11 = useVelocityMeasure?-1:0;

		for(int index=0;index<generate.length;index++)
		{
			int offset = 50;
			int arrayPos = index+offset;
			if(index<generate.length/2)
				generate[index] = new double[]{index,500,arrayPos,vx00,vy00,0.5,0.5,0.5,0.5,arrayPos,500,vx10,vy10,0.5,0.5,0.5,0.5};
			else
				generate[index] = new double[]{index,arrayPos-offset,arrayPos,vx01,vy01,0.5,0.5,0.5,0.5,arrayPos,generate.length-arrayPos+offset,vx11,vy11,0.5,0.5,0.5,0.5};
		}
//		for(int index=0;index<generate.length;index++)
//			if(index<generate.length/2)
//				generate[index] = new double[]{index,500,index,0,1,0.5,0.5,0.5,0.5,index,500,1,0,0.5,0.5,0.5,0.5};
//			else
//				generate[index] = new double[]{index,index,index,0,1,0.5,0.5,0.5,0.5,index,generate.length-index,1,0,0.5,0.5,0.5,0.5};

		return generate;
	}
	private static double[][] getzk(double[] data)
	{
		return new double[][]{Arrays.copyOfRange(data,1,data.length)};
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
	private static double[][] getH( int dim1, int dim2 )
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
	private static double[][] getFk( int tgtCnt,int stateCount, double dt )
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
}
