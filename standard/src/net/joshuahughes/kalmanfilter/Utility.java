package net.joshuahughes.kalmanfilter;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.Random;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.SingularMatrixException;

import net.joshuahughes.kalmanfilter.associator.Associator;

public class Utility {
	public static enum KalmanKey{xkk1,Pkk1,xkk,Pkk,yk,Sk,Kk};
	public static Random rand = new Random(9832927l);
	public static double invalid = 10000000d;
	public static Associator passThroughAssociator = new Associator(){
		@Override
		public double[][] associate(double[][] observations, double[][] stateEstimates, double[][] estimateCovariance) {
			return observations;
		}
	};
	public static double[][] sum( double[][] s0, double s1 )
	{
		double[][] sum = create(s0);
		for(int x=0;x<sum.length;x++)
			for(int y=0;y<sum[0].length;y++)
				sum[x][y] = s0[x][y] + s1;
		return sum;
	}
	public static double[][] sum( double s1, double[][] s0 )
	{
		return sum(s0,s1);
	}
	public static double[][] sum( double[][] s0, double[][] s1 )
	{
		double[][] sum = create(s0);
		for(int x=0;x<sum.length;x++)
			for(int y=0;y<sum[0].length;y++)
				sum[x][y] = s0[x][y] + s1[x][y];
		return sum;
	}
	public static double[][] inverse( double[][] matrix )
	{
		try{
			return new LUDecomposition( new Array2DRowRealMatrix( matrix ) ).getSolver( ).getInverse( ).getData( );
		}catch(SingularMatrixException e){
			// Could be diagonal is effectively zero
			// Try flooring it
			return new LUDecomposition( new Array2DRowRealMatrix( diagonalMax(matrix,.0000001) ) ).getSolver( ).getInverse( ).getData( );
		}
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
		double[][] difference = create(s0);
		for(int x=0;x<difference.length;x++)
			for(int y=0;y<difference[0].length;y++)
				difference[x][y] = s0[x][y] - s1[x][y];
		return difference;
	}
	public static double[][] product( double[][] m1,double[][] m2)
	{
		double sum = 0;
		int dim1 = m1.length;
		int midDim = Math.max(m1[0].length,m2.length);
		int dim2 = m2[0].length;
		double[][] product = new double[dim1][dim2];
		for (int c = 0; c < dim1; c++) {
			for (int d = 0; d < dim2; d++) {
				for (int k = 0; k < midDim; k++) {
					if(Double.isFinite(m1[c][k]) && Double.isFinite(m2[k][d]))
						sum = sum + m1[c][k]*m2[k][d];
				}

				product[c][d] = sum;
				sum = 0;
			}
		}
		return product;
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
			System.out.print(s);
		System.out.println(" - dim: "+matrix.length+" x "+matrix[0].length);
		for(double[] array : matrix)
			System.out.println(Arrays.toString( array ));
	}
	public static double[][] diagonal(double... values) {
		double[][] diagonal = new double[values.length][values.length];
		for(int index=0;index<diagonal.length;index++)
			diagonal[index][index] = values[index];
		return diagonal;
	}
	public static double[][] replace(double[][] candidate, double[][] replacement) {
		double[][] replace = new double[candidate.length][candidate[0].length];
		for(int x=0;x<replace.length;x++)
			for(int y=0;y<replace[0].length;y++)
				replace[x][y] = Double.isNaN(candidate[x][y])?replacement[x][y]:candidate[x][y];
				return replace;
	}
	public static double[][] product(double m1, double[][] m2) {
		return product(m2,m1);
	}
	public static double[][] product(double[][] m1, double m2) {
		double[][] product = create(m1);
		for(int x=0;x<product.length;x++)
			for(int y=0;y<product[0].length;y++)
				product[x][y] = m1[x][y]*m2;
		return product;
	}
	public static double[][] elementalProduct(double[][] m1, double[][] m2) {
		double[][] product = create(m1);
		for(int x=0;x<product.length;x++)
			for(int y=0;y<product[0].length;y++)
				product[x][y] = m1[x][y]*m2[x][y];
		return product;
	}
	public static double[][] abs(double[][] matrix) {
		double[][] abs = create(matrix);
		for(int x=0;x<abs.length;x++)
			for(int y=0;y<abs[0].length;y++)
				abs[x][y] = Math.abs(matrix[x][y]);
		return abs;
	}
	public static double[][] diagonalMax(double[][] matrix, double value) {
		double[][] max = create(matrix);
		for(int x=0;x<matrix.length;x++)
			for(int y=0;y<matrix[0].length;y++)
				max[x][y] = x==y?Math.max(matrix[x][y],value):matrix[x][y];
				return max;
	}
	public static double[][] create(double[][] matrix)
	{
		return new double[matrix.length][matrix[0].length];
	}
	public static double[][] varianceDim2(double[][] matrix) {
		double[][] sum = new double[1][matrix[0].length];
		for(int y=0;y<matrix.length;y++)
			sum = Utility.sum(sum, new double[][]{matrix[y]});
		double[][] mean = Utility.product(sum,1d/matrix.length);
		sum = new double[1][matrix[0].length];
		for(int y=0;y<matrix.length;y++)
		{
			double[][] diff = Utility.difference(mean,new double[][]{matrix[y]});
			double[][] sqrd = Utility.elementalProduct(diff,diff);
			sum = Utility.sum(sum,sqrd);
		}
		return Utility.product(sum,1d/(matrix.length-1));
	}
	public static double[][] varianceDim1(double[][] matrix)
	{
		return transpose(varianceDim2(transpose(matrix)));
	}

	public static LinkedHashMap<KalmanKey,double[][]> predictAssociateUpdate(double[][] xk1k1, double[][] Pk1k1, double[][] Fk,double[][] Qk1, double[][] zk, double[][] Hk, double[][] Rk,Associator... associators)
	{
		return predictAssociateUpdate(xk1k1, Pk1k1, Fk, Qk1, zk, Hk, Rk, null, associators);
	}
	public static LinkedHashMap<KalmanKey,double[][]> predictAssociateUpdate(double[][] xk1k1, double[][] Pk1k1, double[][] Fk,double[][] Qk1, double[][] zk, double[][] Hk, double[][] Rk,LinkedHashMap<KalmanKey,double[][]> map,Associator... associators)
	{
		// Setup
		if(map == null) map = new LinkedHashMap<>();
		Associator associator = associators == null || associators.length<=0? Utility.passThroughAssociator :associators[0];

		// Predict
		map.putAll(predict(xk1k1,Pk1k1,Fk,Qk1,map));

		// Associate by Rearranging
		zk = associator.associate(zk,map.get(KalmanKey.xkk1),map.get(KalmanKey.Pkk1));

		// Update
		map.putAll(update(map.get(KalmanKey.xkk1),map.get(KalmanKey.Pkk1),zk,Hk,Rk,map));

		// Return all results
		return map;
	}
	public static LinkedHashMap<KalmanKey,double[][]> predict(double[][] xk1k1, double[][] Pk1k1, double[][] Fk,double[][] Qk1) {
		return predict(xk1k1, Pk1k1, Fk, Qk1, null);
	}
	public static LinkedHashMap<KalmanKey,double[][]> predict(double[][] xk1k1, double[][] Pk1k1, double[][] Fk,double[][] Qk1,LinkedHashMap<KalmanKey,double[][]> map) {
		if(map == null)map = new LinkedHashMap<>();
		map.put(KalmanKey.xkk1,product(Fk,xk1k1));
		map.put(KalmanKey.Pkk1,sum(product(product(Fk,Pk1k1),transpose(Fk)),Qk1));
		return map;
	}
	public static LinkedHashMap<KalmanKey,double[][]> update(double[][] xkk1, double[][] Pkk1,double[][] zk,double[][] Hk,double[][] Rk) {
		return update(xkk1, Pkk1, zk, Hk, Rk, null);
	}
	public static LinkedHashMap<KalmanKey,double[][]> update(double[][] xkk1, double[][] Pkk1,double[][] zk,double[][] Hk,double[][] Rk,LinkedHashMap<KalmanKey,double[][]> map) {
		if(map == null)map = new LinkedHashMap<>();
		double[][] HkT = transpose(Hk);
		map.put(KalmanKey.yk,difference(zk,product(Hk,xkk1)));
		map.put(KalmanKey.Sk,sum(product(Hk,product(Pkk1,HkT)),Rk));
		map.put(KalmanKey.Kk,product(Pkk1,product(HkT,inverse(map.get(KalmanKey.Sk)))));
		map.put(KalmanKey.xkk,sum(xkk1,product(map.get(KalmanKey.Kk),map.get(KalmanKey.yk))));
		map.put(KalmanKey.Pkk,product(difference(identity(map.get(KalmanKey.Kk).length),product(map.get(KalmanKey.Kk),Hk)),Pkk1));
		return map;
	}
}
