package net.joshuahughes.kalmanfilter.associator;

import java.util.Arrays;

import net.joshuahughes.kalmanfilter.Utility;

public class HungarianAssociator implements Associator{

	int observationCount;
	int targetCount;
	int stateCount;
	public HungarianAssociator( int observationCount, int targetCount, int stateCount )
	{
		this.observationCount = observationCount;
		this.targetCount = targetCount;
		this.stateCount = stateCount;
	}
	@Override
	public double[][] associate(double[][] observations, double[][] stateEstimates, double[][] estimateCovariance) {
		double[][] costMatrix = new double[targetCount][targetCount];
		for(int o=0;o<costMatrix.length;o++)
			for(int s = 0;s<costMatrix.length;s++)
				costMatrix[s][o] = Math.hypot(stateEstimates[s][0] - observations[o][0],stateEstimates[s+targetCount][0]-observations[o+targetCount][0]);
		HungarianAlgorithm algorithm = new HungarianAlgorithm(costMatrix);
		int[] association = algorithm.execute();
		double[][] newObs = new double[observations.length][observations[0].length];
		for(double[] newArray : newObs)Arrays.fill(newArray,Double.NaN);
		for(int o = 0;o<newObs.length;o++)
		{
			int ai = o%targetCount;
			if(association[ai]<0)continue;
			int offset = targetCount*(o/targetCount);
			newObs[o][0] = observations[offset+association[ai]][0];
		}
//		if(Math.abs(stateEstimates[1][0] - newObs[1][0])>100)
		{
			System.out.println(stateEstimates[1][0]+"\t"+newObs[1][0]);
			System.out.println(stateEstimates[5][0]+"\t"+newObs[5][0]);
			Utility.print( Utility.transpose(stateEstimates), "stateEstimates" );
			Utility.print( Utility.transpose(observations), "observations" );
			Utility.print( costMatrix, "cm" );
			Utility.print( Utility.transpose(newObs), "newObs" );
			System.out.println("ass"+Arrays.toString( association ));
			System.out.println("**************************************");
			System.exit(1);
		}
		return newObs;
	}
	public double[][] reshape( double[] vector )
	{
		double[][] reshape = new double[vector.length/targetCount][];
		for(int index=0;index<reshape.length;index++)
			reshape[index] = Arrays.copyOfRange( vector, index*targetCount,(index+1)*targetCount);
		return reshape;
	}
}
