package net.joshuahughes.kalmanfilter.associator;

import java.util.Arrays;

import net.joshuahughes.kalmanfilter.Utility;

public class HungarianAssociator implements Associator{

	int observationCount;
	int targetCount;
	public HungarianAssociator( int observationCount, int targetCount )
	{
		this.observationCount = observationCount;
		this.targetCount = targetCount;
	}
	@Override
	public double[][] associate(double[][] observations, double[][] stateEstimates, double[][] estimateCovariance) {
		double[][] obs = reshape(Utility.transpose( observations )[0],targetCount);
		double[][] ste = reshape(Utility.transpose( stateEstimates )[0],targetCount);
		//    	System.exit(1);
		double[][] costMatrix = new double[ste[0].length][obs[0].length];
		for(int s = 0;s<costMatrix.length;s++)
			for(int o=0;o<costMatrix[s].length;o++)
				costMatrix[s][o] = Math.hypot(ste[0][s] - obs[0][o], ste[1][s] - obs[1][o]);
		HungarianAlgorithm algorithm = new HungarianAlgorithm(costMatrix);
		int[] association = algorithm.execute();
		double[][] newObs = new double[observations.length][observations[0].length];
		for(double[] newArray : newObs)
			Arrays.fill(newArray,Double.NaN);
		for(int offset=0;offset<observations.length;offset+=targetCount)
			for(int a = 0;a<association.length;a++)
				if(association[a]>-1)
				{
					newObs[offset+a][0] = observations[offset+association[a]][0];
				}
		return newObs;
	}
	public double[][] reshape( double[] vector, int stateCount )
	{
		double[][] reshape = new double[vector.length/stateCount][];
		for(int index=0;index<reshape.length;index++)
			reshape[index] = Arrays.copyOfRange( vector, index*stateCount,(index+1)*stateCount);
		return reshape;
	}
}
