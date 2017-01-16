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
        Utility.print( Utility.transpose(observations), "observations" );
        Utility.print( Utility.transpose(stateEstimates), "stateEstimates" );
		double[][] costMatrix = new double[targetCount][targetCount];
        for(int s = 0;s<costMatrix.length;s++)
            for(int o=0;o<costMatrix.length;o++)
				costMatrix[o][s] = Math.hypot(stateEstimates[s][0] - observations[o][0],stateEstimates[s+targetCount][0]-observations[o+targetCount][0]);
        Utility.print( costMatrix, "cm" );
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
        Utility.print( Utility.transpose(newObs), "newObs" );
        System.out.println("ass"+Arrays.toString( association ));
        System.out.println("**************************************");
        System.exit( 1 );
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
