package net.joshuahughes.kalmanfilter.associator;

import java.util.Arrays;

import net.joshuahughes.kalmanfilter.Utility;

public class HungarianAssociator implements Associator{

	int observationCount;
	int stateCount;
    public HungarianAssociator( int observationCount, int stateCount )
    {
        this.observationCount = observationCount;
        this.stateCount = stateCount;
    }
    @Override
	public double[][] associate(double[][] observations, double[][] stateEstimates, double[][] estimateCovariance) {
        double[][] obs = reshape(Utility.transpose( observations )[0],stateCount);
        double[][] ste = reshape(Utility.transpose( stateEstimates )[0],stateCount);
        double[][] costMatrix = new double[ste.length][obs.length];
        for(int x = 0;x<costMatrix.length;x++)
        	for(int y=0;y<costMatrix[x].length;y++)
        		costMatrix[x][y] = Math.hypot(ste[x][0] - obs[y][0], ste[x][1] - obs[y][1]);
        HungarianAlgorithm algorithm = new HungarianAlgorithm(costMatrix);
        int[] association = algorithm.execute();
        double[][] newObs = new double[observations.length][observations[0].length];
        for(double[] newArray : newObs)
        	Arrays.fill(newArray,Double.NaN);
        for(int sIndex = 0;sIndex<association.length;sIndex++)
        	if(association[sIndex]>-1)
        	{
        		int oStartIndex = sIndex*stateCount;
        		int oIndex= association[sIndex];
        		for(int index=0;index<obs[oIndex].length;index++)
        			newObs[oStartIndex+index][0] = obs[oIndex][index];
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
