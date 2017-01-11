package net.joshuahughes.kalmanfilter.associator;

import java.util.Arrays;

public class SimpleAssociator implements Associator{

	int observationCount;
	int stateCount;
    public SimpleAssociator( int observationCount, int stateCount )
    {
        this.observationCount = observationCount;
        this.stateCount = stateCount;
    }
    @Override
	public double[][] associate(double[][] observations, double[][] stateEstimates, double[][] estimateCovariance) {
//        double[][] obs = reshape(Utility.transpose( observations )[0],stateCount);
//        double[][] ste = reshape(Utility.transpose( stateEstimates )[0],stateCount);
		return observations;
	}
    public double[][] reshape( double[] vector, int stateCount )
    {
        double[][] reshape = new double[vector.length/stateCount][];
        for(int index=0;index<reshape.length;index++)
            reshape[index] = Arrays.copyOfRange( vector, index,index+stateCount );
        return reshape;
    }
}
