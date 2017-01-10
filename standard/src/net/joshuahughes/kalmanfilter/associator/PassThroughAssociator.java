package net.joshuahughes.kalmanfilter.associator;

public class PassThroughAssociator implements Associator{

	@Override
	public double[][] associate(double[][] observations, double[][] stateEstimates, double[][] estimateCovariance) {
		return observations;
	}
}
