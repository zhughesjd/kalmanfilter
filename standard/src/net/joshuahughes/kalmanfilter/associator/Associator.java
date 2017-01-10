package net.joshuahughes.kalmanfilter.associator;

public interface Associator{
	public double[][] associate(double[][] observations, double[][] stateEstimates, double[][] estimateCovariance);
}
