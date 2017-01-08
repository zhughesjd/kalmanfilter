package net.joshuahughes.kalmanfilter.receiver;

public interface Receiver {
	public void receive(double time,double[][] stateEstimates,double[][] estimateCovariance);
}
