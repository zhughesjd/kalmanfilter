package net.joshuahughes.kalmanfilter.receiver;

import net.joshuahughes.kalmanfilter.source.Source.Data;

public interface Receiver {
	public void receive(Data data);
	public void receive(double[][] stateEstimates,double[][] estimateCovariance);
}
