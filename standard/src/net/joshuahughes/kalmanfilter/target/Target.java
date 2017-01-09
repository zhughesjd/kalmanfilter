package net.joshuahughes.kalmanfilter.target;

import net.joshuahughes.kalmanfilter.source.Source.Data;

public interface Target {
	public void receive(Data data);
	public void receive(double[][] stateEstimates,double[][] estimateCovariance);
}
