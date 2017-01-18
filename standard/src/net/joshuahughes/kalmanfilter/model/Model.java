package net.joshuahughes.kalmanfilter.model;

import java.util.LinkedHashMap;

import net.joshuahughes.kalmanfilter.Utility.KalmanKey;

public interface Model {
	public LinkedHashMap<KalmanKey,double[][]> map = new LinkedHashMap<>();
	public double[][] getxk0k0();
    public double[][] getPk0k0();
	public double[][] getFk(double dt);
	public double[][] getHk(double time);
	public double[][] getQk1(double priorTime);
	public double[][] getRk(double time);
}
