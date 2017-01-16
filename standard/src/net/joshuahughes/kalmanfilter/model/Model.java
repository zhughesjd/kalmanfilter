package net.joshuahughes.kalmanfilter.model;

public interface Model {
    public double[][] getxk0k0();
    public double[][] getPk0k0();
	public double[][] getFk(double dt);
	public double[][] getHk(double time);
	public double[][] getQk1(double priorTime);
	public double[][] getRk(double time);
}
