package net.joshuahughes.kalmanfilter.source;

import net.joshuahughes.kalmanfilter.source.Source.Data;

public interface Source extends Iterable<Data>{
	public static class Data{
		public double time = Double.NaN;
		public double[][] observations = null;
		public double[][] truth = null;
		public Data(double time, double[][] observations) {
			this(time,observations,null);
		}
		public Data(double time, double[][] observations,double[][] truth) {
			this.time = time;
			this.observations = observations;
			this.truth = truth;
		}
	}

	public double[][] getPk0k0();
	public double[][] getFk(double dt);
	public double[][] getHk(double time);
	public double[][] getQk1(double priorTime);
	public double[][] getRk(double time);
	public Data getData0();
}
