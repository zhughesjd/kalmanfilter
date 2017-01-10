package net.joshuahughes.kalmanfilter.source;

import net.joshuahughes.kalmanfilter.source.Source.Data;

public interface Source extends Iterable<Data>{
	public static class Data{
		public double time = Double.NaN;
		public double[] measurements = null;
		public double[] truth = null;
		public Data(double time, double[] measurements) {
			this(time,measurements,null);
		}
		public Data(double time, double[] measurements,double[] truth) {
			this.time = time;
			this.measurements = measurements;
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
