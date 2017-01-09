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

	double[][] getPk0k0();
	double[][] getFk(double dt);
}
