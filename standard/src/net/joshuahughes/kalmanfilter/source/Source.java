package net.joshuahughes.kalmanfilter.source;

import net.joshuahughes.kalmanfilter.source.Source.Data;

public interface Source extends Iterable<Data>
{
	public Data getData0();
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
}
