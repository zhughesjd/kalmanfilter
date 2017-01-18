package net.joshuahughes.kalmanfilter.model;

import java.util.stream.IntStream;

import net.joshuahughes.kalmanfilter.Utility;

public class SingleTargetKinematicModel implements Model
{
	public static enum Kinematic{none,position,velocity,acceleration,jerk,snap,crackle,pop}
	int spatialDimCount;
	int observationCount;
	Kinematic observation;
	Kinematic motionModel;
	int stateCount;
	double[][] Fk;
	double[][] Hk;
	double lastDt = Double.NaN;
	public SingleTargetKinematicModel(int spatialDimCount,Kinematic observation,Kinematic motionModel)
	{
		this.observation = observation;
		this.spatialDimCount = spatialDimCount;
		this.motionModel = motionModel;
		stateCount = spatialDimCount*motionModel.ordinal();
		observationCount = spatialDimCount*observation.ordinal();
		Hk = new double[observationCount][stateCount];
		IntStream.range(0, Hk.length).forEach(i->Hk[i][i] = 1d);
	}
	@Override
	public double[][] getxk0k0() {
		return null;
	}
	@Override
	public double[][] getPk0k0() {
		// TODO Auto-generated method stub
		return null;
	}
	@Override
	public double[][] getFk(double dt){
		if(dt != lastDt)
		{
			Fk = new double[stateCount][stateCount];
			for(int mIndex=0;mIndex<motionModel.ordinal();mIndex++)
			{
				System.out.println(mIndex);
				int yOffset = mIndex*spatialDimCount;
				double entry = (1d/factorial(mIndex))*Math.pow(dt,mIndex);
				for(int index = 0;index+yOffset<Fk[0].length;index++)
					Fk[index][index+yOffset] = entry;
			}
			lastDt = dt;
		}
		return Fk;
	}
	@Override
	public double[][] getHk(double time) {
		return Hk;
	}
	@Override
	public double[][] getQk1(double priorTime) {
		return null;
	}
	@Override
	public double[][] getRk(double time) {
		return null;
	}
	public static void main(String[] args)
	{
		System.out.println(factorial(6));
		SingleTargetKinematicModel model = new SingleTargetKinematicModel(3,Kinematic.position, Kinematic.snap);
		Utility.print(model.getFk(1));
	}
	public static int factorial(int x)
	{
		if(x<2) return 1;
		return x*factorial(x-1);
	}
}
