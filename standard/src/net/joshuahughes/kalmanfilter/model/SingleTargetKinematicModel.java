package net.joshuahughes.kalmanfilter.model;

import java.util.ArrayList;
import java.util.Map.Entry;
import java.util.Arrays;
import java.util.Random;
import java.util.TreeMap;
import java.util.stream.IntStream;

import net.joshuahughes.kalmanfilter.source.Source;
import net.joshuahughes.kalmanfilter.source.Source.Data;

public abstract class SingleTargetKinematicModel extends ArrayList<Data> implements Model,Source
{
    private static final long serialVersionUID = 4417811585270677341L;
    public static enum KinematicModel{none,position,velocity,acceleration,jerk,snap,crackle,pop}
	int spatialDimCount;
	int timeCount;
	int observationCount;
	int stateCount;
	KinematicModel estimateModel;
	KinematicModel observationModel;
	double[][] Fk;
	double[][] Hk;
    Data data0;
	double lastDt = Double.NaN;
	Random rand = new Random(38493894l);
	public SingleTargetKinematicModel(int timeCount,int spatialDimCount,KinematicModel observationModel,KinematicModel estimateModel)
	{
	    this.timeCount = timeCount;
        this.observationModel = observationModel;
        this.estimateModel = estimateModel;
		this.spatialDimCount = spatialDimCount;
        observationCount = spatialDimCount*observationModel.ordinal( );
		stateCount = spatialDimCount*estimateModel.ordinal( );
		Hk = new double[observationCount][stateCount];
		IntStream.range(0, Hk.length).forEach(i->Hk[i][i] = 1d);
	}
	protected void compute()
	{
        TreeMap<Double,double[]> truth = new TreeMap<>();
        for(int timeIndex=0;timeIndex<timeCount;timeIndex++)
        {
            double time = getTime(timeIndex);
            truth.put( time, getPosit(time) );
        }
        for(Entry<Double, double[]> e : truth.entrySet( ))
        {
            System.out.println(e.getKey( )+" *** "+Arrays.toString( e.getValue( ) ));
        }
	}
	protected double getTime(int timeIndex){return timeIndex;}
    protected abstract double[] getPosit( double time );

	@Override
	public double[][] getxk0k0() {
		return null;
	}
	@Override
	public double[][] getPk0k0() {
		return null;
	}
	@Override
	public double[][] getFk(double dt){
		if(dt != lastDt)
		{
			Fk = new double[stateCount][stateCount];
			for(int mIndex=0;mIndex<estimateModel.ordinal();mIndex++)
			{
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
	public static int factorial(int x)
	{
		if(x<2) return 1;
		return x*factorial(x-1);
	}
    @Override
    public Data getData0( )
    {
        return null;
    }
}
