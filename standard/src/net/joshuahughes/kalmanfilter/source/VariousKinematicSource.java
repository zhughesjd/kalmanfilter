package net.joshuahughes.kalmanfilter.source;

import java.util.ArrayList;
import java.util.Arrays;

public class VariousKinematicSource extends Simple2DKinematicSource 
{
	private static final long serialVersionUID = -5593617619623755656L;

	public VariousKinematicSource(int timeCount, int targetCount, int observationCount, int stateCount, int obsSwapCount)
	{
		super(timeCount, targetCount, observationCount, stateCount, obsSwapCount);
	}

	public void compute(ArrayList<ArrayList<Double>> truth, int timeIndex, int targetIndex)
	{
		double arrayPos = timeIndex;
		//Increase only y then turn at 45 degrees
		if(targetIndex==0)
		{
			if(timeIndex<timeCount/2)
				truth.get(timeIndex).addAll(Arrays.asList(500d,arrayPos,0d,1d));
			else
				truth.get(timeIndex).addAll(Arrays.asList(arrayPos,arrayPos,1d,1d));
		}
		//Increase only x then turn at 45 degrees
		if(targetIndex==1)
		{
			if(timeIndex<timeCount/2)
				truth.get(timeIndex).addAll(Arrays.asList(arrayPos,500d,1d,0d));
			else
				truth.get(timeIndex).addAll(Arrays.asList(arrayPos,timeCount-arrayPos,1d,-1d));
		}
		//Travel a distance then stop
		if(targetIndex==2)
		{
			double stoppingPosit = timeCount/3d;
			if(arrayPos<stoppingPosit)
				truth.get(timeIndex).addAll(Arrays.asList(arrayPos,arrayPos,1d,1d));
			else
				truth.get(timeIndex).addAll(Arrays.asList(stoppingPosit,stoppingPosit,0d,0d));
		}
		//Travel in a circular path
		if(targetIndex==3)
		{
			//Here observations are given only if conditions are met
			double x = Double.NaN;
			double y = Double.NaN;
			//Uncomment the next line in order to skip some observations
			//if(index<.25d*timeCount || .5d*timeCount<index)
			{
				double angle = timeIndex*(2d*Math.PI/timeCount);
				double hypot = 200d;
				x = timeCount/2d + hypot*Math.sin(angle);
				y = timeCount/2d + hypot*Math.cos(angle);
			}
			truth.get(timeIndex).addAll(Arrays.asList(x,y,0d,0d));
		}
		//Increase have only y increase
		if(targetIndex>=4)
		{
			int rest = targetCount - 4;
			double x = (targetIndex-4+1d)*(timeCount/(rest+1d));
			double y = targetIndex%2 == 1?arrayPos:timeCount - arrayPos;
			truth.get(timeIndex).addAll(Arrays.asList(x,y,0d,1d));
		}
	}
}
