package net.joshuahughes.kalmanfilter.source;

public class VariousKinematicSource extends Simple2DKinematicSource 
{
	private static final long serialVersionUID = -5593617619623755656L;

	public VariousKinematicSource(int timeCount, int targetCount, int observationCount, int stateCount, int obsSwapCount, double defaultQk, double defaultRk)
	{
		super(timeCount, targetCount, observationCount, stateCount, obsSwapCount, defaultQk, defaultRk);
	}
	@Override
	public double[] compute( int timeIndex, int targetIndex )
	{
		double arrayPos = timeIndex;
		//Increase only y then turn at 45 degrees
		if(targetIndex==0)
		{
			if(timeIndex<timeCount/2)
				return new double[]{500d,arrayPos,0d,1d};
			else
				return new double[]{arrayPos,arrayPos,1d,1d};
		}
		//Increase only x then turn at 45 degrees
		if(targetIndex==1)
		{
			if(timeIndex<timeCount/2)
				return new double[]{arrayPos,500d,1d,0d};
			else
				return new double[]{arrayPos,timeCount-arrayPos,1d,-1d};
		}
		//Travel a distance then stop
		if(targetIndex==2)
		{
			double stoppingPosit = timeCount/3d;
			if(arrayPos<stoppingPosit)
				return new double[]{arrayPos,arrayPos,1d,1d};
			else
				return new double[]{stoppingPosit,stoppingPosit,0d,0d};
		}
		int rest = targetCount - 4;
		double x = (targetIndex-4+1d)*(timeCount/(rest+1d));
		double y = targetIndex%2 == 1?arrayPos:timeCount - arrayPos;
		return new double[]{x,y,0d,targetIndex%2 == 0?-1:1};
	}
}
