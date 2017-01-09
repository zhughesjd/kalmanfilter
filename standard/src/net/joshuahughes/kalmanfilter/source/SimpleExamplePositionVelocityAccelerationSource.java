package net.joshuahughes.kalmanfilter.source;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import net.joshuahughes.kalmanfilter.Utility;

public class SimpleExamplePositionVelocityAccelerationSource extends ArrayList<Source.Data> implements Source 
{
	private static final long serialVersionUID = 8619045911606133484L;
	double[][] Pk0k0 = null;
	int dim;
	int stateCount;
	public SimpleExamplePositionVelocityAccelerationSource(int timeCount)
	{
		int targetCount = 8;
		stateCount = 6;
        boolean useVelocityMeasures = true;
        
		dim = targetCount*stateCount;
		
		Random rand = new Random(934757384);
		ArrayList<ArrayList<Double>> generate = new ArrayList<>();
		for(int index=0;index<timeCount;index++)generate.add(new ArrayList<>());
		
		for(int index=0;index<timeCount;index++)
		{
			double offset = 50d;
			double arrayPos = index+offset;
			if(targetCount>=1)
			{
				if(index<timeCount/2)
					generate.get(index).addAll(new ArrayList<Double>(Arrays.asList(500d,arrayPos,0d,1d,0d,0d)));
				else
					generate.get(index).addAll(new ArrayList<Double>(Arrays.asList(arrayPos-offset,arrayPos,1d,1d,0d,0d)));
			}
            if(targetCount>=2)
            {
                if(index<timeCount/2)
                    generate.get(index).addAll(new ArrayList<Double>(Arrays.asList(arrayPos,500d,1d,0d,0d,0d)));
                else
                    generate.get(index).addAll(new ArrayList<Double>(Arrays.asList(arrayPos,timeCount-arrayPos+offset,1d,-1d,0d,0d)));
            }
            if(targetCount>=3)
            {
                double stoppingPosit = timeCount/3d;
                if(arrayPos<stoppingPosit)
                    generate.get(index).addAll(new ArrayList<Double>(Arrays.asList(arrayPos,arrayPos,1d,1d,0d,0d)));
                else
                    generate.get(index).addAll(new ArrayList<Double>(Arrays.asList(stoppingPosit,stoppingPosit,0d,0d,0d,0d)));
            }
            if(targetCount>=4)
            {
                double angle = index*(2d*Math.PI/timeCount);
                double hypot = 200d;
                double x = timeCount/2d + hypot*Math.sin(angle);
                double y = timeCount/2d + hypot*Math.cos(angle);
                generate.get(index).addAll(new ArrayList<Double>(Arrays.asList(x,y,0d,0d,0d,0d)));
            }
            int rest = targetCount - generate.get( index ).size( )/stateCount;
            for(int t=0;t<rest;t++)
            {
                double x = (t+1d)*(timeCount/(rest+1d));
                generate.get(index).addAll(new ArrayList<Double>(Arrays.asList(x,arrayPos,0d,1d,0d,0d)));
            }
		}
		if(!useVelocityMeasures)
		{
			for(int t=0;t<generate.size();t++)
				for(int mIndex=2;mIndex<generate.get(t).size();mIndex+=stateCount)
				{
					generate.get(t).set(mIndex, 0d);
					generate.get(t).set(mIndex+1, 0d);
				}
		}
		double oxx = 10;
		double oxy = oxx;
        double ovx = 0;
        double ovy = 0;
        double oax = 0;
        double oay = 0;

		double[] sigmas = new double[]{oxx,oxy,ovx,ovy,oax,oay};

		double[][] perturb = new double[timeCount][stateCount*targetCount];
		for(int x=0;x<perturb.length;x++)
			for(int y=0;y<perturb[0].length;y++)
				perturb[x][y] = generate.get(x).get(y) + sigmas[y%sigmas.length]*rand.nextGaussian();
		for(int time = 0 ;time<generate.size();time++)
		{
		    System.out.println(Arrays.toString( generate.get(time).stream().mapToDouble(d->d.doubleValue()).toArray()));
			add(new Source.Data((double)time, perturb[time], generate.get(time).stream().mapToDouble(d->d.doubleValue()).toArray()));
		}
	}
	@Override
	public double[][] getPk0k0() {
		return Utility.diagonal(dim, .5);
	}
	@Override
	public double[][] getFk(double dt) {
		double[][] Fk = new double[dim][dim];
		for(int i=0;i<dim;i++)
		{
			Fk[i][i] = 1;
            if(i<dim-2 && i%stateCount<2)
            {
                Fk[i][i+2] = dt;
                Fk[i][i+4] = 2d*dt*dt;
            }
		}
		return Fk;
	}
	@Override
	public double[][] getHk(double time) {
        return Utility.identity(dim);
	}
	@Override
	public double[][] getQk1(double priorTime) {
		return Utility.diagonal(dim,.00001);
	}
	@Override
	public double[][] getRk(double time) {
		return Utility.diagonal(dim,100);
	}
	public double[][] getIdentity() {
		return Utility.identity(dim);
	}
}
