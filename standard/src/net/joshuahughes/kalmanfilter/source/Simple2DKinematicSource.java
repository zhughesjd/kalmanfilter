package net.joshuahughes.kalmanfilter.source;


import static net.joshuahughes.kalmanfilter.Utility.identity;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import net.joshuahughes.kalmanfilter.Utility;

public class Simple2DKinematicSource extends ArrayList<Source.Data> implements Source 
{
    private static final long serialVersionUID = 8619045911606133484L;
    double[][] Pk0k0 = null;
    int dim;
    int stateCount;
    Data data0 = null;
    public Simple2DKinematicSource(int timeCount,int targetCount,boolean modelOnlyVelocity)
    {
        stateCount = modelOnlyVelocity?4:6;
        boolean useVelocityMeasures = true;

        dim = targetCount*stateCount;

        Random rand = new Random(934757384);
        ArrayList<ArrayList<Double>> truth = new ArrayList<>();
        for(int index=0;index<timeCount;index++)truth.add(new ArrayList<>());

        for(int index=0;index<timeCount;index++)
        {
            double offset = 0;
            double arrayPos = index+offset;
            for(int tIndex=0;tIndex<targetCount;tIndex++)
            {
            	//Increase only y then turn at 45 degrees
                if(tIndex==0)
                {
                    if(index<timeCount/2)
                        truth.get(index).addAll(Arrays.asList(500d,arrayPos,0d,1d));
                    else
                        truth.get(index).addAll(Arrays.asList(arrayPos-offset,arrayPos,1d,1d));
                }
            	//Increase only x then turn at 45 degrees
                if(tIndex==1)
                {
                    if(index<timeCount/2)
                        truth.get(index).addAll(Arrays.asList(arrayPos,500d,1d,0d));
                    else
                        truth.get(index).addAll(Arrays.asList(arrayPos,timeCount-arrayPos+offset,1d,-1d));
                }
                //Travel a distance then stop
                if(tIndex==2)
                {
                    double stoppingPosit = timeCount/3d;
                    if(arrayPos<stoppingPosit)
                        truth.get(index).addAll(Arrays.asList(arrayPos,arrayPos,1d,1d));
                    else
                        truth.get(index).addAll(Arrays.asList(stoppingPosit,stoppingPosit,0d,0d));
                }
                //Travel in a circular path
                if(tIndex==3)
                {
                	//Here observations are given only if conditions are met
                	double x = Double.NaN;
                	double y = Double.NaN;
                	//Uncomment the next line in order to skip some observations
                	//if(index<.25d*timeCount || .5d*timeCount<index)
                	{
	                    double angle = index*(2d*Math.PI/timeCount);
	                    double hypot = 200d;
	                    x = timeCount/2d + hypot*Math.sin(angle);
	                    y = timeCount/2d + hypot*Math.cos(angle);
                	}
                    truth.get(index).addAll(Arrays.asList(x,y,0d,0d));
                }
            	//Increase have only y increase
                if(tIndex>=4)
                {
                    int rest = targetCount - 4;
                    double x = (tIndex-4+1d)*(timeCount/(rest+1d));
                    double y = tIndex%2 == 1?arrayPos:timeCount - arrayPos;
                    truth.get(index).addAll(Arrays.asList(x,y,0d,1d));
                }
                if(stateCount == 6)
                    truth.get(index).addAll(Arrays.asList(0d,0d));
            }
        }
        if(!useVelocityMeasures)
        {
            for(int t=0;t<truth.size();t++)
                for(int mIndex=2;mIndex<truth.get(t).size();mIndex+=stateCount)
                {
                    truth.get(t).set(mIndex, 0d);
                    truth.get(t).set(mIndex+1, 0d);
                }
        }
        double oxx = 30;
        double oxy = oxx;
        double ovx = 0;
        double ovy = 0;
        double oax = 0;
        double oay = 0;

        double[] sigmas = new double[]{oxx,oxy,ovx,ovy,oax,oay};
        sigmas = Arrays.copyOf( sigmas,stateCount );
        double[][] perturb = new double[timeCount][stateCount*targetCount];
        for(int x=0;x<perturb.length;x++)
            for(int y=0;y<perturb[0].length;y++)
                perturb[x][y] = truth.get(x).get(y) + sigmas[y%stateCount]*rand.nextGaussian();
        for(int time = 0;time<truth.size();time++)
        {
        	Data data = new Data((double)time, perturb[time], truth.get(time).stream().mapToDouble(d->d.doubleValue()).toArray());
        	if(time == 0)
        		data0 = data;
        	else
        		add(data);
        }
    }
    @Override
    public double[][] getPk0k0() {
        return Utility.diagonal(dim, .5);
    }
    @Override
    public double[][] getFk(double dt) {
    	//Position model entries
    	double[][] Fk = identity(dim);
    	//Velocity model entries
    	for(int i=0;i<dim-2;i+=stateCount)
    	{
    		Fk[i][i+2] = Fk[i+1][i+3] = Fk[i+2][i+4] = Fk[i+3][i+5] = dt;
    	}
    	//Acceleration model entries
        if(stateCount==6)
        {
            for(int i=0;i<dim-4;i+=stateCount)
            {
            	Fk[i][i+4] = Fk[i+1][i+5] = .5*dt*dt;
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
        return Utility.diagonal(dim,.0000000001);
    }
    @Override
    public double[][] getRk(double time) {
        return Utility.diagonal(dim,10);
    }
    public double[][] getIdentity() {
        return Utility.identity(dim);
    }
    public Data getData0()
    {
    	return data0;
    }
}
