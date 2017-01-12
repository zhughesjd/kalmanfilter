package net.joshuahughes.kalmanfilter.source;

import static net.joshuahughes.kalmanfilter.Utility.transpose;
import static net.joshuahughes.kalmanfilter.Utility.identity;
import static net.joshuahughes.kalmanfilter.Utility.swap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import net.joshuahughes.kalmanfilter.Utility;

public abstract class Simple2DKinematicSource extends ArrayList<Source.Data> implements Source 
{
    private static final long serialVersionUID = 8619045911606133484L;
    Random rand = new Random(934757384);
    double[][] Pk0k0 = null;
    Data data0 = null;
    int timeCount;
    int targetCount;
    int observationCount;
    int stateCount;
    int obsSwapCount;
    public Simple2DKinematicSource(int timeCount,int targetCount,int observationCount,int stateCount,int obsSwapCount)
    {
    	this.timeCount = timeCount;
        this.targetCount = targetCount;
        this.observationCount = observationCount;
        this.stateCount = stateCount;
        this.obsSwapCount = obsSwapCount;
    }
    public void compute()
    {
        ArrayList<ArrayList<Double>> truth = new ArrayList<>();
        for(int index=0;index<timeCount;index++)truth.add(new ArrayList<>());

        for(int timeIndex=0;timeIndex<timeCount;timeIndex++)
        {
            for(int targetIndex=0;targetIndex<targetCount;targetIndex++)
            {
            	compute(truth,timeIndex,targetIndex);
            	if(stateCount == 6)
                    truth.get(timeIndex).addAll(Arrays.asList(0d,0d));
            }
        }
        if(observationCount==2)
        {
            for(int t=0;t<truth.size();t++)
                for(int mIndex=2;mIndex<truth.get(t).size();mIndex+=stateCount)
                {
                    truth.get(t).set(mIndex, 0d);
                    truth.get(t).set(mIndex+1, 0d);
                }
        }
        
        
        double pxx = defaultProcessNoise;
        double pxy = pxx;
        double pvx = 0;
        double pvy = 0;
        double pax = 0;
        double pay = 0;
        double[] processSigma = new double[]{pxx,pxy,pvx,pvy,pax,pay};
        processSigma = Arrays.copyOf( processSigma,stateCount );

        double oxx = defaultObservationNoise;
        double oxy = oxx;
        double ovx = 0;
        double ovy = 0;
        double oax = 0;
        double oay = 0;
        double[] obsSigma = new double[]{oxx,oxy,ovx,ovy,oax,oay};
        obsSigma = Arrays.copyOf( obsSigma,stateCount );

        
        double[][] perturb = new double[timeCount][stateCount*targetCount];
        for(int x=0;x<perturb.length;x++)
            for(int y=0;y<perturb[0].length;y++)
                perturb[x][y] = truth.get(x).get(y) + processSigma[y%stateCount]*rand.nextGaussian() +  + obsSigma[y%stateCount]*rand.nextGaussian();
        
        	
        for(int time = 0;time<truth.size();time++)
        {
            for(int exIndex=0;exIndex<obsSwapCount;exIndex++)
            	swap(perturb[time],rand.nextInt(targetCount)*stateCount,rand.nextInt(targetCount)*stateCount,stateCount);
        	Data data = new Data(
        			(double)time,
        			transpose(new double[][]{perturb[time]}),
        			transpose( new double[][]{truth.get(time).stream().mapToDouble(d->d.doubleValue()).toArray()}
        	));
        	if(time == 0)
        		data0 = data;
        	else
        		add(data);
        }
    }
    double defaultProcessNoise = 10d;
    double defaultObservationNoise = 10d;
    @Override
    public double[][] getPk0k0() {
        return Utility.diagonal(targetCount*stateCount,1);
    }
    @Override
    public double[][] getQk1(double priorTime) {
        return Utility.diagonal(targetCount*stateCount,defaultProcessNoise*.000000001);
    }
    @Override
    public double[][] getRk(double time) {
        return Utility.diagonal(targetCount*stateCount,defaultObservationNoise);
    }
    @Override
    public double[][] getFk(double dt) {
        //Position model entries
        double[][] Fk = identity(targetCount*stateCount);
        //Velocity model entries
        for(int i=0;i<Fk.length-2;i+=stateCount)
        {
            Fk[i][i+2] = Fk[i+1][i+3] = dt;
            if(stateCount==6) Fk[i+2][i+4] = Fk[i+3][i+5] = dt;
        }
        //Acceleration model entries
        if(stateCount==6)
        {
            for(int i=0;i<Fk.length-4;i+=stateCount)
            {
                Fk[i][i+4] = Fk[i+1][i+5] = .5*dt*dt;
            }
        }
        return Fk;
    }
    @Override
    public double[][] getHk(double time) {
        return Utility.identity(targetCount*stateCount);
    }
    public Data getData0()
    {
    	return data0;
    }
    public abstract void compute(ArrayList<ArrayList<Double>> truth, int timeIndex, int targetIndex);
}
