package net.joshuahughes.kalmanfilter.source;

import static net.joshuahughes.kalmanfilter.Utility.transpose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;
import java.util.stream.IntStream;

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
        for(int index=0;index<timeCount;index++)
        {
        	ArrayList<Double> list = new ArrayList<>();
        	IntStream.range(0, targetCount*stateCount).forEach(i->list.add(0d));
        	truth.add(list);
        }
        
        for(int timeIndex=0;timeIndex<timeCount;timeIndex++)
            for(int targetIndex=0;targetIndex<targetCount;targetIndex++)
            	compute(truth,timeIndex,targetIndex);
        
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
        for(int tme=0;tme<perturb.length;tme++)
            for(int ste=0;ste<stateCount;ste++)
                for(int tgt=0;tgt<targetCount;tgt++)
                	perturb[tme][ste*targetCount+tgt] = truth.get(tme).get(ste*targetCount+tgt) + processSigma[ste]*rand.nextGaussian() + obsSigma[ste]*rand.nextGaussian();
        	
        for(int time = 0;time<truth.size();time++)
        {
            for(int exIndex=0;exIndex<obsSwapCount;exIndex++)
            {
            	int t0 = rand.nextInt(targetCount);
            	int t1 = rand.nextInt(targetCount);
            	for(int s=0,p0=t0,p1=t1;s<stateCount;s++,p0+=targetCount,p1+=targetCount)
            	{
            		double tmp = perturb[time][p0];
            		perturb[time][p0] = perturb[time][p1];
            		perturb[time][p1] = tmp;
            	}
            }
        	add(new Data(
        			(double)time,
        			transpose(new double[][]{Arrays.copyOfRange(perturb[time],0,observationCount*targetCount)}),
        			transpose( new double[][]{truth.get(time).stream().mapToDouble(d->d.doubleValue()).toArray()}
        	)));
        }
        data0 = this.remove(0);
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
        return Utility.diagonal(observationCount*targetCount,defaultObservationNoise);
    }
    @Override
    public double[][] getFk(double dt) {
    	double[][] Fk = new double[stateCount*targetCount][stateCount*targetCount];
        //Position model entries
    	IntStream.range(0,observationCount).forEach(i->Fk[i][i] = 1);
        //Velocity model entries
    	int c = stateCount*targetCount/3;
    	for(int r=0;c<Fk[0].length;r++) Fk[r][c++] = dt;
        //Acceleration model entries
    	c = 2*stateCount*targetCount/3;
    	for(int r=0;c<Fk[0].length;r++) Fk[r][c++] = .5*dt*dt;
        return Fk;
    }
    @Override
    public double[][] getHk(double time) {
    	double[][] Hk = new double[observationCount*targetCount][stateCount*targetCount];
    	IntStream.range(0,observationCount).forEach(i->Hk[i][i] = 1);
        return Hk;
    }
    public Data getData0()
    {
    	return data0;
    }
    public abstract void compute(ArrayList<ArrayList<Double>> truth, int timeIndex, int targetIndex);
}
