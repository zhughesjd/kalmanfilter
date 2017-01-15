package net.joshuahughes.kalmanfilter.source;

import static net.joshuahughes.kalmanfilter.Utility.abs;
import static net.joshuahughes.kalmanfilter.Utility.diagonal;
import static net.joshuahughes.kalmanfilter.Utility.difference;
import static net.joshuahughes.kalmanfilter.Utility.identity;
import static net.joshuahughes.kalmanfilter.Utility.transpose;
import static net.joshuahughes.kalmanfilter.Utility.varianceDim2;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map.Entry;
import java.util.Random;
import java.util.TreeMap;
import java.util.stream.IntStream;

import net.joshuahughes.kalmanfilter.model.Model;

public abstract class Simple2DKinematicSource extends ArrayList<Source.Data> implements Source,Model 
{
    private static final long serialVersionUID = 8619045911606133484L;
    TreeMap<Double,double[][]> RMap = new TreeMap<>();
    TreeMap<Double,double[][]> QkMap = new TreeMap<>();
    Random rand = new Random(934757384);
    double[][] Pk0k0 = null;
    Data data0 = null;
    int timeCount;
    int targetCount;
    int observationCount;
    int stateCount;
    int obsSwapCount;
	double defaultQk;
	double defaultRk;
    public Simple2DKinematicSource(int timeCount,int targetCount,int observationCount,int stateCount,int obsSwapCount,double defaultQk,double defaultRk)
    {
    	this.defaultRk = defaultRk;
        this.timeCount = timeCount;
        this.targetCount = targetCount;
        this.observationCount = observationCount;
        this.stateCount = stateCount;
        this.obsSwapCount = obsSwapCount;
        TreeMap<Double,ArrayList<Double>> truth = new TreeMap<>();
        for(int timeIndex=0;timeIndex<timeCount;timeIndex++)
            for(int targetIndex=0;targetIndex<targetCount;targetIndex++)
            {
                ArrayList<Double> allTargetStates = truth.get( (double)timeIndex);
                if(allTargetStates == null)
                {
                    allTargetStates = new ArrayList<Double>(  );
                    for(int i=0;i<6;i++)allTargetStates.add( 0d );
                    truth.put( (double)timeIndex, allTargetStates);
                }

                double[] states = compute(timeIndex,targetIndex);
                states = Arrays.copyOf( states, stateCount );
                for(int i=0;i<stateCount*targetCount;i++)allTargetStates.add( 0d );
                for(int sIndex=0,tgtIndex=targetIndex;sIndex<states.length;sIndex++,tgtIndex+=targetCount)
                    allTargetStates.set(tgtIndex,states[sIndex]);
            }
        insertVelocityAcceleration(truth);

        double oxx = defaultRk;
        double oxy = oxx;
        double[] obsSigma = new double[]{oxx,oxy,0,0,0,0};
        obsSigma = Arrays.copyOf( obsSigma,stateCount );

        double[][] truthMatrix = new double[timeCount][stateCount*targetCount];
        double[][] perturb = new double[timeCount][stateCount*targetCount];
        for(int tme=0;tme<perturb.length;tme++)
            for(int ste=0;ste<stateCount;ste++)
                for(int tgt=0;tgt<targetCount;tgt++){
                    truthMatrix[tme][ste*targetCount+tgt] = truth.get((double)tme).get(ste*targetCount+tgt);
                    perturb[tme][ste*targetCount+tgt] = truthMatrix[tme][ste*targetCount+tgt] + obsSigma[ste]*rand.nextGaussian();
                }
        calculateQR(truthMatrix,perturb);
        for(int time = 0;time<truth.size();time++)
        {
            for(int index=0;index<obsSwapCount-1;index++)
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
                    transpose( new double[][]{truth.get((double)time).stream().mapToDouble(d->d.doubleValue()).toArray()}
                            )));
        }
        data0 = this.remove(0);
    }
	private void calculateQR(double[][] truth, double[][] observed) {
		//Calculate Q
		int spread = 5;
		int offset = spread/2;
		for(int x=0;x<truth.length;x++)
		{
			int xs = x-offset;
			int xf = x+offset;
			if(xs<0)
			{
				xs = 0;
				xf = spread-1;
			}
			if(xf>=truth.length)
			{
				xs = truth.length-spread;
				xf = truth.length-1;
			}
			double[][] sub = new double[spread][];
			for(int s=0,o=xs;o<=xf;s++,o++)	sub[s] = truth[o];
			double[] allVariance = varianceDim2(sub)[0];
			double[][] Qk = new double[allVariance.length][allVariance.length];
			for(int i=targetCount*2;i<targetCount*4;i++)
				Qk[i][i] = allVariance[i];
			QkMap.put((double)x,Qk);
		}
		//Calculate R
		double[][] magnitude = abs(difference(observed, truth));
		for(int i=0;i<magnitude.length;i++)
			magnitude[i] = Arrays.copyOfRange(magnitude[i],0,observationCount*targetCount);
		double[][] Rk = diagonal(varianceDim2(magnitude)[0]);
		for(int y=0;y<observed.length;y++)
			this.RMap.put((double)y, Rk);
	}
	private void insertVelocityAcceleration( TreeMap<Double, ArrayList<Double>> truth )
    {
        for(Entry<Double, ArrayList<Double>> t1 : truth.entrySet( ))
        {
        	Entry<Double,ArrayList<Double>> tEntry = t1;
            Entry<Double,ArrayList<Double>> t0 = truth.lowerEntry( t1.getKey( ) );
            if(t0 == null)
            {
                t0 = t1;
                t1 = truth.higherEntry( t1.getKey( ) );
            }
            Entry<Double,ArrayList<Double>> t2 = truth.higherEntry( t1.getKey( ) );
            if(t2 == null)
            {
                t2 = t1;
                t1 = t0;
                t0 = truth.lowerEntry( t1.getKey( ) );
            }
            for(int targetIndex=0;targetIndex<targetCount;targetIndex++)
            {
                for(int sIndex=0;sIndex<6+1;sIndex++)
                {
                    double dt = t2.getKey( )-t1.getKey( );
                    int tIndex = -targetCount;
                    int xi = tIndex+=targetCount;
                    int yi = tIndex+=targetCount;
                    int vxi = tIndex+=targetCount;
                    int vyi = tIndex+=targetCount;
                    int axi = tIndex+=targetCount;
                    int ayi = tIndex+=targetCount;

                    double x0 = t0.getValue().get( xi );
                    double x1 = t1.getValue().get( xi );
                    double x2 = t2.getValue().get( xi );
                    double y0 = t0.getValue().get( yi );
                    double y1 = t1.getValue().get( yi );
                    double y2 = t2.getValue().get( yi );
                    
                    tEntry.getValue().set(vxi,((x2-x1)+(x1-x0))/(2*dt));
                    tEntry.getValue().set(vyi,((y2-y1)+(y1-y0))/(2*dt));
                    tEntry.getValue().set(axi,(x0 -2*x1 + x2)/(dt*dt));
                    tEntry.getValue().set(ayi,(y0 -2*y1 + y2)/(dt*dt));
                }
            }
        }
    }
    @Override
    public double[][] getPk0k0() {
        return diagonal(targetCount*stateCount,1);
    }
    @Override
    public double[][] getQk1(double time) {
    	return QkMap.get(time);
    }
    @Override
    public double[][] getRk(double time) {
    	return RMap.get(time);
    }
    @Override
    public double[][] getFk(double dt) {
        //Position model entries
        double[][] Fk = identity( stateCount*targetCount );
        //Velocity model entries
        int c = 2*targetCount;
        for(int r=0;c<Fk[0].length;r++) Fk[r][c++] = dt;
        //Acceleration model entries
        c = 4*targetCount;
        for(int r=0;c<Fk[0].length;r++) Fk[r][c++] = .5*dt*dt;
        return Fk;
    }
    @Override
    public double[][] getHk(double time) {
        double[][] Hk = new double[observationCount*targetCount][stateCount*targetCount];
        IntStream.range(0,observationCount*targetCount).forEach(i->Hk[i][i] = 1);
        return Hk;
    }
    public Data getData0()
    {
        return data0;
    }
    public abstract double[] compute(int timeIndex, int targetIndex);

}
