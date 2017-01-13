package net.joshuahughes.kalmanfilter.source;

import static net.joshuahughes.kalmanfilter.Utility.transpose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map.Entry;
import java.util.Random;
import java.util.TreeMap;
import java.util.stream.IntStream;

import net.joshuahughes.kalmanfilter.Utility;

public abstract class Simple2DKinematicSource extends ArrayList<Source.Data> implements Source 
{
    private static final long serialVersionUID = 8619045911606133484L;

    double defaultProcessNoise = .01;
    double defaultObservationNoise = 20;

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
                    perturb[tme][ste*targetCount+tgt] = truth.get((double)tme).get(ste*targetCount+tgt) + processSigma[ste]*rand.nextGaussian() + obsSigma[ste]*rand.nextGaussian();
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
        Utility.print( this.get( 0 ).observations);
    }
    private void insertVelocityAcceleration( TreeMap<Double, ArrayList<Double>> truth )
    {
        for(Entry<Double, ArrayList<Double>> t2 : truth.entrySet( ))
        {
            Entry<Double,ArrayList<Double>> t1 = truth.lowerEntry( t2.getKey( ) );
            if(t1 == null)
            {
                t1 = t2;
                t2 = truth.higherEntry( t1.getKey( ) );
            }
            Entry<Double,ArrayList<Double>> t0 = truth.lowerEntry( t1.getKey( ) );
            if(t0 == null)
            {
                t0 = t1;
                t1 = t2;
                t2 = truth.higherEntry( t1.getKey( ) );
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

                    double x0 = t0.getValue().get( xi );
                    double x1 = t1.getValue().get( xi );
                    double x2 = t2.getValue().get( xi );
                    double y0 = t0.getValue().get( yi );
                    double y1 = t1.getValue().get( yi );
                    double y2 = t2.getValue().get( yi );
                    
                    double vx = (x2-x1)/dt;
                    double vy = (y2-y1)/dt;
                    
                    System.out.println(vx - t1.getValue( ).get( vxi ));
                    System.out.println(vy - t1.getValue( ).get( vyi ));

                    double ax = (x0 -2*x1 + x2)/(dt*dt);
                    double ay = (y0 -2*y1 + y2)/(dt*dt);
                }
            }
        }
    }
    @Override
    public double[][] getPk0k0() {
        return Utility.diagonal(targetCount*stateCount,1);
    }
    @Override
    public double[][] getQk1(double priorTime) {
        return Utility.diagonal(targetCount*stateCount,defaultProcessNoise);
    }
    @Override
    public double[][] getRk(double time) {
        return Utility.diagonal(observationCount*targetCount,defaultObservationNoise);
    }
    @Override
    public double[][] getFk(double dt) {
        //Position model entries
        double[][] Fk = Utility.identity( stateCount*targetCount );
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
        IntStream.range(0,observationCount*targetCount).forEach(i->Hk[i][i] = 1);
        return Hk;
    }
    public Data getData0()
    {
        return data0;
    }
    public abstract double[] compute(int timeIndex, int targetIndex);

}
