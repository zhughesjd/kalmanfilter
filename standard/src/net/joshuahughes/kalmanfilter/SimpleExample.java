package net.joshuahughes.kalmanfilter;

import static net.joshuahughes.kalmanfilter.Utility.passThroughAssociator;
import static net.joshuahughes.kalmanfilter.Utility.predictAssociateUpdate;
import static net.joshuahughes.kalmanfilter.Utility.replace;

import java.util.LinkedHashMap;

import net.joshuahughes.kalmanfilter.Utility.KalmanKey;
import net.joshuahughes.kalmanfilter.associator.Associator;
import net.joshuahughes.kalmanfilter.associator.HungarianAssociator;
import net.joshuahughes.kalmanfilter.model.Model;
import net.joshuahughes.kalmanfilter.receiver.JDialogReceiver;
import net.joshuahughes.kalmanfilter.receiver.Receiver;
import net.joshuahughes.kalmanfilter.source.GridStartSource;
import net.joshuahughes.kalmanfilter.source.Source;
import net.joshuahughes.kalmanfilter.source.Source.Data;
import net.joshuahughes.kalmanfilter.source.VariousKinematicSource;

public class SimpleExample
{
    public static void main(String[] args) throws Exception
    {
        int timeCount = 1000;//can be any positive integer
        int targetCount = 8;//can be any positive integer
        int observationCount = 2;// needs to be 2 or 4
        int stateCount = 6;//needs to be 4 or 6
        int obsSwapCount = 4;//can be any non-negative number
        double defaultQk = Double.NaN;//not used yet
        double defaultRk = 20;//now being used
        boolean useGrid = true;

        Source source = useGrid?new GridStartSource(timeCount,targetCount,observationCount,stateCount,obsSwapCount,defaultQk,defaultRk):new VariousKinematicSource(timeCount,targetCount,observationCount,stateCount,obsSwapCount,defaultQk,defaultRk);
        Model model = (Model) source;

        Receiver receiver = new JDialogReceiver(timeCount, timeCount,observationCount,stateCount,targetCount);
        Associator associator = 
                obsSwapCount<=0?passThroughAssociator:
                new HungarianAssociator(observationCount,targetCount,stateCount);

        // Implementing https://en.wikipedia.org/wiki/Kalman_filter#Details
        Data data0 = source.getData0();

        double tk1=data0.time;
        double[][] xk1k1 = model.getxk0k0( );

        double[][] Pk1k1 = model.getPk0k0();
        receiver.receive(data0);
        for(Data data : source)
        {
            double tk = data.time;
            double[][] zk = data.observations;
            receiver.receive(data);

            // Get matricies at time k
            double[][] Fk = model.getFk(tk-tk1);
            double[][] Qk1 = model.getQk1(tk1);
            double[][] Hk = model.getHk(tk);
            double[][] Rk = model.getRk(tk);

            // Predict - Associate - Update
            LinkedHashMap<KalmanKey, double[][]> map = predictAssociateUpdate(xk1k1,Pk1k1,Fk,Qk1,zk,Hk,Rk,associator);
            double[][] xkk1 = map.get(KalmanKey.xkk1);
            double[][] Pkk1 = map.get(KalmanKey.Pkk1);
            double[][] xkk = map.get(KalmanKey.xkk);
            double[][] Pkk = map.get(KalmanKey.Pkk);

            // In this configuration, Double.NaN represents no data update for this iteration
            xk1k1 = replace(xkk,xkk1);
            Pk1k1 = replace(Pkk,Pkk1);

            receiver.receive(xk1k1,Pk1k1);
            tk1 = tk;
        }
    }
}
