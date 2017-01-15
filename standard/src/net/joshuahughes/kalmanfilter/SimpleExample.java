package net.joshuahughes.kalmanfilter;

import static net.joshuahughes.kalmanfilter.Utility.passThroughAssociator;
import static net.joshuahughes.kalmanfilter.Utility.predictUpdate;
import static net.joshuahughes.kalmanfilter.Utility.replace;

import net.joshuahughes.kalmanfilter.associator.Associator;
import net.joshuahughes.kalmanfilter.associator.HungarianAssociator;
import net.joshuahughes.kalmanfilter.source.Source;
import net.joshuahughes.kalmanfilter.source.Source.Data;
import net.joshuahughes.kalmanfilter.source.VariousKinematicSource;
import net.joshuahughes.kalmanfilter.target.JDialogTarget;
import net.joshuahughes.kalmanfilter.target.Target;

public class SimpleExample
{
	public static void main(String[] args) throws Exception
	{
		int timeCount = 1000;//can be any positive integer
		int targetCount = 4;//can be any positive integer
		int observationCount = 2;// needs to be 2 or 4
		int stateCount = 6;//needs to be 4 or 6
		int obsSwapCount = 0;//can be any non-negative number
		double defaultQk = Double.NaN;//not used yet
		double defaultRk = 20;//now being used

		Source source = new VariousKinematicSource(timeCount,targetCount,observationCount,stateCount,obsSwapCount,defaultQk,defaultRk);
		source.compute();
		Target target = new JDialogTarget(timeCount, timeCount,observationCount,stateCount,targetCount);
		Associator associator = obsSwapCount <= 0?passThroughAssociator:new HungarianAssociator(observationCount,stateCount);
		
		// Implementing https://en.wikipedia.org/wiki/Kalman_filter#Details
		Data data0 = source.getData0();
		double tk1=data0.time;
		double[][] xk1k1 = new double[stateCount*targetCount][1];
        for(int i=0;i<data0.observations.length;i++)xk1k1[i][0]=data0.observations[i][0];
        double[][] Pk1k1 = source.getPk0k0();
		target.receive(data0);
		for(Data data : source)
		{
			double tk = data.time;
			double[][] zk = data.observations;
			target.receive(data);

			// Get k matricies
			double[][] Fk = source.getFk(tk-tk1);
			double[][] Qk1 = source.getQk1(tk1);
			double[][] Hk = source.getHk(tk);
			double[][] Rk = source.getRk(tk);
			
			// Predict
			double[][][] update = predictUpdate(xk1k1,Pk1k1,Fk,Qk1,zk,Hk,Rk,associator);
			double[][] xkk1 = update[0];
			double[][] Pkk1 = update[1];
			double[][] xkk = update[2];
			double[][] Pkk = update[3];

			xk1k1 = replace(xkk,xkk1);
			Pk1k1 = replace(Pkk,Pkk1);
			target.receive(xk1k1,Pk1k1);
			tk1 = tk;
		}
	}
}
