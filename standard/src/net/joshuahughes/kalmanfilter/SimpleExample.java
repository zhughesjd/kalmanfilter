package net.joshuahughes.kalmanfilter;

import static net.joshuahughes.kalmanfilter.Utility.difference;
import static net.joshuahughes.kalmanfilter.Utility.identity;
import static net.joshuahughes.kalmanfilter.Utility.inverse;
import static net.joshuahughes.kalmanfilter.Utility.product;
import static net.joshuahughes.kalmanfilter.Utility.replace;
import static net.joshuahughes.kalmanfilter.Utility.sum;
import static net.joshuahughes.kalmanfilter.Utility.transpose;
import net.joshuahughes.kalmanfilter.associator.Associator;
import net.joshuahughes.kalmanfilter.associator.HungarianAssociator;
import net.joshuahughes.kalmanfilter.associator.PassThroughAssociator;
import net.joshuahughes.kalmanfilter.source.GridStartSource;
import net.joshuahughes.kalmanfilter.source.Source;
import net.joshuahughes.kalmanfilter.source.Source.Data;
import net.joshuahughes.kalmanfilter.target.JDialogTarget;
import net.joshuahughes.kalmanfilter.target.Target;

public class SimpleExample
{
	public static void main(String[] args) throws Exception
	{
		int timeCount = 1000;//can be any positive integer
		int targetCount = 36;//can be any positive integer
		int observationCount = 4;// needs to be 2 or 4
		int stateCount = 6;//needs to be 4 or 6
		int obsSwapCount = 10;//can be any non-negative number

		Source source = new GridStartSource(timeCount,targetCount,observationCount,stateCount,obsSwapCount);
		source.compute();
		Target target = new JDialogTarget(timeCount, timeCount,observationCount,stateCount);
		Associator associator = obsSwapCount <= 0?new PassThroughAssociator():new HungarianAssociator(observationCount,stateCount);
		
		// Implementing https://en.wikipedia.org/wiki/Kalman_filter#Details
		Data data0 = source.getData0();
		double[][] Pk1k1 = source.getPk0k0();
		double tk1=data0.time;
		double[][] xk1k1 = data0.observations;
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
			double[][] HkT = transpose(Hk);
			double[][] Rk = source.getRk(tk);
			
			// Predict
			double[][] xkk1 = product(Fk,xk1k1);
			double[][] Pkk1 = sum(product(product(Fk,Pk1k1),transpose(Fk)),Qk1);			

			// Associate by Rearranging
			zk = associator.associate(zk,xkk1,Pkk1);
			
			// Update
			double[][] yk = difference(zk,product(Hk,xkk1));
			double[][] Sk = sum(product(Hk,product(Pkk1,HkT)),Rk);
			double[][] Kk = product(Pkk1,product(HkT,inverse(Sk)));
			double[][] xkk = sum(xkk1,product(Kk,yk));
			double[][] Pkk = product(difference(identity(Kk.length),product(Kk,Hk)),Pkk1);
			xk1k1 = replace(xkk,xkk1);
			Pk1k1 = replace(Pkk,Pkk1);
			tk1 = tk;
			target.receive(xk1k1,Pk1k1);
		}
	}
}
