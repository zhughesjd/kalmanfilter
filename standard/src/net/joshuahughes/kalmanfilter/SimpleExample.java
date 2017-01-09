package net.joshuahughes.kalmanfilter;

import java.util.Random;

import net.joshuahughes.kalmanfilter.source.SimpleExamplePositionSource;
import net.joshuahughes.kalmanfilter.source.Source;
import net.joshuahughes.kalmanfilter.source.Source.Data;
import net.joshuahughes.kalmanfilter.target.JDialogTarget;
import net.joshuahughes.kalmanfilter.target.Target;

public class SimpleExample
{
	static Random rand = new Random(437583478);//bad1,bad2
	public static void main(String[] args) throws Exception
	{
		int timeCount = 1000;

		Source source = new SimpleExamplePositionSource(timeCount);
		Target target = new JDialogTarget(timeCount, timeCount);

		// Using https://en.wikipedia.org/wiki/Kalman_filter#Details
		double tk1=Double.NaN;
		double[][] xk1k1 = null;
		double[][] Pk1k1 = null;
		
		for(Data data : source)
		{
			double tk = data.time;
			double[][] zk = Utility.transpose(new double[][]{data.measurements});
			target.receive(data);
			if(Double.isNaN(tk1)) // initialize xk_0k_0 and Pk_0k_0 with first measurements
			{
				xk1k1 = zk;
				Pk1k1 = source.getPk0k0();
			}
			else //otherwise 
			{
				double[][] Fk = source.getFk(tk-tk1);
				double[][] Qk1 = source.getQk1(tk1);
				// Predict
				double[][] xkk1 = Utility.product(Fk,xk1k1);
				double[][] Pkk1 = Utility.sum(Utility.product(Utility.product(Fk,Pk1k1),Utility.transpose(Fk)),Qk1);
				//Skip update if no measurements  *** this needs to be tested ***
				if(zk.length>0)
				{
					double[][] Hk = source.getHk(tk);
					double[][] HkT = Utility.transpose(Hk);
					double[][] Rk = source.getRk(tk);
					double[][] I = source.getIdentity();
					//Update
					double[][] yk = Utility.difference(zk,Utility.product(Hk,xkk1));
					double[][] Sk = Utility.sum(Utility.product(Hk,Utility.product(Pkk1,HkT)),Rk);
					double[][] Kk = Utility.product(Pkk1,Utility.product(HkT,Utility.inverse(Sk)));
					double[][] xkk = Utility.sum(xkk1,Utility.product(Kk,yk));
					double[][] Pkk = Utility.product(Utility.difference(I,Utility.product(Kk,Hk)),Pkk1);
					xk1k1 = xkk;
					Pk1k1 = Pkk;
				}
			}
			tk1 = tk;
			target.receive(xk1k1,Pk1k1);
		}
	}
}
