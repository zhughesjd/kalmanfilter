package net.joshuahughes.kalmanfilter;

import static net.joshuahughes.kalmanfilter.Utility.difference;
import static net.joshuahughes.kalmanfilter.Utility.identity;
import static net.joshuahughes.kalmanfilter.Utility.inverse;
import static net.joshuahughes.kalmanfilter.Utility.product;
import static net.joshuahughes.kalmanfilter.Utility.replace;
import static net.joshuahughes.kalmanfilter.Utility.sum;
import static net.joshuahughes.kalmanfilter.Utility.transpose;

import java.util.Random;

import net.joshuahughes.kalmanfilter.source.Simple2DKinematicSource;
import net.joshuahughes.kalmanfilter.source.Source;
import net.joshuahughes.kalmanfilter.source.Source.Data;
import net.joshuahughes.kalmanfilter.target.JDialogTarget;
import net.joshuahughes.kalmanfilter.target.Target;

public class SimpleExample
{
	static Random rand = new Random(437583478);
	public static void main(String[] args) throws Exception
	{
		int timeCount = 1000;
		boolean modelVelocityOnly = false;
		int targetCount = 16;

		Source source = new Simple2DKinematicSource(timeCount,targetCount,modelVelocityOnly);
		Target target = new JDialogTarget(timeCount, timeCount, modelVelocityOnly);

		// Using https://en.wikipedia.org/wiki/Kalman_filter#Details
		Data data0 = source.getData0();
		double[][] Pk1k1 = source.getPk0k0();
		double tk1=data0.time;
		double[][] xk1k1 = transpose(new double[][]{data0.measurements});
		target.receive(data0);
		for(Data data : source)
		{
			double tk = data.time;
			double[][] zk = transpose(new double[][]{data.measurements});
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
