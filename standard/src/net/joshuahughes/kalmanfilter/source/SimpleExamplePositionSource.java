package net.joshuahughes.kalmanfilter.source;
import java.util.ArrayList;
import java.util.Random;

import net.joshuahughes.kalmanfilter.Utility;

public class  SimpleExamplePositionSource extends ArrayList<Source.Data> implements Source 
{
	private static final long serialVersionUID = 8619045911606133484L;
	double[][] Pk0k0 = null;
	int dim;
	public SimpleExamplePositionSource(int timeCount,int targetCount,int stateCount,boolean useVelocityMeasure)
	{
		dim = targetCount*stateCount;

		Random rand = new Random(934757384);
		double[][] generate = new double[timeCount][];
		double vx00 = useVelocityMeasure?0:0;
		double vy00 = useVelocityMeasure?1:0;

		double vx01 = useVelocityMeasure?1:0;
		double vy01 = useVelocityMeasure?1:0;

		double vx10 = useVelocityMeasure?1:0;
		double vy10 = useVelocityMeasure?0:0;

		double vx11 = useVelocityMeasure?1:0;
		double vy11 = useVelocityMeasure?-1:0;

		for(int index=0;index<generate.length;index++)
		{
			int offset = 50;
			int arrayPos = index+offset;
			if(index<generate.length/2)
				generate[index] = new double[]{500,arrayPos,vx00,vy00,arrayPos,500,vx10,vy10};
			else
				generate[index] = new double[]{arrayPos-offset,arrayPos,vx01,vy01,arrayPos,generate.length-arrayPos+offset,vx11,vy11};
		}
		
		double[][] data = generate;
		double oxx = 20;
		double oxy = oxx;
		double ovx = 0;
		double ovy = 0;

		double[] sigmas = new double[]{oxx,oxy,ovx,ovy,oxx,oxy,ovx,ovy};

		double[][] perturb = new double[data.length][data[0].length];
		for(int x=0;x<perturb.length;x++)
			for(int y=0;y<perturb[0].length;y++)
				perturb[x][y] = data[x][y] + sigmas[y]*rand.nextGaussian();
		for(int time = 0 ;time<generate.length;time++)
			add(new Source.Data((double)time, perturb[time], generate[time]));
		
	}
	@Override
	public double[][] getPk0k0() {
		return Utility.diagonal(dim, .5);
	}
	@Override
	public double[][] getFk(double dt) {
		double[][] Fk = new double[dim][dim];
		for(int i=0;i<dim;i++)
		{
			Fk[i][i] = 1;
			if(i<dim-2 && i%4<2)
			{
				Fk[i][i+2] = dt;
			}
		}
		return Fk;
	}
	@Override
	public double[][] getHk(double time) {
		double[][] H = new double[dim][dim];
		for(int index=0;index<dim;index++)
			H[index][index] = 1;
		return H;
	}
	@Override
	public double[][] getQk1(double priorTime) {
		return Utility.diagonal(dim,.0001);
	}
	@Override
	public double[][] getRk(double time) {
		return Utility.diagonal(dim,50);
	}
}
