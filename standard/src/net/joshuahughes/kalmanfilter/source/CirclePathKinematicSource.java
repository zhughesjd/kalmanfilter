package net.joshuahughes.kalmanfilter.source;

import java.awt.Point;
import java.util.LinkedHashMap;
/**
 * TODO Still need to work on this motion model
 * @author joshua
 *
 */
public class CirclePathKinematicSource extends Simple2DKinematicSource 
{
	private static final long serialVersionUID = -5593617619623755656L;
	LinkedHashMap<Integer,Point> idCenterMap;
	public CirclePathKinematicSource(int timeCount, int targetCount, int observationCount, int stateCount, int obsSwapCount, double defaultQk, double defaultRk)
	{
		super(timeCount, targetCount, observationCount, stateCount, obsSwapCount, defaultQk, defaultRk);
	}
	@Override
	public double[] compute( int timeIndex, int targetIndex )
	{
		if(idCenterMap == null)idCenterMap = new LinkedHashMap<>();
		Point point = idCenterMap.get(targetIndex);
		if(point == null)idCenterMap.put(targetIndex, point = new Point(rand.nextInt(timeCount), rand.nextInt(timeCount)));
		double angle = timeIndex*(2d*Math.PI/timeCount);
		double hypot = 200d;
		double x = point.x + hypot*Math.sin(angle);
		double y = point.y + hypot*Math.cos(angle);
		return new double[]{x,y,0d,0d};
	}
}
