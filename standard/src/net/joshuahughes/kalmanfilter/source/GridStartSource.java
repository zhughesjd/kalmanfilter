package net.joshuahughes.kalmanfilter.source;

import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Arrays;

public class GridStartSource extends Simple2DKinematicSource 
{
    private static final long serialVersionUID = -5593617619623755656L;

    public class Target
    {
        double t0;
        double x0;
        double y0;
        double xv;
        double yv;
        public Target( double t0, double x0, double y0, double xv, double yv )
        {
            this.t0 = t0;
            this.x0 = x0;
            this.y0 = y0;
            this.xv = xv;
            this.yv = yv;
        }
        public String toString()
        {
            return t0+"\t"+x0+"\t"+y0+"\t"+xv+"\t"+yv;
        }
    }
    ArrayList<Target> list = new ArrayList<>();
    Rectangle2D.Double extents; 
    public GridStartSource(int timeCount, int targetCount, int observationCount, int stateCount, int obsSwapCount)
    {
        super(timeCount, targetCount, observationCount, stateCount, obsSwapCount);
        extents = new Rectangle2D.Double( 0, 0, timeCount, timeCount );
        double sqrt = Math.sqrt( targetCount );
        int floor = ( int ) Math.floor( sqrt );
        int xCount = floor  + (sqrt-floor!=0?1:0);
        int yCount = xCount;
        int tgtIndex = 0;
        for(int x=0;x<xCount;x++)
        {
            for(int y=0;y<yCount;y++)
            {
                if(tgtIndex++>targetCount) break;
                double x0 = (x+1)*timeCount/(xCount+1);
                double y0 = (y+1)*timeCount/(yCount+1);
                double xv = (.5-rand.nextDouble( ))*4;
                double yv = (.5-rand.nextDouble( ))*4;
                list.add( new Target(0, x0, y0, xv, yv ) );
            }
        }

    }
    
    public void compute(ArrayList<ArrayList<Double>> truth, int timeIndex, int targetIndex)
    {
        Target tgt = list.get( targetIndex );
        double x = tgt.x0 + (timeIndex-tgt.t0)*tgt.xv;
        double y = tgt.y0 + (timeIndex-tgt.t0)*tgt.yv;
        if(!extents.contains( x, y ))
        {
            double dx = (extents.getCenterX( )- x);
            double dy = (extents.getCenterY( )- y);
            double max = Math.max( Math.abs( dx ), Math.abs( dy ) );
            dx/=max;
            dy/=max;
            tgt.t0 = timeIndex;
            tgt.x0 = x;
            tgt.xv = (.5-rand.nextDouble( ))*2*dx;
            tgt.y0 = y;
            tgt.yv = (.5-rand.nextDouble( ))*2*dy;
            x = tgt.x0 + (timeIndex-tgt.t0)*tgt.xv;
            y = tgt.y0 + (timeIndex-tgt.t0)*tgt.yv;
        }
        truth.get(timeIndex).addAll(Arrays.asList(x,y,tgt.xv,tgt.yv));
    }
}
