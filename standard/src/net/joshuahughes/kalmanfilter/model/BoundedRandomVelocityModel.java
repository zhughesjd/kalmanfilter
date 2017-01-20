package net.joshuahughes.kalmanfilter.model;

import java.awt.geom.Rectangle2D;
import java.util.Arrays;
import java.util.stream.IntStream;

public class BoundedRandomVelocityModel extends SingleTargetKinematicModel
{
    private static final long serialVersionUID = 1991827019397812192L;
    double[] posit0;
    double[] velocity;
    Rectangle2D.Double bounds;
    double time0 = Double.NaN;
    private BoundedRandomVelocityModel( Rectangle2D.Double bounds,double[] posit0, int timeCount, int spatialDimCount, KinematicModel observationModel, KinematicModel estimateModel )
    {
        super( timeCount, spatialDimCount, observationModel, estimateModel );
        this.bounds = bounds;
        this.posit0 = posit0;
        this.velocity = IntStream.range( 0, posit0.length ).mapToDouble(  d->10d*rand.nextDouble( )).toArray( );
        compute();
    }

    @Override
    protected double[] getPosit( double time )
    {
        double[] posit = Arrays.copyOf( posit0, posit0.length );
        if(Double.isFinite( time0 ))
        {
            double dt = time-time0;
            posit = IntStream.range( 0, posit0.length ).mapToDouble(  d->posit0[d]+dt*velocity[d]).toArray( );
            double x = posit[0];
            double y = spatialDimCount<2?bounds.getCenterY( ):posit[1];
            if(!bounds.contains( x, y ))
            {
                posit0[0] = x;
                double dx = (bounds.getCenterX( )-x);
                double max = Math.abs( dx );
                if(spatialDimCount>1)
                {
                    posit0[1] = y;
                    double dy = (bounds.getCenterY( )-y);
                    max = Math.max( max,Math.abs( dy ));
                    velocity[1] = dy/max/dt;
                }
                velocity[0] = dx/max/dt;
            }
//            else if(rand.nextDouble( )<.1d)
//            {
//                posit0 = posit;
//                this.velocity = IntStream.range( 0, posit0.length ).mapToDouble(  d->100d*rand.nextDouble( )).toArray( );
//            }
            
        }
        else
            time0 = time;
        return posit;
    }
    public static void main(String[] args)
    {
        int timeCount = 1000;
        BoundedRandomVelocityModel model = new BoundedRandomVelocityModel( new Rectangle2D.Double(0,0,1000,1000),new double[]{500,500}, timeCount, 2, KinematicModel.position, KinematicModel.acceleration );
        IntStream.range( 0, timeCount ).forEach( t-> model.getPosit( model.getTime( t ) ));
    }

}
