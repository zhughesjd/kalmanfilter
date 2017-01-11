package net.joshuahughes.kalmanfilter.target;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.util.Arrays;
import java.util.stream.Collectors;

import javax.swing.ImageIcon;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JScrollPane;
import javax.swing.WindowConstants;

import net.joshuahughes.kalmanfilter.source.Source.Data;

public class JDialogTarget extends JDialog implements Target{
    private static final long serialVersionUID = 8982418677492822173L;
    BufferedImage estimateImage;
    BufferedImage measurementImage;
    BufferedImage truthImage;
    BufferedImage combinedImage;
    JLabel lbl;
	int stateCount;
	boolean grayMeasurement = true;
	public JDialogTarget(int xSize,int ySize, boolean modelVelocityOnly)
	{
		this.stateCount = modelVelocityOnly?4:6;
		estimateImage = new BufferedImage(xSize,ySize,BufferedImage.TYPE_4BYTE_ABGR);
		measurementImage = new BufferedImage(xSize,ySize,BufferedImage.TYPE_4BYTE_ABGR);
		truthImage = new BufferedImage(xSize,ySize,BufferedImage.TYPE_4BYTE_ABGR);
		combinedImage = new BufferedImage( truthImage.getWidth( ),truthImage.getHeight(), BufferedImage.TYPE_4BYTE_ABGR);
		lbl = new JLabel(new ImageIcon( combinedImage ) );
		setSize(truthImage.getWidth()+50, truthImage.getHeight()+50);
		setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
		getContentPane().add(new JScrollPane(lbl));
		setVisible(true);
	}
	public void receive(double x,double y, Color color, Graphics2D g2d)
	{
		if(Double.isFinite(x) && Double.isFinite(y))
		{
			g2d.setColor(color);
			g2d.fillOval((int)x,(int)y, 5, 5);
		}
	}
	Color[] eColors = {Color.magenta,Color.cyan,Color.blue,Color.red,Color.green,Color.yellow,Color.orange, new Color(165,42,42)};
	Color[] mColors = grayMeasurement ? new Color[]{Color.gray}:Arrays.stream( eColors ).map(  c->new Color(c.getRed()/4,c.getGreen()/4,c.getBlue()/4) ).collect( Collectors.toList( ) ).toArray( new Color[]{} );
	@Override
	public void receive(Data data) {
		if(data.truth!=null)
		{
			for(int index=0;index<data.truth.length/stateCount;index++)
					receive(data.truth[stateCount*index],data.truth[stateCount*index+1],Color.white,truthImage.createGraphics( ));
		}
		for(int index=0;index<data.measurements.length/stateCount;index++)
		    receive(data.measurements[stateCount*index],data.measurements[stateCount*index+1],mColors[index%mColors.length],measurementImage.createGraphics( ));
	}
	@Override
	public void receive(double[][] stateEstimates, double[][] estimateCovariance) {
		for(int index=0;index<stateEstimates.length/stateCount;index++)
			receive(stateEstimates[stateCount*index][0],stateEstimates[stateCount*index+1][0],eColors[index%eColors.length],estimateImage.createGraphics( ));
		Graphics2D g2d = combinedImage.createGraphics( );
		Arrays.asList( new BufferedImage( truthImage.getWidth( ),truthImage.getHeight(), BufferedImage.TYPE_3BYTE_BGR),measurementImage,truthImage,estimateImage ).stream( ).forEach( img->g2d.drawImage( img, 0, 0, null ));
		lbl.repaint( );
		try {Thread.sleep(40);} catch (InterruptedException e) {e.printStackTrace();}
	}
}
