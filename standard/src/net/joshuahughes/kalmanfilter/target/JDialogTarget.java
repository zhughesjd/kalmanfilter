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

public class JDialogTarget implements Target{
	Graphics2D g2d;
	JLabel lbl;
	int stateCount;
	public JDialogTarget(int xSize,int ySize, boolean modelVelocityOnly)
	{
		this.stateCount = modelVelocityOnly?4:6;
		BufferedImage image = new BufferedImage(xSize,ySize,BufferedImage.TYPE_3BYTE_BGR);
		g2d = image.createGraphics();
		lbl = new JLabel(new ImageIcon(image) );
		JDialog dlg = new JDialog();
		dlg.setSize(image.getWidth()+50, image.getHeight()+50);
		dlg.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
		dlg.getContentPane().add(new JScrollPane(lbl));
		dlg.setVisible(true);

	}
	public void receive(double x,double y, Color color)
	{
		if(Double.isFinite(x) && Double.isFinite(y))
		{
			g2d.setColor(color);
			g2d.fillOval((int)x,(int)y, 5, 5);
			lbl.repaint();
		}
	}
	Color[] eColors = {Color.magenta,Color.cyan,Color.blue,Color.red,Color.green,Color.yellow,Color.orange, new Color(165,42,42)};
	Color[] mColors = Arrays.stream( eColors ).map(  c->new Color(c.getRed()/4,c.getGreen()/4,c.getBlue()/4) ).collect( Collectors.toList( ) ).toArray( new Color[]{} );
	@Override
	public void receive(Data data) {
		if(data.truth!=null)
		{
			for(int index=0;index<data.truth.length/stateCount;index++)
				receive(data.truth[stateCount*index],data.truth[stateCount*index+1],Color.white);
		}
		for(int index=0;index<data.measurements.length/stateCount;index++)
			receive(data.measurements[stateCount*index],data.measurements[stateCount*index+1],mColors[index%mColors.length]);
	}
	@Override
	public void receive(double[][] stateEstimates, double[][] estimateCovariance) {
		for(int index=0;index<stateEstimates.length/stateCount;index++)
			receive(stateEstimates[stateCount*index][0],stateEstimates[stateCount*index+1][0],eColors[index%eColors.length]);
		//        try {Thread.sleep(40);} catch (InterruptedException e) {e.printStackTrace();}
	}
}
