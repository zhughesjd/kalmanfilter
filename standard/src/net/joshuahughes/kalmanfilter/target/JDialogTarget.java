package net.joshuahughes.kalmanfilter.target;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;

import javax.swing.ImageIcon;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.WindowConstants;

import net.joshuahughes.kalmanfilter.source.Source.Data;

public class JDialogTarget implements Target{
	Graphics2D g2d;
	JLabel lbl;
	public JDialogTarget(int xSize,int ySize)
	{
		BufferedImage image = new BufferedImage(xSize,ySize,BufferedImage.TYPE_3BYTE_BGR);
    	g2d = image.createGraphics();
    	lbl = new JLabel(new ImageIcon(image) );
    	JDialog dlg = new JDialog();
    	dlg.setSize(image.getWidth()+50, image.getHeight()+50);
    	dlg.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
    	dlg.getContentPane().add(lbl);
    	dlg.setVisible(true);

	}
	public void receive(double x,double y, Color color) {
        g2d.setColor(color);
        g2d.fillOval((int)x,(int)y, 5, 5);
        lbl.repaint();
	}
	Color[] tColors = {Color.green,Color.yellow,Color.gray,Color.gray.darker()};
	Color[] mColors = {Color.red,Color.pink,Color.red.brighter(),Color.red.darker()};
	Color[] eColors = {Color.magenta,Color.cyan,Color.blue,Color.orange};
	
	@Override
	public void receive(Data data) {
		if(data.truth!=null)
		{
			for(int index=0;index<data.truth.length/4;index++)
				receive(data.truth[4*index],data.truth[4*index+1],tColors[index%tColors.length]);
		}
		for(int index=0;index<data.measurements.length/4;index++)
			receive(data.measurements[4*index],data.measurements[4*index+1],mColors[index%mColors.length]);
	}
	@Override
	public void receive(double[][] stateEstimates, double[][] estimateCovariance) {
		for(int index=0;index<stateEstimates.length/4;index++)
			receive(stateEstimates[4*index][0],stateEstimates[4*index+1][0],eColors[index%eColors.length]);
		try {Thread.sleep(10);} catch (InterruptedException e) {e.printStackTrace();}
	}
}
