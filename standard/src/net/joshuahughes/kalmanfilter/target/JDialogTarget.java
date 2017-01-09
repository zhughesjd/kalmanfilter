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
	@Override
	public void receive(Data data) {
		if(data.truth!=null)
		{
			receive(data.truth[0],data.truth[1],Color.yellow);
			receive(data.truth[4],data.truth[5],Color.white);
		}
		receive(data.measurements[0],data.measurements[1],Color.blue);
		receive(data.measurements[4],data.measurements[5],Color.cyan);
	}
	@Override
	public void receive(double[][] stateEstimates, double[][] estimateCovariance) {
		receive(stateEstimates[0][0],stateEstimates[1][0],Color.red);
		receive(stateEstimates[4][0],stateEstimates[5][0],Color.green);
		try {Thread.sleep(10);} catch (InterruptedException e) {e.printStackTrace();}
	}
}
