package net.joshuahughes.kalmanfilter.receiver;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;

import javax.swing.ImageIcon;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.WindowConstants;

public class JDialogReceiver{
	Graphics2D g2d;
	JLabel lbl;
	public JDialogReceiver(int xSize,int ySize)
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

}
