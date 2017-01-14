package net.joshuahughes.kalmanfilter.target;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.GridLayout;
import java.awt.image.BufferedImage;
import java.util.Arrays;
import java.util.stream.Collectors;

import javax.swing.ImageIcon;
import javax.swing.JCheckBox;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.WindowConstants;

import net.joshuahughes.kalmanfilter.source.Source.Data;

public class JDialogTarget extends JDialog implements Target{
    private static final long serialVersionUID = 8982418677492822173L;
    public static enum Type{recent_obs,all_obs,truth,estimates; JCheckBox box = new JCheckBox( this.name( ),false); BufferedImage image;};
    JLabel lbl;
    int observationCount;
    int stateCount;
    boolean grayMeasurement = true;
    BufferedImage combinedImage;
    private int targetCount;
    public JDialogTarget(int xSize,int ySize,int observationCount,int stateCount,int targetCount)
    {
    	Type.recent_obs.box.setSelected(true);
        this.targetCount = targetCount;
        this.observationCount = observationCount;
        this.stateCount = stateCount;
        combinedImage = new BufferedImage( xSize,ySize, BufferedImage.TYPE_4BYTE_ABGR);
        lbl = new JLabel(new ImageIcon( combinedImage ) );
        setSize(combinedImage.getWidth()+110, combinedImage.getHeight()+100);
        setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
        JPanel contentPane = new JPanel(new BorderLayout());
        contentPane.add(new JScrollPane(lbl),BorderLayout.CENTER);
        JPanel westPanel = new JPanel(new GridLayout(Type.values( ).length,1));
        for(Type type : Type.values( ))
        {
            type.image = new BufferedImage(xSize,ySize,BufferedImage.TYPE_4BYTE_ABGR);
            type.box.addActionListener( l->reset() );
            westPanel.add( type.box );
        }
        contentPane.add(westPanel,BorderLayout.WEST);
        setContentPane( contentPane );
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
            for(int index=0;index<targetCount;index++)
                receive(data.truth[index][0],data.truth[index+targetCount][0],Color.white,Type.truth.image.createGraphics( ));
        }
        Graphics2D g2d = Type.recent_obs.image.createGraphics( );
        g2d.setColor( new Color(0,0,0,0) );
        g2d.clearRect( 0, 0, Type.recent_obs.image.getWidth( ), Type.recent_obs.image.getHeight( ) );
        for(int index=0;index<targetCount;index++)
        {
            receive(data.observations[index][0],data.observations[index+targetCount][0],mColors[index%mColors.length],Type.all_obs.image.createGraphics( ));
            receive(data.observations[index][0],data.observations[index+targetCount][0],mColors[index%mColors.length],Type.recent_obs.image.createGraphics( ));
        }
        reset();
    }
    @Override
    public void receive(double[][] stateEstimates, double[][] estimateCovariance) {
        for(int index=0;index<targetCount;index++)
            receive(stateEstimates[index][0],stateEstimates[index+targetCount][0],eColors[index%eColors.length],Type.estimates.image.createGraphics( ));
        reset();
        try {Thread.sleep(0);} catch (InterruptedException e) {e.printStackTrace();}
    }
    public void reset()
    {
        Graphics2D g2d = combinedImage.createGraphics( );
        g2d.setColor( new Color(0,0,0,0) );
        g2d.clearRect( 0, 0, combinedImage.getWidth( ), combinedImage.getHeight( ) );
        for(Type type : Type.values( ))
            if(type.box.isSelected( ))
                combinedImage.createGraphics( ).drawImage( type.image, 0, 0, null );
        lbl.repaint( );
    }
}
