package frc.robot.subsystems;

import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.KeyListener;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.Color;
import java.awt.Dimension;
import java.nio.ByteBuffer;
import java.util.concurrent.ConcurrentLinkedQueue;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.AlignToHub;

import javax.swing.*;

public class LEDSimGUI extends JPanel  implements ActionListener{
     //JLabel label;
     LEDSubsystem ledSubsystem;
     Timer timer;
     public LEDSimGUI(LEDSubsystem leds){
        this.setPreferredSize(new Dimension(600,600));
        this.setBackground(Color.black);
        ledSubsystem = leds;
        ledSubsystem.updateData();
        timer = new Timer(100, this);
        timer.start();
     }
     int ledPerRow;
     int ledSize = 20;
     byte[] data = {0, 0, 0};
     //@Override
    public void paint(Graphics g){
        super.paint(g);
        Graphics2D g2D = (Graphics2D) g;
        SmartDashboard.putNumberArray("LED SUBSYSTEM GUI/Raw Data", LEDSubsystem.bytesToDouble(ledSubsystem.getDataForGUI()));
        //all this draws the leds
        if (ledSubsystem.getDataForGUI()!=null){
            int totalLEDs = ledSubsystem.getDataForGUI().length/4;
            ledPerRow = (int) Math.ceil((double) totalLEDs/Constants.LEDConstants.ledRowsInSimulation);
        for (int i = 0; i < totalLEDs; i++){
                int red = (int) ledSubsystem.getDataForGUI()[i*4+2] & 0xFF;
                int green = (int) ledSubsystem.getDataForGUI()[i*4+1] & 0xFF;
                int blue = (int) ledSubsystem.getDataForGUI()[i*4] & 0xFF;
                SmartDashboard.putNumber("LED SUBSYSTEM GUI/ledRed", red);
                SmartDashboard.putNumber("LED SUBSYSTEM GUI/ledGreen", green);
                SmartDashboard.putNumber("LED SUBSYSTEM GUI/ledBlue", blue);
                g2D.setColor(new Color(red,green,blue));
            //g2D.setColor(Color.BLUE);
            g2D.fillRect((i%ledPerRow)*(ledSize+5)+50,(i/ledPerRow)*(ledSize+5)+50,ledSize,ledSize);
            //g2D.setColor(Color.BLUE);
            //g2D.fillRect(200,200,300,300);
        }
        }
        
     }
    //  public void paintComponent(Graphics g){
    //     super.paintComponent(g);
    //     draw(g);
    // }
    @Override
	public void actionPerformed(ActionEvent e) {
        ledSubsystem.updateData();
        SmartDashboard.putNumberArray("LED SUBSYSTEM GUI/Raw Data in Periodic", LEDSubsystem.bytesToDouble(ledSubsystem.getDataForGUI()));
		repaint();//Periodically cals the paintComponent function.
        
	}
}