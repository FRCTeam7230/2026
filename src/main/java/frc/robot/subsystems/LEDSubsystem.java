// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import java.awt.Font;
import java.awt.event.KeyListener;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.awt.event.KeyEvent;
import java.nio.ByteBuffer;
import java.util.concurrent.ConcurrentLinkedQueue;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.AlignToHub;

import javax.swing.*;


public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  AddressableLED m_LED;
  AddressableLEDSim m_LEDSim;
  AddressableLEDBuffer m_LEDBuffer;
  AddressableLEDBufferView m_bottom;
  AddressableLEDBufferView m_top;
  
  JFrame frame = new JFrame();
  

  double timer = 0;
  byte[] ledData;
  public LEDSubsystem() {
    m_LED = new AddressableLED(Constants.LEDConstants.kPort);
   //m_LED = new AddressableLEDSim();
    m_LEDBuffer = new AddressableLEDBuffer(Constants.LEDConstants.kLEDLength);
   // m_bottom = m_LEDBuffer.createView(Constants.LEDConstants.kBottomStartIndex, Constants.LEDConstants.kBottomEndIndex);
    //m_top = m_LEDBuffer.createView(Constants.LEDConstants.kTopStartIndex, Constants.LEDConstants.kTopEndIndex);
    m_LEDSim = new AddressableLEDSim();
    m_LED.setLength(m_LEDBuffer.getLength());
    m_LED.setData(m_LEDBuffer);
    m_LED.start();
    
    solidPurpleAll();
    createFileForCSV();
    csvConverter();
  }
  public static double[] bytesToDouble(byte[] bytes) {
      double[] doubleArray = new double[bytes.length];
      for (int i = 0; i < bytes.length; i++){
        doubleArray[i] = (double) bytes[i];
      }
      return doubleArray;
  }

  public void solidPurpleAll() {
    LEDPattern purplePattern = LEDPattern.solid(Color.kMediumPurple);
    purplePattern.applyTo(m_LEDBuffer);
    m_LED.setData(m_LEDBuffer);
  }

  public void solidYellowAll() {
    LEDPattern yellowPattern = LEDPattern.solid(Constants.LEDConstants.kNiceYellow);// #DEC95D - contrasts medium purple
    yellowPattern.applyTo(m_LEDBuffer);
    m_LED.setData(m_LEDBuffer);
  }
  public byte[] getData(){
    return m_LEDSim==null?null:m_LEDSim.getData();
    //return ledData == null ? null : ledData.clone();

  }
  Object ledSimLock = new Object();
  public void updateData(){
    synchronized (ledSimLock){
      ledData = getData();
    }
  }
  public byte[] getDataForGUI(){
    synchronized (ledSimLock){
      if (ledData==null){
        byte[] ans = {0,0,0,0};
        return ans;
      }
      return ledData.clone();
    }
  }
  // int
  // when put in periodic might want to add a delay?

  public void transitionBlinkingColorSmoothAll() { // could change later if want to use mask and make breath (brightness really fast but as a funtion of percentage)
    //is valid
    int currentPhase = FieldManagementPublisher.getHubState();
    if (!((currentPhase == 2) || (currentPhase == 3))) {
      return;
    }
    // yes valid
    double percentage = 1 - (FieldManagementPublisher.timeLeftInTransition() / 3); //start transition: 0; end transition: 1
    if (percentage > 1) {
      return;
    }
    //select which is starting color based on type of transition
    Color first;
    Color second;
    if (currentPhase == 2) { // testing: change to 3 if backwards
      first = Constants.LEDConstants.kNiceYellow;
      second = Color.kMediumPurple;
    }
    else {
      first = Color.kMediumPurple;
      second = Constants.LEDConstants.kNiceYellow;
    }
    //blinking or solid yelpurp
    Color actual;
    if (percentage % 1/12 > 1/24) {
      actual = Color.kBlack;
    }
    else {
      actual = Color.lerpRGB(first, second, percentage);
    }
    //set pattern
    LEDPattern yelpurpPattern = LEDPattern.solid(actual);
    yelpurpPattern.applyTo(m_LEDBuffer);
    m_LED.setData(m_LEDBuffer);
  }
  
  /** LED pattern runs during Auto(gradient, breathe, and scroll) */
  public void autoPattern() {
    LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kMediumPurple, Constants.LEDConstants.kNiceYellow);
    LEDPattern DCbreathe = base.breathe(Units.Seconds.of(2));
    LEDPattern pattern = base.scrollAtRelativeSpeed(Units.Hertz.of(0.5));
    LEDPattern maskedthing = DCbreathe.mask(pattern);
    maskedthing.applyTo(m_LEDBuffer);
    m_LED.setData(m_LEDBuffer);
  }

  // @Override
  // public void periodic() {
  //   /*
  //  * 1 = Active
  //  * 0 = inactive
  //  * -1 = no data
  //  * 2 = transition (active to inactive)
  //  * 3 = transition (inactive to active)
  //  */
  //   double hubState = FieldManagementPublisher.getHubState();
  //   if (hubState == 2 || hubState == 3) {
  //     transitionBlinkingColorSmoothAll();
  //   }
  //   if (hubState == 1) {
  //     solidYellowAll();
  //   }
  //   if (hubState == 0) {
  //     solidPurpleAll();
  //   }
    
  //   if (hubState == -1) {
  //     autoPattern();
  //   }
      

  //  public void ledGUI(ConcurrentLinkedQueue<Runnable> queue){
  //       JLabel label = new JLabel("No input");
  //       label.setFont(new Font("Arial",Font.BOLD, 30));
  //       frame.setTitle("My JFrame Example");
  //       frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
  //       frame.setSize(400, 300);
  //       frame.add(label);
  //       frame.setVisible(true);
        
  //   }
    


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_LEDSim!=null){
     SmartDashboard.putNumberArray("LED Data", bytesToDouble(m_LEDSim.getData()));
    }
    if (ledData!=null){
     SmartDashboard.putNumberArray("LED SUBSYSTEM GUI/Raw Data", bytesToDouble(ledData));
    }
    // SmartDashboard.putNumberArray("LED Buffer Data", m_LEDBuffer.getRed(0));
    timer ++;
    if (timer%100<50){
      solidPurpleAll();
    } else {
      solidYellowAll();
    }
    
  }
  





  public void createFileForCSV(){
    File csvFileForLED = new File("ledData.csv");
      if (!csvFileForLED.exists()) {
        try{
        csvFileForLED.createNewFile();
          System.out.println("File created: " + csvFileForLED.getName());
        }catch(IOException e){}
      }
  }
  public void csvConverter(){
    byte[][] data = {
      {127, 0, 127},
      {127,127,0},
      {0,127,127}
    };
    try (FileWriter writer = new FileWriter("ledData.csv")) {
        BufferedWriter bwriter = new BufferedWriter(writer);
        // Loop through the array and write each element separated by a comma
        for (int i = 0; i < data.length; i++) {
            for (int j = 0; j < data[i].length; j++) {
              bwriter.append(Byte.toString(data[i][j]));
              if (j < data.length - 1) {
                  bwriter.append(",");
              }
          }
          bwriter.newLine();
        }
        bwriter.close();
    } catch (IOException e) {
        e.printStackTrace();
    }
  }
}
