// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.AlignToHub;

import javax.swing.*;


public class LEDSubsystem extends SubsystemBase {
  /** Directly controls LEDs, used to start and set data of LEDs */
  AddressableLED m_LED;
  AddressableLEDSim m_LEDSim;
  /** Used to hold state of LED strip, used as parameter to set data */
  AddressableLEDBuffer m_LEDBuffer;
  /** Able to control one part of LED strip */
  AddressableLEDBufferView m_bottom;
  /** Able to control one part of LED strip*/
  AddressableLEDBufferView m_top;
  
  JFrame frame = new JFrame();
  

  double timer = 0;
  byte[] ledData;
  AddressableLEDBufferView m_firstHalf;

  AddressableLEDBufferView m_secondHalf;
  /** Used to tell if button is pressed(true when button is pressed)*/
  boolean isOverriden;
  /** used for sinusoidal pattern because it is time based */
  Timer robotTimer; 
   /** used to keep track of function for current pattern */
  int currentState;

  BooleanPublisher isOverridenPublisher = NetworkTableInstance.getDefault().getBooleanTopic("LEDSubsystem/isOverriden").publish();
  IntegerPublisher currentStatePublisher = NetworkTableInstance.getDefault().getIntegerTopic("LEDSubsystem/CurrentState").publish();
  DoublePublisher robotTimerPublisher = NetworkTableInstance.getDefault().getDoubleTopic("LEDSubsystem/RobotTimer").publish();

  
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    m_LED = new AddressableLED(Constants.LEDConstants.kPort);
   //m_LED = new AddressableLEDSim();
    m_LEDBuffer = new AddressableLEDBuffer(Constants.LEDConstants.kLEDLength);
   // m_bottom = m_LEDBuffer.createView(Constants.LEDConstants.kBottomStartIndex, Constants.LEDConstants.kBottomEndIndex);
    //m_top = m_LEDBuffer.createView(Constants.LEDConstants.kTopStartIndex, Constants.LEDConstants.kTopEndIndex);
    
    m_bottom = m_LEDBuffer.createView(Constants.LEDConstants.kBottomStartIndex, Constants.LEDConstants.kBottomEndIndex);
    m_top = m_LEDBuffer.createView(Constants.LEDConstants.kTopStartIndex, Constants.LEDConstants.kTopEndIndex);
    m_firstHalf = m_LEDBuffer.createView(Constants.LEDConstants.kTopStartIndex, Constants.LEDConstants.kTopMiddleIndex);
    m_secondHalf = m_LEDBuffer.createView(Constants.LEDConstants.kTopMiddleIndex + 1, Constants.LEDConstants.kTopEndIndex);
    robotTimer = new Timer();
    isOverriden = false;
    currentState = 0; // 0 is idle

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

  /** used to make solid color based on input
   @param c color of the pattern
   */
  public void solidColorAll(Color c) {
    LEDPattern colorPattern = LEDPattern.solid(c);
    colorPattern.applyTo(m_LEDBuffer);
    m_LED.setData(m_LEDBuffer);
  }

  /**current state = 1, makes LEDs purple */
  public void solidPurpleAll() {
    currentState = 1;
    solidColorAll(Color.kMediumPurple);// #9370DB
  }

  /** current state = 2, makes LEDs yellow */
  public void solidYellowAll() {
    currentState = 2;
    solidColorAll(Constants.LEDConstants.kNiceYellow);// #DEC95D - contrasts medium purple
  }
  public byte[] getData(){
    return m_LEDSim.getData();
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
  /**current state = 3, makes LEDs green */
  public void solidGreenAll() {
    currentState = 3;
    solidColorAll(Constants.LEDConstants.kGreen);
  }

  /** current state = 4, used for transition between shifts
   @param hubState is the state of the hub, used to determine which colors to transition between and when to transition
     2 = active to inactive, transition from yellow to purple
     3 = inactive to active, transition from purple to yellow
   */
  public void transitionBlinkingColorSmoothAll(int hubState) { // could change later if want to use mask and make breath (brightness really fast but as a funtion of percentage)
    //is valid
    int currentPhase = hubState;
    if (!((currentPhase == 2) || (currentPhase == 3))) {
      return;
    }
    currentState = 4;
    // yes valid
    double percentage = 1 - (FieldManagementPublisher.timeLeftInTransition(currentPhase) / 3); //start transition: 0; end transition: 1
    if (percentage > 1) {
      return;
    }
    //blinking or solid yelpurp
    Color actual;
    double frequency = Constants.LEDConstants.kblinkingFrequency; //hz
    double period = 1/frequency;
    double percentagePeriod = period/3;
    double percentageOfTimeYelpurp = Constants.LEDConstants.kpercentageOfTimeYelpurp;
    if (percentage % percentagePeriod > percentagePeriod * percentageOfTimeYelpurp) {
      actual = Color.kBlack;
    }
    else {
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
      actual = Color.lerpRGB(first, second, percentage);
    }
    //set pattern
    solidColorAll(actual);
  }
  
  /** LED pattern runs during Auto(gradient, breathe, and scroll) */

  /** current state = 5, used for auto(gradient, scroll, breathe) */
  public void autoPattern() {
    currentState = 5;
    LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kMediumPurple, Constants.LEDConstants.kNiceYellow);
    LEDPattern DCbreathe = base.breathe(Units.Seconds.of(2));
    LEDPattern pattern = base.scrollAtRelativeSpeed(Units.Hertz.of(0.5));
    LEDPattern maskedthing = DCbreathe.mask(pattern);
    maskedthing.applyTo(m_LEDBuffer);
    m_LED.setData(m_LEDBuffer);
  }

  /** method to set boolean isOverriden
    @param over is the value to set isOverriden to, true when button is pressed
   */
  public void setIsOverriden(boolean over) {
    isOverriden = over;
  }

  /** gets value of boolean isOverriden */
  public boolean getIsOverriden() {
    return isOverriden;
  }

  /**current state = 6, used for shooting(yellow sinusoidal pattern) */
  public void shootingPattern() { 
    currentState = 6;
    setCustomGradientToBuffer(Constants.LEDConstants.kYellowH, Constants.LEDConstants.kYellowS, Constants.LEDConstants.kYellowV, Constants.LEDConstants.kshootingMovingFrequency);
    m_LED.setData(m_LEDBuffer);
  }

  /**current state = 6, used for shooting but mirrored on top and bottom(also yellow sinusoidal pattern) */
  public void shootingPatternMirroredPulse() { 
    currentState = 6;
    setCustomGradientToBuffer2(Constants.LEDConstants.kYellowH, Constants.LEDConstants.kYellowS, Constants.LEDConstants.kYellowV, Constants.LEDConstants.kshootingMovingFrequency);
    m_LED.setData(m_LEDBuffer);
  }

  /** manually sets gradient to entire buffer
    @param h hue of the pattern
    @param s saturation of the pattern
    @param v value of the pattern before brightness adjustment
    @param movingFrequency how many times per second the pattern should move, if applicable
   */
  public void setCustomGradientToBuffer(int h, int s, int v, double movingFrequency) {
    double time = robotTimer.get();
    for (int i=0; i < m_LEDBuffer.getLength(); i++) {
      int vAfterBrightness = (int)(v * calculateBrightnessPercentage(i, time, movingFrequency));
      Color colorAfterBrightness = Color.fromHSV(h, s, vAfterBrightness);
      m_LEDBuffer.setLED(i, colorAfterBrightness);
    }
  }
    /** manually sets gradient to entire buffer but mirrored on bottom, first top half, and second top half
     @param h hue of the pattern
     @param s saturation of the pattern
     @param v value of the pattern before brightness adjustment
     @param movingFrequency how many times per second the pattern should move, if applicable
     */
  public void setCustomGradientToBuffer2(int h, int s, int v, double movingFrequency) {
    double time = robotTimer.get();
    //under glow
    for (int i=0; i < (Constants.LEDConstants.kTopStartIndex); i++) {
      int vAfterBrightness = (int)(v * calculateBrightnessPercentage(i, time, movingFrequency));
      Color colorAfterBrightness = Color.fromHSV(h, s, vAfterBrightness);
      m_LEDBuffer.setLED(i, colorAfterBrightness);
    }
    //first half
    for (int i=Constants.LEDConstants.kTopStartIndex; i < (Constants.LEDConstants.kTopMiddleIndex+1); i++) {
      int vAfterBrightness = (int)(v * calculateBrightnessPercentage(i, time, movingFrequency));
      Color colorAfterBrightness = Color.fromHSV(h, s, vAfterBrightness);
      m_LEDBuffer.setLED(i, colorAfterBrightness);
    }
    //second half, backwards
    for (int i=(Constants.LEDConstants.kTopEndIndex); i >= Constants.LEDConstants.kTopMiddleIndex+1; i--) {
      int vAfterBrightness = (int)(v * calculateBrightnessPercentage(Constants.LEDConstants.kTopStartIndex + (Constants.LEDConstants.kTopEndIndex - i), time, movingFrequency));
      Color colorAfterBrightness = Color.fromHSV(h, s, vAfterBrightness);
      m_LEDBuffer.setLED(i, colorAfterBrightness);
    }

  }

  /** calculates brightness of purple based on LEDLength and repeatTimes along the led strip
    @param index the index of the LED being calculated, used to determine brightness based on distance from center of wave
    @param t the time since the start of the pattern, used to determine how much the pattern has moved
    @param movingFrequency how many times per second the pattern should move, used to determine how much the pattern has moved
   */
  private double calculateBrightnessPercentage(int index, double t, double movingFrequency) {
    double offset;
    if (t >= 0) {
      offset = (movingFrequency * t * Constants.LEDConstants.kLEDLength) % Constants.LEDConstants.kLEDLength;
    }
    else {
      offset = 0;
    }

    double period = Constants.LEDConstants.kLEDLength/Constants.LEDConstants.krepeatTimes;
    double a = 1/2 - Constants.LEDConstants.khalfPercentageFromBottom; 
    double b = (2*Math.PI)/period;
    double c = 1/2 + Constants.LEDConstants.khalfPercentageFromBottom;
    return a*Math.cos(b*(index - offset)) + c;
  }
  /** current state = 0, used when robot is idle(purple sinusoidal) */
  public void idlePattern() {
    currentState = 0;
    setCustomGradientToBuffer(Constants.LEDConstants.kPurpleH, Constants.LEDConstants.kPurpleS, Constants.LEDConstants.kPurpleV, Constants.LEDConstants.kidleMovingFrequency);
    m_LED.setData(m_LEDBuffer);
  }

  /** current state = 7, used for intaking(orange sinusoidal) */
  public void intakePattern() {
    currentState = 7;
    setCustomGradientToBuffer(Constants.LEDConstants.kOrangeH, Constants.LEDConstants.kOrangeS, Constants.LEDConstants.kOrangeV, Constants.LEDConstants.kintakeMovingFrequency);
    m_LED.setData(m_LEDBuffer);
  }

  //commands

  /** If the robot is aligning to hub, LEDs are green */
  public Command whileAlignSolidGreen() { //call with aligntohub alongwith i think
    return startEnd(()->{setIsOverriden(true); solidGreenAll();}, ()->{setIsOverriden(false);});
  }

  /** If the robot is shooting, LEDs are yellow sinusoidal pattern */
  public Command whileShootingPattern() { //call with shooting trigger is held alongwith i think
    return startEnd(()->{setIsOverriden(true); shootingPattern();}, ()->{setIsOverriden(false);});
  }  
  /** If the robot is intaking, LEDs are orange sinusoidal pattern */
  public Command whileIntakingPattern() { //call with shooting trigger is held alongwith i think
    return startEnd(()->{setIsOverriden(true); intakePattern();}, ()->{setIsOverriden(false);});
  }

  @Override
  public void periodic() {
    /*
   * 1 = Active
   * 0 = inactive
   * -1 = no data
   * 2 = transition (active to inactive)
   * 3 = transition (inactive to active)
   */
    
    if (DriverStation.isAutonomousEnabled() && currentState != 5) { 
      autoPattern();
    }
    // this section is the default pattern based on the match when no button is pressed
    else if (!isOverriden) { //had currentState == 0 before idk why
      int hubState = FieldManagementPublisher.getHubState();
      if (hubState == 2 || hubState == 3) {
        transitionBlinkingColorSmoothAll(hubState);
      }
      if (hubState == 1 && currentState != 2) {
        solidYellowAll();
      }
      if (hubState == 0 && currentState != 1) {
        solidPurpleAll();
      }
      if (hubState == -1) {
        idlePattern();
      }
    }
    // first priority
    if (currentState == 6) {
      shootingPattern();
    }
    // second priority
    else if (currentState == 7) {
      intakePattern();
    }
    // default pattern
    else {
      idlePattern();
    }
    
    isOverridenPublisher.set(isOverriden);
    currentStatePublisher.set(currentState);
    robotTimerPublisher.set(robotTimer.get());

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
