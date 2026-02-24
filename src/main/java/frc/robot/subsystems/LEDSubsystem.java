// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  AddressableLED m_LED;
  AddressableLEDBuffer m_LEDBuffer;
  AddressableLEDBufferView m_bottom;
  AddressableLEDBufferView m_top;
  boolean isOverriden;
  Timer robotTimer; 
  int currentState;

  BooleanPublisher isOverridenPublisher = NetworkTableInstance.getDefault().getBooleanTopic("LEDSubsystem/isOverriden").publish();
  IntegerPublisher currentStatePublisher = NetworkTableInstance.getDefault().getIntegerTopic("LEDSubsystem/CurrentState").publish();
  DoublePublisher robotTimerPublisher = NetworkTableInstance.getDefault().getDoubleTopic("LEDSubsystem/RobotTimer").publish();

  

  public LEDSubsystem() {
    m_LED = new AddressableLED(Constants.LEDConstants.kPort);
    m_LEDBuffer = new AddressableLEDBuffer(Constants.LEDConstants.kLEDLength);
    m_bottom = m_LEDBuffer.createView(Constants.LEDConstants.kBottomStartIndex, Constants.LEDConstants.kBottomEndIndex);
    m_top = m_LEDBuffer.createView(Constants.LEDConstants.kTopStartIndex, Constants.LEDConstants.kTopEndIndex);
    robotTimer = new Timer();
    isOverriden = false;
    currentState = 0; // 0 is idle

    m_LED.setLength(m_LEDBuffer.getLength());
    m_LED.setData(m_LEDBuffer);
    m_LED.start();
    robotTimer.start();
  }

  public void solidColorAll(Color c) {
    LEDPattern colorPattern = LEDPattern.solid(c);
    colorPattern.applyTo(m_LEDBuffer);
    m_LED.setData(m_LEDBuffer);
  }

  //current state = 1
  public void solidPurpleAll() {
    currentState = 1;
    solidColorAll(Color.kMediumPurple);// #9370DB
  }

  //current state = 2
  public void solidYellowAll() {
    currentState = 2;
    solidColorAll(Constants.LEDConstants.kNiceYellow);// #DEC95D - contrasts medium purple
  }

  //current state = 3
  public void solidGreenAll() {
    currentState = 3;
    solidColorAll(Constants.LEDConstants.kGreen);
  }

  //current state = 4
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
    if (percentage % 1/12 > 1/24) {
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

  //current state = 5
  public void autoPattern() {
    currentState = 5;
    LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kMediumPurple, Constants.LEDConstants.kNiceYellow);
    LEDPattern DCbreathe = base.breathe(Units.Seconds.of(2));
    LEDPattern pattern = base.scrollAtRelativeSpeed(Units.Hertz.of(0.5));
    LEDPattern maskedthing = DCbreathe.mask(pattern);
    maskedthing.applyTo(m_LEDBuffer);
    m_LED.setData(m_LEDBuffer);
  }

  public void setIsOverriden(boolean over) {
    isOverriden = over;
  }
  public boolean getIsOverriden() {
    return isOverriden;
  }

  //current state = 6
  public void shootingPattern() { 
    currentState = 6;
    setCustomGradientToBuffer(Constants.LEDConstants.kYellowH, Constants.LEDConstants.kYellowS, Constants.LEDConstants.kYellowV, Constants.LEDConstants.kshootingMovingFrequency);
    m_LED.setData(m_LEDBuffer);
  }

  // manually sets gradient to buffer
  public void setCustomGradientToBuffer(int h, int s, int v, double movingFrequency) {
    double time = robotTimer.get();
    for (int i=0; i < m_LEDBuffer.getLength(); i++) {
      int vAfterBrightness = (int)(v * calculateBrightnessPercentage(i, time, movingFrequency));
      Color colorAfterBrightness = Color.fromHSV(h, s, vAfterBrightness);
      m_LEDBuffer.setLED(i, colorAfterBrightness);
    }
  }

  // calculates brightness of purple based on LEDLength and repeatTimes along the led strip
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

  public void idlePattern() {
    currentState = 0;
    setCustomGradientToBuffer(Constants.LEDConstants.kPurpleH, Constants.LEDConstants.kPurpleS, Constants.LEDConstants.kPurpleV, Constants.LEDConstants.kshootingMovingFrequency);
    m_LED.setData(m_LEDBuffer);
  }

  public void intakePattern() {
    currentState = 7;
    setCustomGradientToBuffer(Constants.LEDConstants.kOrangeH, Constants.LEDConstants.kOrangeS, Constants.LEDConstants.kOrangeV, Constants.LEDConstants.kintakeMovingFrequency);
    m_LED.setData(m_LEDBuffer);
  }

  //commandss
  public Command whileAlignSolidGreen() { //call with aligntohub alongwith i think
    return startEnd(()->{setIsOverriden(true); solidGreenAll();}, ()->{setIsOverriden(false);});
  }

  public Command whileShootingPattern() { //call with shooting trigger is held alongwith i think
    return startEnd(()->{setIsOverriden(true); shootingPattern();}, ()->{setIsOverriden(false);});
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
    if (!isOverriden || currentState == 0) {
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
      if (hubState == -1 && currentState != 5) {
        autoPattern();
      }
    }
    else if (currentState == 6) {
      shootingPattern();
    }
    else if (currentState == 7) {
      intakePattern();
    }
    else {
      idlePattern();
    }
    
    isOverridenPublisher.set(isOverriden);
    currentStatePublisher.set(currentState);
    robotTimerPublisher.set(robotTimer.get());

  }
  
}
