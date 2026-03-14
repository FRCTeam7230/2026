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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  /** Directly controls LEDs, used to start and set data of LEDs */
  AddressableLED m_LED;
  /** Used to hold state of LED strip, used as parameter to set data */
  AddressableLEDBuffer m_LEDBuffer;
  /** Able to control one part of LED strip */
  AddressableLEDBufferView m_bottom;
  /** Able to control one part of LED strip*/
  AddressableLEDBufferView m_top;

  AddressableLEDBufferView m_firstHalf;

  AddressableLEDBufferView m_secondHalf;
  /** Used to tell if button is pressed(true when button is pressed)*/
  boolean isOverriden;
  /** used for sinusoidal pattern because it is time based */
  Timer robotTimer; 
   /** used to keep track of function for current pattern */
  int currentState;

  Color defaultHubColor;


  BooleanPublisher isOverridenPublisher = NetworkTableInstance.getDefault().getBooleanTopic("LEDSubsystem/isOverriden").publish();
  IntegerPublisher currentStatePublisher = NetworkTableInstance.getDefault().getIntegerTopic("LEDSubsystem/CurrentState").publish();
  DoublePublisher robotTimerPublisher = NetworkTableInstance.getDefault().getDoubleTopic("LEDSubsystem/RobotTimer").publish();
  BooleanPublisher timerIsRunningPublisher = NetworkTableInstance.getDefault().getBooleanTopic("LEDSubsystem/timerIsRunning").publish();
  IntegerPublisher hubStatePublisher = NetworkTableInstance.getDefault().getIntegerTopic("LEDSubsystem/HubState").publish();
  DoublePublisher matchTimePublisher = NetworkTableInstance.getDefault().getDoubleTopic("LEDSubsystem/matchTim").publish();
  
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    m_LED = new AddressableLED(Constants.LEDConstants.kPort);
    m_LEDBuffer = new AddressableLEDBuffer(Constants.LEDConstants.kLEDLength);
    m_bottom = m_LEDBuffer.createView(Constants.LEDConstants.kBottomStartIndex, Constants.LEDConstants.kBottomEndIndex);
    m_top = m_LEDBuffer.createView(Constants.LEDConstants.kTopStartIndex, Constants.LEDConstants.kTopEndIndex);
    robotTimer = new Timer();
    robotTimer.start();
    isOverriden = false;
    currentState = 0; // 0 is idle
    defaultHubColor = Constants.LEDConstants.kPurple;

    m_LED.setLength(m_LEDBuffer.getLength());
    m_LED.setData(m_LEDBuffer);
    m_LED.start();
    robotTimer.start();
  }

  /** used to make solid color based on input
   @param c color of the pattern
   */
  public void solidColorTop(Color c) {
    LEDPattern colorPattern = LEDPattern.solid(c);
    colorPattern.applyTo(m_top);
    m_LED.setData(m_LEDBuffer);
  }

  public void solidColorBottom(Color c) {
    LEDPattern colorPattern = LEDPattern.solid(c);
    colorPattern.applyTo(m_bottom);
    m_LED.setData(m_LEDBuffer);
  }

  /**current state = 1, makes LEDs purple */
  public void solidPurpleTop() {
    currentState = 1;
    solidColorTop(Color.kMediumPurple);// #9370DB
    defaultHubColor = (Color.kMediumPurple);
  }


  /** current state = 2, makes LEDs yellow */
  public void solidYellowTop() {
    currentState = 2;
    solidColorTop(Constants.LEDConstants.kNiceYellow);// #DEC95D - contrasts medium purple
    defaultHubColor = Constants.LEDConstants.kNiceYellow;
  }


  /** current state = 4, used for transition between shifts
   @param hubState is the state of the hub, used to determine which colors to transition between and when to transition
     2 = active to inactive, transition from yellow to purple
     3 = inactive to active, transition from purple to yellow
   */
    // int
  // when put in periodic might want to add a delay?
  /**current state = 3, makes LEDs green */
  public void solidGreenTop() {
    currentState = 3;
    solidColorTop(Constants.LEDConstants.kGreen);
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
      if (currentPhase == 3) { // testing: change to 2 if backwards
        first = Constants.LEDConstants.kNiceYellow;
        second = Color.kMediumPurple;
      }
      else {
        first = Color.kMediumPurple;
        second = Constants.LEDConstants.kNiceYellow;
      }
      actual = Color.lerpRGB(first, second, percentage);
      defaultHubColor = actual;
    }
    //set pattern
    solidColorTop(actual);
  }
  
  /** LED pattern runs during Auto(gradient, breathe, and scroll) */

  /** current state = 5, used for auto(gradient, scroll, breathe) */
  public void autoPattern() {
    currentState = 5;
    LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kMediumPurple, Constants.LEDConstants.kNiceYellow);
    LEDPattern DCbreathe = base.breathe(Units.Seconds.of(2));
    LEDPattern pattern = DCbreathe.scrollAtRelativeSpeed(Units.Hertz.of(0.5));
    pattern.applyTo(m_LEDBuffer); //applies to both top and bottom
    m_LED.setData(m_LEDBuffer);
  }

  /** cool pattern but not used currently for match */
  public void crazyPattern() {
    currentState = 5;
    LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kMediumPurple, Constants.LEDConstants.kNiceYellow);
    LEDPattern DCbreathe = base.breathe(Units.Seconds.of(2));
    LEDPattern pattern = base.scrollAtRelativeSpeed(Units.Hertz.of(0.5));
    LEDPattern maskedthing = DCbreathe.mask(pattern);
    maskedthing.applyTo(m_top);
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
    setCustomGradientToBuffer(Constants.LEDConstants.kYellowH, Constants.LEDConstants.kYellowS, Constants.LEDConstants.kYellowV, Constants.LEDConstants.kshootingMovingFrequency, Constants.LEDConstants.kshootingRepeatTimes);
    m_LED.setData(m_LEDBuffer);
  }

  /**current state = 6, used for shooting but mirrored on top and bottom(also yellow sinusoidal pattern) */
  public void shootingPatternMirroredPulse() { 
    currentState = 6;
    setCustomGradientToBuffer2(Constants.LEDConstants.kYellowH, Constants.LEDConstants.kYellowS, Constants.LEDConstants.kYellowV, Constants.LEDConstants.kshootingMovingFrequency, Constants.LEDConstants.kshootingRepeatTimes);
    m_LED.setData(m_LEDBuffer);
  }

  /**current state = 8, used for passing(purple sinusoidal pattern) */
  public void passingPattern(int hubState) {
    currentState = 8;
    int h, s, v;
    if (hubState == 2 || hubState == 3 || hubState == 1) {
      h = Constants.LEDConstants.kYellowH;
      s = Constants.LEDConstants.kYellowS;
      v = Constants.LEDConstants.kYellowV;
    }
    else {
      h = Constants.LEDConstants.kPurpleH;
      s = Constants.LEDConstants.kPurpleS;
      v = Constants.LEDConstants.kPurpleV;
    }
    setCustomGradientToBuffer(h, s, v, Constants.LEDConstants.kshootingMovingFrequency, Constants.LEDConstants.kshootingRepeatTimes);
    m_LED.setData(m_LEDBuffer);
  }


  /** current state = 8, used for passing but mirrored on top and bottom(also blue sinusoidal pattern) */
  public void passingPatternMirroredPulse(int hubState) {
    currentState = 8;
    int h, s, v;
    if (hubState == 2 || hubState == 3 || hubState == 1) {
      h = Constants.LEDConstants.kYellowH;
      s = Constants.LEDConstants.kYellowS;
      v = Constants.LEDConstants.kYellowV;
    }
    else {
      h = Constants.LEDConstants.kPurpleH;
      s = Constants.LEDConstants.kPurpleS;
      v = Constants.LEDConstants.kPurpleV;
    }
    setCustomGradientToBuffer2(h, s, v, Constants.LEDConstants.kshootingMovingFrequency, Constants.LEDConstants.kshootingRepeatTimes);
    m_LED.setData(m_LEDBuffer);
  }

  /** manually sets gradient to entire buffer
    @param h hue of the pattern
    @param s saturation of the pattern
    @param v value of the pattern before brightness adjustment
    @param movingFrequency how many times per second the pattern should move, if applicable
   */
  public void setCustomGradientToBuffer(int h, int s, int v, double movingFrequency, double repeatTimes) {
    double time = robotTimer.get();
    for (int i=0; i < m_LEDBuffer.getLength(); i++) {
      int vAfterBrightness = (int)(v * calculateBrightnessPercentage(i, time, movingFrequency, repeatTimes));
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
  public void setCustomGradientToBuffer2(int h, int s, int v, double movingFrequency, int repeatTimes) {
    double time = robotTimer.get();
    //under glow
    /* 
    for (int i=0; i < (Constants.LEDConstants.kTopStartIndex); i++) {
      int vAfterBrightness = (int)(v * calculateBrightnessPercentage(i, time, movingFrequency, repeatTimes));
      Color colorAfterBrightness = Color.fromHSV(h, s, vAfterBrightness);
      m_LEDBuffer.setLED(i, colorAfterBrightness);
    }
    */
    //first half
    for (int i=Constants.LEDConstants.kTopStartIndex; i < (Constants.LEDConstants.kTopMiddleIndex+1); i++) {
      int vAfterBrightness = (int)(v * calculateBrightnessPercentage(i, time, movingFrequency, repeatTimes));
      Color colorAfterBrightness = Color.fromHSV(h, s, vAfterBrightness);
      m_LEDBuffer.setLED(i, colorAfterBrightness);
    }
    //second half, backwards
    for (int i=Constants.LEDConstants.kTopEndIndex; i > (Constants.LEDConstants.kTopMiddleIndex); i--) {
      int vAfterBrightness = (int)(v * calculateBrightnessPercentage((Constants.LEDConstants.kTopStartIndex + (Constants.LEDConstants.kTopEndIndex-i)), time, movingFrequency, repeatTimes));
      Color colorAfterBrightness = Color.fromHSV(h, s, vAfterBrightness);
      m_LEDBuffer.setLED(i, colorAfterBrightness);
    }

  }

  /** calculates brightness of purple based on LEDLength and repeatTimes along the led strip
    @param index the index of the LED being calculated, used to determine brightness based on distance from center of wave
    @param t the time since the start of the pattern, used to determine how much the pattern has moved
    @param movingFrequency how many times per second the pattern should move, used to determine how much the pattern has moved
   */
  private double calculateBrightnessPercentage(int index, double t, double movingFrequency, double repeatTimes) {
    double offset;
    if (t >= 0) {
      offset = (movingFrequency * t * Constants.LEDConstants.kLEDLength) % Constants.LEDConstants.kLEDLength;
    }
    else {
      offset = 0;
    }

    double period = Constants.LEDConstants.kLEDLength/repeatTimes;
    double a = 0.5 - Constants.LEDConstants.khalfPercentageFromBottom; 
    double b = (2*Math.PI)/period;
    double c = 0.5 + Constants.LEDConstants.khalfPercentageFromBottom;
    return a*Math.cos(b*(index - offset)) + c;
  }

  private double calculateBrightnessPercentage2(double matchTime) {
    if (matchTime <= 0) {
      return 1;
    }
    double b = Constants.LEDConstants.kTenSecondsLeftWarpConstant;
    int k = Constants.LEDConstants.kTenSecondsLeftMaximumIndex;
    double a = 10-(2/(b*(1+4*k)));
    if (a >=0 ) {
      return 1;
    }
    return (0.5 - Constants.LEDConstants.kTenSecondsLeftHalfPercentageFromBottom) * Math.sin((Math.PI)/(b*(matchTime-a))) + (0.5 + Constants.LEDConstants.kTenSecondsLeftHalfPercentageFromBottom);
  }


  double tenSecondsLeft = 1;
  /** used for last ten seconds of match, blinks faster and faster until match ends */
    public void tenSecondsLeft() {
    currentState = 9;
    tenSecondsLeft -= 0.0015;
    LEDPattern blinkbase = LEDPattern.solid(Color.kWhite);
    LEDPattern blinkpattern = blinkbase.blink(Units.Seconds.of(tenSecondsLeft));
    blinkpattern.applyTo(m_top);
    m_LED.setData(m_LEDBuffer);
  }

  public void tenSecondsLeft2() {
    currentState = 9;
    double matchTime = DriverStation.getMatchTime();
    int newV = (int)((Constants.LEDConstants.kYellowV) * (calculateBrightnessPercentage2(matchTime)));
    solidColorTop(Color.fromHSV(Constants.LEDConstants.kYellowH, Constants.LEDConstants.kYellowS, newV));
  } 

  public void tenSecondsLeft3() {
    currentState = 9;
    double matchTime = DriverStation.getMatchTime();
    double percentage = calculateBrightnessPercentage2(matchTime);
    // int newS = manuallyLinearlyInterpolate(Constants.LEDConstants.kYellowS, Constants.LEDConstants.kPurpleS, percentage, false);
    // int newV = manuallyLinearlyInterpolate(Constants.LEDConstants.kYellowV, Constants.LEDConstants.kPurpleV, percentage, false);
    // int newH = manuallyLinearlyInterpolate(Constants.LEDConstants.kYellowH, Constants.LEDConstants.kPurpleH, percentage, true);
    solidColorTop(Color.lerpRGB(Constants.LEDConstants.kPurple, Constants.LEDConstants.kNiceYellow, percentage));
  }

  private int manuallyLinearlyInterpolate(int val1, int val2, double t, boolean isHue) {
    if (isHue) {
      int deltaH = val2 - val1;
      int hue = (int)(val1 + (deltaH * t));
      return (hue + 180) % 180;
    }
    else {
      return (int)(val1 + (t * (val2 - val1)));
    }
    
  }

  /** current state = 0, used when robot is idle(purple sinusoidal) */
  public void idlePattern() { // could change later if want to use mask and make breath (brightness really fast but as a funtion of percentage)
    // If the Driver Station is not attached, show the red/blue scrolling gradient
    if (!DriverStation.isDSAttached() || DriverStation.isAutonomous() || DriverStation.isTest() || DriverStation.isDisabled()) {
      LEDPattern basegradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Constants.LEDConstants.kRed, Constants.LEDConstants.kBlue);
      LEDPattern scrollpattern = basegradient.scrollAtRelativeSpeed(Units.Hertz.of(Constants.LEDConstants.kidleScrollingMovingFrequency));
      scrollpattern.applyTo(m_LEDBuffer); //applies to both top and bottom
      m_LED.setData(m_LEDBuffer);
      return;
    }

    // Use the optional alliance value when DS is attached
    var allianceOpt = DriverStation.getAlliance();
    if (allianceOpt.isPresent()) {
      var alliance = allianceOpt.get();
      if (alliance == DriverStation.Alliance.Red) {
        setCustomGradientToBuffer(Constants.LEDConstants.kRedH, Constants.LEDConstants.kRedS, Constants.LEDConstants.kRedV, Constants.LEDConstants.kidleSinusoidalMovingFrequency, Constants.LEDConstants.kidleRepeatTimes);
        m_LED.setData(m_LEDBuffer);
        return;
      } else if (alliance == DriverStation.Alliance.Blue) {
        setCustomGradientToBuffer(Constants.LEDConstants.kBlueH, Constants.LEDConstants.kBlueS, Constants.LEDConstants.kBlueV, Constants.LEDConstants.kidleSinusoidalMovingFrequency, Constants.LEDConstants.kidleRepeatTimes);
        m_LED.setData(m_LEDBuffer);
        return;
      }
    } 

    // Fallback: if we don't have a known alliance, show the red/blue scrolling gradient
    LEDPattern basegradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Constants.LEDConstants.kRed, Constants.LEDConstants.kBlue);
    LEDPattern scrollpattern = basegradient.scrollAtRelativeSpeed(Units.Hertz.of(Constants.LEDConstants.kidleScrollingMovingFrequency));
    scrollpattern.applyTo(m_top);
    m_LED.setData(m_LEDBuffer);
  }

  public void idlePatternMirroredPulse() {
    currentState = 0;
    setCustomGradientToBuffer2(Constants.LEDConstants.kBlueH, Constants.LEDConstants.kBlueS, Constants.LEDConstants.kBlueV, Constants.LEDConstants.kidleSinusoidalMovingFrequency, Constants.LEDConstants.kidleRepeatTimes);
    m_LED.setData(m_LEDBuffer);
  }

  /** current state = 7, used for intaking(orange sinusoidal) */
  public void intakePattern() {
    currentState = 7;
    setCustomGradientToBuffer(Constants.LEDConstants.kOrangeH, Constants.LEDConstants.kOrangeS, Constants.LEDConstants.kOrangeV, Constants.LEDConstants.kintakeMovingFrequency, Constants.LEDConstants.kintakingtRepeatTimes);
    m_LED.setData(m_LEDBuffer);
  }

  public void intakePatternMirroredPulse() {
    currentState = 7;
    setCustomGradientToBuffer2(Constants.LEDConstants.kOrangeH, Constants.LEDConstants.kOrangeS, Constants.LEDConstants.kOrangeV, Constants.LEDConstants.kintakeMovingFrequency, Constants.LEDConstants.kintakingtRepeatTimes);
    m_LED.setData(m_LEDBuffer);
  }

  //commands

  /** If the robot is aligning to hub, LEDs are green */
  public Command whileAlignSolidGreen() { //call with aligntohub alongwith i think
    return startEnd(()->{setIsOverriden(true); solidGreenTop();}, ()->{setIsOverriden(false);});
  }

  /** If the robot is shooting, LEDs are yellow sinusoidal pattern */
  public Command whileShootingPattern() { //call with shooting trigger is held alongwith i think
    return startEnd(()->{setIsOverriden(true); shootingPattern();}, ()->{setIsOverriden(false);}); //THIS RUNS SHOOTINGPATTERN ONCE; NEED TO CHAGNE SO IT RUNS EVERY PERIODIC. MAKE IT CHANGE CURRENTPHASE INSTEAD OF CALLING PATTERN
  }  
  /** If the robot is intaking, LEDs are orange sinusoidal pattern */
  public Command whileIntakingPattern() { //call with shooting trigger is held alongwith i think
    return startEnd(()->{setIsOverriden(true); intakePattern();}, ()->{setIsOverriden(false);}); //THIS RUNS SHOOTINGPATTERN ONCE; NEED TO CHAGNE SO IT RUNS EVERY PERIODIC. MAKE IT CHANGE CURRENTPHASE INSTEAD OF CALLING PATTERN
  }

  boolean runOnce = true;

  @Override
  public void periodic() {
  
    //shootingPattern();
    //shootingPatternMirroredPulse();
    //idlePattern();
    //idlePatternMirroredPulse();
    //intakePattern();
    //intakePatternMirroredPulse();
    //crazyPattern();
    //passingPattern(FieldManagementPublisher.getHubState());
    //passingPatternMirroredPulse();
    //autoPattern();
    //tenSecondsLeft2();

    
    

      
    
    /*
   * 1 = Active
   * 0 = inactive
   * -1 = no data
   * 2 = transition (active to inactive)
   * 3 = transition (inactive to active)
   */
    
   int hubState = FieldManagementPublisher.getHubState();
    if (DriverStation.isAutonomousEnabled()){// && currentState != 5) { 
      autoPattern();
    }

    ///*
    // this section is the default pattern based on the match when no button is pressed
    
    else if (FieldManagementPublisher.isLastTenSeconds() && hubState != -1) {
      tenSecondsLeft2();
    }
    else if (!isOverriden) { //had currentState == 0 before idk why
      hubStatePublisher.set(hubState);
      matchTimePublisher.set(DriverStation.getMatchTime());
      if (hubState == 2 || hubState == 3) {
        transitionBlinkingColorSmoothAll(hubState);
        solidColorBottom(defaultHubColor);
      }
      if (hubState == 1){// && currentState != 2) {
        solidYellowTop();
        solidColorBottom(defaultHubColor);
      }
      if (hubState == 0){// && currentState != 1) {
        solidPurpleTop();
        solidColorBottom(defaultHubColor);
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
    else if (currentState == 8) {
      passingPattern(hubState);
    }
    //*/
    
    

    isOverridenPublisher.set(isOverriden);
    currentStatePublisher.set(currentState);
    robotTimerPublisher.set(robotTimer.get());
    timerIsRunningPublisher.set(robotTimer.isRunning());

  }
}
