// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  AddressableLED m_LED;
  AddressableLEDBuffer m_LEDBuffer;
  AddressableLEDBufferView m_bottom;
  AddressableLEDBufferView m_top;

  public LEDSubsystem() {
    m_LED = new AddressableLED(Constants.LEDConstants.kPort);
    m_LEDBuffer = new AddressableLEDBuffer(Constants.LEDConstants.kLEDLength);
    m_bottom = m_LEDBuffer.createView(Constants.LEDConstants.kBottomStartIndex, Constants.LEDConstants.kBottomEndIndex);
    m_top = m_LEDBuffer.createView(Constants.LEDConstants.kTopStartIndex, Constants.LEDConstants.kTopEndIndex);

    m_LED.setLength(m_LEDBuffer.getLength());
    m_LED.setData(m_LEDBuffer);
    m_LED.start();
  }

  public void solidPurpleAll() {
    LEDPattern purplePattern = LEDPattern.solid(Color.kMediumPurple);
    purplePattern .applyTo(m_LEDBuffer);
    m_LED.setData(m_LEDBuffer);
  }

  public void solidYellowAll() {
    LEDPattern yellowPattern = LEDPattern.solid(Constants.LEDConstants.kNiceYellow);// #DEC95D - contrasts medium purple
    yellowPattern.applyTo(m_LEDBuffer);
    m_LED.setData(m_LEDBuffer);
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
  

  @Override
  public void periodic() {
    /*
   * 1 = Active
   * 0 = inactive
   * -1 = no data
   * 2 = transition (active to inactive)
   * 3 = transition (inactive to active)
   */
    double hubState = FieldManagementPublisher.getHubState();
    if (hubState == 2 || hubState == 3) {
      transitionBlinkingColorSmoothAll();
    }
    if (hubState == 1) {
      solidYellowAll();
    }
    if (hubState == 0) {
      solidPurpleAll();
    }
    /* 
    if (hubState == -1) {
      autoPattern();
    }
      */
  }
  
}
