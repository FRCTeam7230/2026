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
    LEDPattern purple = LEDPattern.solid(Color.kMediumPurple);
    purple.applyTo(m_LEDBuffer);
    m_LED.setData(m_LEDBuffer);
  }

  public void solidYellowAll() {
    LEDPattern purple = LEDPattern.solid(Constants.LEDConstants.kNiceYellow);// #DEC95D - co medium purple
    purple.applyTo(m_LEDBuffer);
    m_LED.setData(m_LEDBuffer);
  }

  // int
/*
  public void changeChangeColorSmooth(int currentPhase, double time) {
    if (!(currentPhase == 2)) {
      return;
    }
    Color yelpurp = Color.lerpRGB(Color.kMediumPurple, Constants.LEDConstants.kNiceYellow, time);

  }
*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
  
}
