// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */
  private final SparkMax rollermotor = new SparkMax(Constants.FeederConstants.rollerCANID, MotorType.kBrushless);
  private final SparkMax kickermotor = new SparkMax(Constants.FeederConstants.kickerCANID, MotorType.kBrushless);
  private final SparkMaxConfig m_rollermotorconfig = new SparkMaxConfig();
  private final SparkMaxConfig m_kickermotorconfig = new SparkMaxConfig();

  public FeederSubsystem() {
    m_rollermotorconfig.idleMode(IdleMode.kBrake);
    m_rollermotorconfig.smartCurrentLimit(Constants.OuttakeConstants.motorlimitcurrent);
    m_rollermotorconfig.closedLoopRampRate(Constants.OuttakeConstants.krampratesec);
    m_kickermotorconfig.idleMode(IdleMode.kBrake);
    m_kickermotorconfig.smartCurrentLimit(Constants.OuttakeConstants.motorlimitcurrent);
    m_kickermotorconfig.closedLoopRampRate(Constants.OuttakeConstants.krampratesec);
    m_rollermotorconfig.closedLoop.maxMotion
    .allowedProfileError(Units.inchesToMeters(0.1));
    m_kickermotorconfig.closedLoop.maxMotion
    .allowedProfileError(Units.inchesToMeters(0.1));
    rollermotor.configure(m_rollermotorconfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    kickermotor.configure(m_kickermotorconfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setRollerSpeed(double rollerspeed) {
    rollermotor.set(rollerspeed);
  }
  public void setKickerSpeed(double kickerspeed) {
    kickermotor.set(kickerspeed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
