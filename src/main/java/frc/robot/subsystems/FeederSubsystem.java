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
  private final SparkMax kickermotor1 = new SparkMax(Constants.FeederConstants.kickerCANID, MotorType.kBrushless);
  private final SparkMax kickermotor2 = new SparkMax(Constants.FeederConstants.kickerCANID, MotorType.kBrushless);
  private final SparkMaxConfig m_rollermotorconfig = new SparkMaxConfig();
  private final SparkMaxConfig m_kickermotor1config = new SparkMaxConfig();
  private final SparkMaxConfig m_kickermotor2config = new SparkMaxConfig();

  public FeederSubsystem() {
    m_rollermotorconfig.idleMode(IdleMode.kBrake);
    m_rollermotorconfig.smartCurrentLimit(Constants.FeederConstants.kfeedermotorlimitcurrent);
    m_rollermotorconfig.closedLoopRampRate(Constants.FeederConstants.kfeederrampratesec);
    m_kickermotor1config.idleMode(IdleMode.kBrake);
    m_kickermotor1config.smartCurrentLimit(Constants.FeederConstants.kfeedermotorlimitcurrent);
    m_kickermotor1config.closedLoopRampRate(Constants.FeederConstants.kfeederrampratesec);
    m_kickermotor2config.idleMode(IdleMode.kBrake);
    m_kickermotor2config.smartCurrentLimit(Constants.FeederConstants.kfeedermotorlimitcurrent);
    m_kickermotor2config.closedLoopRampRate(Constants.FeederConstants.kfeederrampratesec);
    m_rollermotorconfig.closedLoop.maxMotion
    .allowedProfileError(Units.inchesToMeters(0.1));
    m_kickermotor1config.closedLoop.maxMotion
    .allowedProfileError(Units.inchesToMeters(0.1));
    m_kickermotor2config.closedLoop.maxMotion
    .allowedProfileError(Units.inchesToMeters(0.1));
    rollermotor.configure(m_rollermotorconfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    kickermotor1.configure(m_kickermotor1config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    kickermotor2.configure(m_kickermotor2config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setRollerSpeed(double rollerspeed) {
    rollermotor.set(rollerspeed);
  }
  public void setKickerSpeed(double kickerspeed1, double kickerspeed2) {
    kickermotor1.set(kickerspeed1);
    kickermotor2.set(kickerspeed2);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
