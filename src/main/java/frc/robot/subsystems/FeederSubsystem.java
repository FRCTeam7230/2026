// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
  //Kicker TODO: Maybe use a velocity controller on the kicker as well? If needed

/** 1 motor used for all rollers on bottom of chassis, meant for intaking and moving fuel to outtake(positive speed for intake, negative for outtake)*/
  private final SparkFlex rollermotor = new SparkFlex(Constants.FeederConstants.rollerCANID, MotorType.kBrushless);
/** 1 motor used for running kicker tubes, meant for moving fuel from feeder to shooter */
  private final SparkMax kickermotor1 = new SparkMax(Constants.FeederConstants.kickerCANID, MotorType.kBrushless);
  private final SparkFlexConfig m_rollermotorconfig = new SparkFlexConfig();
  private final SparkMaxConfig m_kickermotor1config = new SparkMaxConfig();

  DoublePublisher rollerCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Feeder/RollerCurrent").publish();
  DoublePublisher kickerCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Feeder/KickerCurrent").publish();
  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    m_rollermotorconfig.idleMode(IdleMode.kBrake);
    m_rollermotorconfig.smartCurrentLimit(Constants.FeederConstants.kfeedermotorlimitcurrent);
    m_rollermotorconfig.closedLoopRampRate(Constants.FeederConstants.kfeederrampratesec);
    m_kickermotor1config.idleMode(IdleMode.kBrake);
    m_kickermotor1config.smartCurrentLimit(Constants.FeederConstants.kfeedermotorlimitcurrent);
    m_kickermotor1config.closedLoopRampRate(Constants.FeederConstants.kfeederrampratesec);
    m_rollermotorconfig.closedLoop.maxMotion
    .allowedProfileError(Units.inchesToMeters(0.1));
    m_kickermotor1config.closedLoop.maxMotion
    .allowedProfileError(Units.inchesToMeters(0.1));
    rollermotor.configure(m_rollermotorconfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    kickermotor1.configure(m_kickermotor1config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }
  /** Sets roller motor speed in PERCENTAGE(positive is intake, negative is outtake)
   @param rollerspeed the speed that roller motor will be set to in PERCENTAGE
   */
  public void setRollerSpeed(double rollerspeed) {
    rollermotor.set(rollerspeed);
  }
  /** Sets kicker motor speed in PERCENTAGE 
@param kickerspeed the speed that kicker motor will be set to in PERCENTAGE
  */
  public void setKickerSpeed(double kickerspeed) {
    kickermotor1.set(kickerspeed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rollerCurrentPublisher.set(rollermotor.getOutputCurrent());
    kickerCurrentPublisher.set(kickermotor1.getOutputCurrent());
  }
}
