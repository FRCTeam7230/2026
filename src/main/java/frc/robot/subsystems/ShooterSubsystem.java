// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OuttakeConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystemcurrently. */
  private final SparkMax outtakemotor1 = new SparkMax(0, MotorType.kBrushless);
  private final SparkMax outtakemotor2 = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax outtakemotor3 = new SparkMax(2, MotorType.kBrushless);
  private final AbsoluteEncoder m_outtakemotorencoder1 = outtakemotor1.getAbsoluteEncoder();
  private final AbsoluteEncoder m_outtakemotorencoder2 = outtakemotor2.getAbsoluteEncoder();
  private final AbsoluteEncoder m_outtakemotorencoder3 = outtakemotor3.getAbsoluteEncoder();
  private final SparkMaxConfig m_outtakemotor1config = new SparkMaxConfig();
  private final SparkMaxConfig m_outtakemotor2config = new SparkMaxConfig();
  private final SparkMaxConfig m_outtakemotor3config = new SparkMaxConfig();
  private final SparkClosedLoopController m_outtakecontroller1 = outtakemotor1.getClosedLoopController();
  private final SparkClosedLoopController m_outtakecontroller2 = outtakemotor2.getClosedLoopController();
  private final SparkClosedLoopController m_outtakecontroller3 = outtakemotor3.getClosedLoopController();

  public ShooterSubsystem() {
    m_outtakemotor1config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(OuttakeConstants.kOuttakeKp, OuttakeConstants.kOuttakeKi, OuttakeConstants.kOuttakeKd)
    .outputRange(-0.8,0.8, ClosedLoopSlot.kSlot0);
    m_outtakemotor1config.idleMode(IdleMode.kCoast);
    m_outtakemotor1config.smartCurrentLimit(Constants.OuttakeConstants.motorlimitcurrent);
    m_outtakemotor1config.closedLoopRampRate(Constants.OuttakeConstants.krampratesec);
    m_outtakemotor2config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(OuttakeConstants.kOuttakeKp, OuttakeConstants.kOuttakeKi, OuttakeConstants.kOuttakeKd)
    .outputRange(-0.8,0.8, ClosedLoopSlot.kSlot0);
    m_outtakemotor2config.idleMode(IdleMode.kCoast);
    m_outtakemotor2config.smartCurrentLimit(Constants.OuttakeConstants.motorlimitcurrent);
    m_outtakemotor2config.closedLoopRampRate(Constants.OuttakeConstants.krampratesec);
    m_outtakemotor3config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(OuttakeConstants.kOuttakeKp, OuttakeConstants.kOuttakeKi, OuttakeConstants.kOuttakeKd)
    .outputRange(-0.8,0.8, ClosedLoopSlot.kSlot0);
    m_outtakemotor3config.closedLoop.maxMotion
    .allowedProfileError(Units.inchesToMeters(0.1));
    m_outtakemotor3config.idleMode(IdleMode.kCoast);
    m_outtakemotor3config.smartCurrentLimit(Constants.OuttakeConstants.motorlimitcurrent);
    m_outtakemotor3config.closedLoopRampRate(Constants.OuttakeConstants.krampratesec);
    outtakemotor1.configure(m_outtakemotor1config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    outtakemotor2.configure(m_outtakemotor2config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    outtakemotor3.configure(m_outtakemotor3config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }





  public void reachSpeed(double Velocity) {
    m_outtakecontroller1.setSetpoint(Velocity, ControlType.kVelocity,ClosedLoopSlot.kSlot0);
    m_outtakecontroller2.setSetpoint(Velocity, ControlType.kVelocity,ClosedLoopSlot.kSlot0);
    m_outtakecontroller3.setSetpoint(Velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public void reachTestSpeed(double testVelocity) {
    outtakemotor1.set(testVelocity);
    outtakemotor2.set(testVelocity);
    outtakemotor3.set(testVelocity);
    
    
  }
  public void stopMotor() {
    outtakemotor1.stopMotor();
    outtakemotor2.stopMotor();
    outtakemotor3.stopMotor();
  }
  public double[] getMotorVelocity() {
    double outtakemotor1velocity = m_outtakemotorencoder1.getVelocity();
    double outtakemotor2velocity = m_outtakemotorencoder2.getVelocity();
    double outtakemotor3velocity = m_outtakemotorencoder3.getVelocity();
    return new double[]{outtakemotor1velocity, outtakemotor2velocity, outtakemotor3velocity};
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
