// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OuttakeConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystemcurrently. */
  private final SparkFlex outtakemotor1 = new SparkFlex(Constants.OuttakeConstants.ShootMotor1ID, MotorType.kBrushless); 
  private final SparkFlex outtakemotor2 = new SparkFlex(Constants.OuttakeConstants.ShootMotor2ID, MotorType.kBrushless); 
  private final SparkFlex outtakemotor3 = new SparkFlex(Constants.OuttakeConstants.ShootMotor3ID, MotorType.kBrushless); 
  private final RelativeEncoder m_outtakemotorencoder1 = outtakemotor1.getEncoder(); 
  private final RelativeEncoder m_outtakemotorencoder2 = outtakemotor2.getEncoder(); 
  private final RelativeEncoder m_outtakemotorencoder3 = outtakemotor3.getEncoder();  
  private final SparkFlexConfig m_outtakemotor1config = new SparkFlexConfig(); 
  private final SparkFlexConfig m_outtakemotor2config = new SparkFlexConfig(); 
  private final SparkFlexConfig m_outtakemotor3config = new SparkFlexConfig(); 
  private final SparkClosedLoopController m_outtakecontroller1 = outtakemotor1.getClosedLoopController();
  private final SparkClosedLoopController m_outtakecontroller2 = outtakemotor2.getClosedLoopController();
  private final SparkClosedLoopController m_outtakecontroller3 = outtakemotor3.getClosedLoopController();
  
  DoubleArrayPublisher velocityPublisher = NetworkTableInstance.getDefault().getDoubleArrayTopic("Shooter/Velocities").publish();
  DoubleArrayPublisher setpointPublisher = NetworkTableInstance.getDefault().getDoubleArrayTopic("Shooter/Setpoints").publish();
  DoubleArrayPublisher errorPublisher = NetworkTableInstance.getDefault().getDoubleArrayTopic("Shooter/Errors").publish();

  public ShooterSubsystem() {
    //Motor 1
    m_outtakemotor1config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(OuttakeConstants.kOuttakeKp, OuttakeConstants.kOuttakeKi, OuttakeConstants.kOuttakeKd)
    .feedForward.kV(OuttakeConstants.kOuttakeKf); 
    m_outtakemotor1config.idleMode(IdleMode.kCoast);
    m_outtakemotor1config.smartCurrentLimit(Constants.OuttakeConstants.motorlimitcurrent);
    m_outtakemotor1config.closedLoopRampRate(Constants.OuttakeConstants.krampratesec);
    
    //Motor 2
    m_outtakemotor2config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(OuttakeConstants.kOuttakeKp, OuttakeConstants.kOuttakeKi, OuttakeConstants.kOuttakeKd)
    .feedForward.kV((Constants.OuttakeConstants.kOuttakeKf)*12); 
    m_outtakemotor2config.idleMode(IdleMode.kCoast);
    m_outtakemotor2config.smartCurrentLimit(Constants.OuttakeConstants.motorlimitcurrent);
    m_outtakemotor2config.closedLoopRampRate(Constants.OuttakeConstants.krampratesec);
    
    //Motor 3
    m_outtakemotor3config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(OuttakeConstants.kOuttakeKp, OuttakeConstants.kOuttakeKi, OuttakeConstants.kOuttakeKd)
    .feedForward.kV(OuttakeConstants.kOuttakeKf);
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
    velocityPublisher.set(getMotorVelocity());
    setpointPublisher.set(new double[]{m_outtakecontroller1.getSetpoint(), m_outtakecontroller2.getSetpoint(), m_outtakecontroller3.getSetpoint()});
    errorPublisher.set(new double[]{m_outtakecontroller1.getSetpoint()-getMotorVelocity()[0], m_outtakecontroller2.getSetpoint()-getMotorVelocity()[1], m_outtakecontroller3.getSetpoint()-getMotorVelocity()[2]});

    
  }
}
