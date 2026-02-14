package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase
{
  // Set up Joint and Shooter properties
  
  private final SparkMax                  m_joint               = new SparkMax(Constants.IntakeConstants.kJointCANID, MotorType.kBrushless);
  private final SparkMax                  m_roller              = new SparkMax(Constants.IntakeConstants.kRollerCANID, MotorType.kBrushless);
  private final SparkClosedLoopController m_controller          = m_joint.getClosedLoopController();
  private final SparkAbsoluteEncoder      m_jointEncoder        = m_joint.getAbsoluteEncoder();
  private final SparkMaxConfig            m_config_joint        = new SparkMaxConfig();
  private final SparkMaxConfig            m_config_roller       = new SparkMaxConfig();
  
  private double desiredAngle;
  
  DoublePublisher jointEncoder_publisher = NetworkTableInstance.getDefault().getDoubleTopic("Intake/jointEncoderValue").publish();
  DoublePublisher targetPosition_publisher = NetworkTableInstance.getDefault().getDoubleTopic("Intake/jointTarget").publish();
  

  ArmFeedforward m_feedforward =
      new ArmFeedforward(
        0,
        Constants.IntakeConstants.kjointkG,
        0,
        0);


   public IntakeSubsystem(){
        
        m_config_joint.absoluteEncoder
        //.inverted(true) // idk what this does - musa //Intake TODO: You want positive motor values to correspond to positive increaing encoder values, if that's not true, you can set inverted to true
        .positionConversionFactor(360) //Intake TODO: Are we sure about these conversion factors? What did L1 use?
        .velocityConversionFactor(360);

        m_config_joint.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(Constants.IntakeConstants.kjointKp,Constants.IntakeConstants.kjointKi,Constants.IntakeConstants.kjointKd)
        .outputRange(Constants.IntakeConstants.koutputMin,Constants.IntakeConstants.koutputMax, ClosedLoopSlot.kSlot0)
        .positionWrappingEnabled(true) //need because we dont want the joint arm to go throught the robot and horizontal is 0
        .positionWrappingInputRange(-180.0, 180.0) //arm is mounted on the rear
        .maxMotion
        .maxAcceleration(0) //we dont know what this does but it works //Not needed, can leave with set to zero
        .cruiseVelocity(0) //we dont know what this does but it works //Not needed, can leave with set to zero
        .allowedProfileError(Constants.IntakeConstants.kallowedError); //Not needed, can leave with set to zero

        m_config_joint.idleMode(IdleMode.kBrake);
        m_config_joint.smartCurrentLimit(Constants.IntakeConstants.kMaxCurrent); //Intake TODO: MaxCurrent can go up for the Neo 1.1, maybe 40 or 60?

        m_config_roller.idleMode(SparkBaseConfig.IdleMode.kBrake);
        m_config_roller.smartCurrentLimit(Constants.IntakeConstants.kMaxCurrent); //Intake TODO: MaxCurrent can go up for the Neo 1.1, maybe 40 or 60?

        m_joint.configure(m_config_joint, ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
        m_roller.configure(m_config_roller, ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
        
        desiredAngle = 0;
    }

    /* //dont need anymore
    public double getEncoderDegrees() {
      return m_jointEncoder.getPosition();
    }
      */
    
    public void spinRoller(double percentage) {
      m_roller.set(percentage);
    }
    public void stop() {
      m_roller.set(0);
    }

    //for testing
    public void spinJoint(double percentage) {
      m_joint.set(percentage);
    }
    public void hoverJoint() {
      m_controller.setSetpoint(m_feedforward.calculate(Math.toRadians(m_jointEncoder.getPosition()), Math.toRadians(m_jointEncoder.getVelocity())), ControlType.kVoltage, ClosedLoopSlot.kSlot0);
    }

    public void reachGoal(double goal) {
      desiredAngle = goal;
      m_controller.setSetpoint(
        goal,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        m_feedforward.calculate(Math.toRadians(m_jointEncoder.getPosition()), Math.toRadians(m_jointEncoder.getVelocity()))
        );
    }

    public Command setGoal(double goal){
        return run(()-> reachGoal(goal));//converting runnable to command. 
    }

  
    public void periodic(){
      jointEncoder_publisher.set(m_jointEncoder.getPosition());
      targetPosition_publisher.set(desiredAngle);

    }




  


}

