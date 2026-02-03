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
  
  DoublePublisher jointEncoder_publisher = NetworkTableInstance.getDefault().getDoubleTopic("Algae/jointEncoderValue").publish();
  DoublePublisher targetPosition_publisher = NetworkTableInstance.getDefault().getDoubleTopic("Algae/jointTarget").publish();
  

  ArmFeedforward m_feedforward =
      new ArmFeedforward(
        Constants.IntakeConstants.kjointkS,
        Constants.IntakeConstants.kjointkG,
        Constants.IntakeConstants.kjointkV,
        Constants.IntakeConstants.kjointkA);


   public IntakeSubsystem(){
        
        m_config_joint.absoluteEncoder
        //.inverted(true) // idk what this does - musa
        .positionConversionFactor(360)
        .velocityConversionFactor(360);

        m_config_joint.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(Constants.IntakeConstants.kjointKp,Constants.IntakeConstants.kjointKi,Constants.IntakeConstants.kjointKd)
        .outputRange(Constants.IntakeConstants.koutputMin,Constants.IntakeConstants.koutputMax, ClosedLoopSlot.kSlot0)
        .positionWrappingEnabled(true) //need because we dont want the joint arm to go throught the robot and horizontal is 0
        .positionWrappingInputRange(-180.0, 180.0) //arm is mounted on the rear
        .maxMotion
        .maxAcceleration(0) //we dont know what this does but it works
        .cruiseVelocity(0) //we dont know what this does but it works
        .allowedProfileError(Constants.IntakeConstants.kallowedError);

        m_config_joint.idleMode(IdleMode.kBrake);
        m_config_joint.smartCurrentLimit(Constants.IntakeConstants.kMaxCurrent);

        m_config_roller.idleMode(SparkBaseConfig.IdleMode.kBrake);
        m_config_roller.smartCurrentLimit(Constants.IntakeConstants.kMaxCurrent);

        m_joint.configure(m_config_joint, ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
        m_roller.configure(m_config_roller, ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
        
        desiredAngle = 0;
    }

    public double getEncoderDegrees() {
      return m_jointEncoder.getPosition() + Constants.IntakeConstants.kjointDegreeOffset;
    }
    
    public void spinRoller(double percentage) {
      m_roller.set(percentage);
    }
    public void stop() {
      m_roller.set(0);
    }

    public void jointreachGoal(double goal) {
      desiredAngle = goal;
      m_controller.setSetpoint(
        goal,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        m_feedforward.calculate(Math.toRadians(getEncoderDegrees()), Math.toRadians(m_jointEncoder.getVelocity()))
        );
    }

    public Command setGoal(double goal){
        return run(()-> jointreachGoal(goal));//converting runnable to command. 
    }

  
    public void periodic(){
      jointEncoder_publisher.set(getEncoderDegrees());
      targetPosition_publisher.set(desiredAngle);

    }




  


}

