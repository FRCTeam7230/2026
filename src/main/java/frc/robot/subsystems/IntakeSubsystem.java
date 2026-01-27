/* 
 * Pesudocode:

VARIABLES:

	Spark motors: jointMotor& rollerMotor
	Subsystems: none
ArmFeedforward jointFeedforward = new ArmFeedforward(s,g,v,a)
AbsoluteEncoder jointEncoder = jointMotor.getAbsoluteEncoder()

METHODS:

spinRoller(double percent)
		rollerMotor.set(percent)

reachGoal(positionGoal)
controller.setSetpoint(positionGoal, ControlType.kPosition, ClosedLoopSlot.kSlot0, jointFeedforward.calculate(radianPosition, velocity))
// make sure use Math.toRadians() so we can stay in degrees the whole time

CONSTANTS

	rollerMotor speed
	Kp, ki, kd
	Joint retracted position
	Joint extended position
	CAN IDs

 * 
*/
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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase
{
  // Set up Joint and Shooter properties
  
  private final SparkMax                  m_joint               = new SparkMax(Constants.IntakeConstants.kJointCANID, MotorType.kBrushless);
  private final SparkClosedLoopController m_controller          = m_joint.getClosedLoopController();
  private final SparkAbsoluteEncoder      m_jointEncoder        = m_joint.getAbsoluteEncoder();
  private final SparkMaxConfig            m_config_joint        = new SparkMaxConfig();
  
  private double desiredAngle;
  private double storedPosition;
  
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
        .positionWrappingEnabled(true) //need because we dont want the joint arm to go throught the robot and horizontal is 0 - musa
        .positionWrappingInputRange(-180.0, 180.0) //arm is mounted on the rear
        .maxMotion
        .maxAcceleration(0) //we dont know what this does but it works
        .maxVelocity(0) //we dont know what this does but it works
        .allowedClosedLoopError(Constants.IntakeConstants.kallowedError);

        m_config_joint.idleMode(IdleMode.kBrake);
        m_config_joint.smartCurrentLimit(Constants.IntakeConstants.kMaxCurrent);

        m_joint.configure(m_config_joint, ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
        
        desiredAngle = 0;
        storedPosition = Constants.IntakeConstants.kinitialStoredPostion;
    }

    public double getEncoderDegrees() {
      return m_jointEncoder.getPosition() + Constants.IntakeConstants.kjointDegreeOffset;
    }

    //these most likely wont be used
    public void spin(double percentage) {
      m_joint.set(percentage);
    }
    public void stop() {
      m_joint.set(0);
    }

    public double getAngle() {
      return getEncoderDegrees();
    }

    //goal is the desired angle and override is true when to store the goal in the storedPosition
    public void reachGoal(double goal, boolean override) {
      desiredAngle = goal;
      m_controller.setReference(
        goal,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        m_feedforward.calculate(Math.toRadians(getEncoderDegrees()), Math.toRadians(m_jointEncoder.getVelocity()))
        );
      //this is so the goal is kept once arm is safe
      if (!override){
        storedPosition = goal;
      }
    }

    //this is the command that will actually run the reach goal method
    public Command setGoal(double goal){
        return run(()-> reachGoal(goal,false));//converting runnable to command. 
    }

  
    public void periodic(){
      

    }




  


}

