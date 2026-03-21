package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

/**
 * Subsystem controlling the Intake join and the roller on the end of the joint for intaking fuel into the robot. 
 */
public class IntakeSubsystem extends SubsystemBase
{
  // Set up Joint and Shooter properties
  /**Motor Controlling the joint for the intake, which is also connected to the hopper extensions */
  private final SparkMax                  m_joint               = new SparkMax(Constants.IntakeConstants.kJointCANID, MotorType.kBrushless);
  /**Motor controlling the roller on one end of the intake, which must be running in order to get balls through the intake */
  private final SparkMax                  m_roller              = new SparkMax(Constants.IntakeConstants.kRollerCANID, MotorType.kBrushless);
  /**Closed Loop (PID) Controller for the joint to move it to set positions */
  private final SparkClosedLoopController m_controller          = m_joint.getClosedLoopController();
  /**Encoder on the joint to determine its position for PID movement and precise tracking */
  private final SparkAbsoluteEncoder      m_jointEncoder        = m_joint.getAbsoluteEncoder();
  /**Config object for the joint motor */
  private final SparkMaxConfig            m_config_joint        = new SparkMaxConfig();
  /**Config object for the first roller motor */
  private final SparkMaxConfig            m_config_roller       = new SparkMaxConfig();
  /**The Angle we want the join to be at, used for debugging and advantageScope */
  private double desiredAngle;

  private boolean isIntakeOut = false;
  private boolean isRollerSpinning = true;

  DoublePublisher jointEncoder_publisher = NetworkTableInstance.getDefault().getDoubleTopic("Intake/jointEncoderValue").publish();
  DoublePublisher targetPosition_publisher = NetworkTableInstance.getDefault().getDoubleTopic("Intake/jointTarget").publish();
  DoublePublisher rollerCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Intake/RollerCurrent").publish();
  DoublePublisher jointCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Intake/JointCurrent").publish();
  DoublePublisher jointVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Intake/JointVoltage").publish();
  BooleanPublisher isIntakeOutPublisher = NetworkTableInstance.getDefault().getBooleanTopic("Intake/IsIntakeOut").publish();
  /**
   * Feedforward class that accounts for gravity for the intake PID 
  */
  ArmFeedforward m_feedforward =
      new ArmFeedforward(
        0,
        Constants.IntakeConstants.kjointkG,
        0,
        0);

  /**Constructs an intake subsystem */
   public IntakeSubsystem(){
        
        m_config_joint.absoluteEncoder
        //.inverted(true) // idk what this does - musa //Intake TODO: You want positive motor values to correspond to positive increaing encoder values, if that's not true, you can set inverted to true
        .positionConversionFactor(360) //Intake TODO: Are we sure about these conversion factors? What did L1 use? - L1 used the same
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
        .allowedProfileError(0); //Not needed, can leave with set to zero
        
        m_config_joint.idleMode(IdleMode.kCoast);
        m_config_joint.smartCurrentLimit(Constants.IntakeConstants.kMaxCurrent); //Intake TODO: MaxCurrent can go up for the Neo 1.1, maybe 40 or 60?
        
        m_config_roller.idleMode(SparkBaseConfig.IdleMode.kBrake);
        m_config_roller.smartCurrentLimit(Constants.IntakeConstants.kMaxCurrent); //Intake TODO: MaxCurrent can go up for the Neo 1.1, maybe 40 or 60?
        //This setup means that it will not reset certain "safe" parameters to their defaults, and will also persist the new parameters we set
        m_joint.configure(m_config_joint, ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
        m_roller.configure(m_config_roller, ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
        
        desiredAngle = 0;
    }
    /**Spins the roller on the end of the joint
     * @param percentage the percentage to spin the roller at, between -1 and 1, where 1 is full forward and -1 is full reverse.
     */
    public void spinRoller(double percentage) {
      m_roller.set(percentage);
      if(percentage == 0){
        isRollerSpinning = false;
      }
      else{
        isRollerSpinning = true;
      }
    }
    /**Turns the roller off */
    public void stop() {
      m_roller.set(0);
    }

    //for testing
    /**Manual joint control method for testing purposes
     * @param percentage the percentage to spin the join at. Since the joint has constraints, less than 20% is recommended at first. 
     */
    public void spinJoint(double percentage) {
      m_joint.set(percentage);
    }
    /**
     * Method for testing the kG feedforward value for the join PID. 
     * When run, it should hold the joint at a fixed position with no loss from gravity. Adjust kG accordingly if it does not. 
     */
    public void hoverJoint() {
      m_controller.setSetpoint(
        m_feedforward.calculate(Math.toRadians(m_jointEncoder.getPosition()), Math.toRadians(m_jointEncoder.getVelocity())),
        ControlType.kVoltage,
        ClosedLoopSlot.kSlot0
      );
    }
    /** Adjusts the PID setpoint for the intake joint and starts the intake moving there
     * @param goal the angle in degrees to move the joint to, where 0 is horizontal. 
    */
    public void reachGoal(double goal) {
      desiredAngle = goal;
      if(goal==Constants.IntakeConstants.kretractedPostion){
        isIntakeOut = false;
      }
      else{
        isIntakeOut = true;
      }
      m_controller.setSetpoint(
        goal,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        m_feedforward.calculate(Math.toRadians(m_jointEncoder.getPosition()), Math.toRadians(m_jointEncoder.getVelocity()))
        );
    }
    /** simple run command that uses reach goal 
     * @param goal the angle in degrees to move the joint to, where 0 is horizontal.
    */
    public Command setGoal(double goal){
        return run(()-> reachGoal(goal));//converting runnable to command. 
    }

    /**
     * Periodic runs every cycle. It published the encoder value and target position for the joint to networktables for debugging. 
     */
    public void periodic(){
      jointEncoder_publisher.set(m_jointEncoder.getPosition());
      targetPosition_publisher.set(desiredAngle);
      rollerCurrentPublisher.set(m_roller.getOutputCurrent());
      jointCurrentPublisher.set(m_joint.getOutputCurrent());
      isIntakeOutPublisher.set(isIntakeOut);
    }
    
    public boolean reachedGoal(){
      return m_controller.getSetpoint()==m_jointEncoder.getPosition();
    }

    /**
     * Overridden
     */
    public boolean reachedGoal(double goal){
      return m_controller.getSetpoint()==goal;
    }

    public Command jiggleIntake(double goal, double currentLimit){
      return new RunCommand(() -> {
        reachGoal(goal);
      }).until(
        () -> reachedGoal() || (m_joint.getOutputCurrent()<currentLimit)
      ).andThen(
        () -> {
          reachGoal(Constants.IntakeConstants.kextendedPostion);
        }
      ).until(
        () -> reachedGoal() || (m_joint.getOutputCurrent()<currentLimit)
      );
    }
    public Command toggleIntake(boolean intakeOut)
    {
      
      if(intakeOut)
      {
        return new InstantCommand(()->{
        reachGoal(Constants.IntakeConstants.kretractedPostion);
        isIntakeOut = false;
        spinRoller(0);
        }
        ,this
        );
      }
      else
      {
        return new InstantCommand(()->{
        reachGoal(Constants.IntakeConstants.kextendedPostion);
        isIntakeOut = true;},this)
        .andThen(new WaitUntilCommand(()-> {return fullyExtended();}))
        .andThen(new InstantCommand(()->spinRoller(Constants.IntakeConstants.kintakeRollerSpeed),this));
      }
    }
    public void toggleIntakeRoller()
    {
      if(isRollerSpinning)
      {
        spinRoller(0);
      }
      else
      {
        spinRoller(Constants.IntakeConstants.kintakeRollerSpeed);
      }
    }
    public boolean getIntakeOut()
    {
      return isIntakeOut;
    }
    public boolean getRollerSpinning()
    {
      return isRollerSpinning;
    }
    public boolean fullyExtended()
    {
      if (Math.abs(m_jointEncoder.getPosition() - Constants.IntakeConstants.kextendedPostion) < Constants.IntakeConstants.kpositionTolerance) {
       return true;
      }
      else {
        return false;
      }
    }



  


}

