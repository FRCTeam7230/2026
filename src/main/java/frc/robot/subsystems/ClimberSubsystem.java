package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase 
{
  private final SparkMax                  m_motor1        = new SparkMax(ClimberConstants.kClimMotor1, MotorType.kBrushless);
  private final SparkClosedLoopController m_controller    = m_motor1.getClosedLoopController();
  private final RelativeEncoder           m_encoder       = m_motor1.getEncoder();
  private final SparkMaxConfig            m_config_motor1 = new SparkMaxConfig();
  private double m_desiredHeight = 0.0;

  private final ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          ClimberConstants.kClimberkS,
          ClimberConstants.kClimberkG,
          ClimberConstants.kClimberkV,
          ClimberConstants.kClimberkA);

  private final DoublePublisher  encoder1_publisher    = NetworkTableInstance.getDefault().getDoubleTopic("Climber/encoder1value").publish();
  private final DoublePublisher  velocity_publisher    = NetworkTableInstance.getDefault().getDoubleTopic("Climber/velocity").publish();
  private final DoublePublisher  output1_publisher     = NetworkTableInstance.getDefault().getDoubleTopic("Climber/outputMotor1").publish();
  private final DoublePublisher  heightError_publisher = NetworkTableInstance.getDefault().getDoubleTopic("Climber/heightError").publish();
  private final DoublePublisher  current1_publisher    = NetworkTableInstance.getDefault().getDoubleTopic("Climber/currentMotor1").publish();
  private final BooleanPublisher climReset_publisher   = NetworkTableInstance.getDefault().getBooleanTopic("Climber/resetElev").publish();

  // Constructor

  public ClimberSubsystem()
  {
    m_config_motor1.encoder
        .positionConversionFactor(ClimberConstants.kRotationToMeters)
        .velocityConversionFactor(ClimberConstants.kRotationToMeters / 60.0);
    m_config_motor1.closedLoop
        .pid(ClimberConstants.kCLimberKp, ClimberConstants.kCLimberKi, ClimberConstants.kClimberKd, ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot0);
    m_config_motor1.idleMode(SparkBaseConfig.IdleMode.kBrake);
    m_config_motor1.smartCurrentLimit(ClimberConstants.kMaxCurrent);
    m_config_motor1.closedLoopRampRate(ClimberConstants.kClimberRampRate);

    // Apply config
    m_motor1.configure(m_config_motor1, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // desired height
    m_desiredHeight = 0.0;
  }

  public void motorStop()
  {
    stop();
  }

  public void resetEncoder()
  {
    m_encoder.setPosition(0);
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain (meters)
   */
  public void reachGoal(double goal)
  {
    m_desiredHeight = MathUtil.clamp(goal,
        ClimberConstants.kMinRealClimberHeightMeters,
        ClimberConstants.kMaxRealClimberHeightMeters);
    m_controller.setSetpoint(m_desiredHeight,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        m_feedforward.calculate(m_encoder.getVelocity()));
  }

  /**
   * Get the height in meters.
   *
   * @return Height in meters
   */
  public double getHeight()
  {
    return m_encoder.getPosition();
  }

  // Soft limit buffer
  private static final double kLimitBufferMeters = 0.01;

  private boolean atLowerLimit() {
    return getHeight() <= (ClimberConstants.kMinRealClimberHeightMeters + kLimitBufferMeters);
  }

  private boolean atUpperLimit() {
    return getHeight() >= (ClimberConstants.kMaxRealClimberHeightMeters - kLimitBufferMeters);
  }

  /**
  * Apply manual motor output but stop if we would drive past soft limits.
  * @param percent [-1, 1]
  */
  private void setManualOutputSafe(double percent) {
    if (percent > 0 && atUpperLimit()) {
      m_motor1.set(0);
      return;
    }
    if (percent < 0 && atLowerLimit()) {
      m_motor1.set(0);
      return;
    }
    m_motor1.set(percent);
  }

  //Climber TODO: Add comment to explain this (similar to other methods)
  public void setManualOutput(double percent) {
    m_motor1.set(percent);
  }
  /**
   * A trigger for when the height is at an acceptable tolerance.
   *
   * @param height    Height in Meters
   * @param tolerance Tolerance in meters.
   * @return {@link Trigger}
   */
  public Trigger atHeight(double height, double tolerance)
  {
    return new Trigger(() -> MathUtil.isNear(height, getHeight(), tolerance));
  }

  public boolean isFullyExtended(double tolerance)
  {
    return MathUtil.isNear(ClimberConstants.kMaxRealClimberHeightMeters, getHeight(), tolerance);
  }

  /**
   * Set the goal of the climber
   *
   * @param goal Goal in meters
   * @return {@link edu.wpi.first.wpilibj2.command.Command}
   */
  public Command setGoal(double goal)
  {
    return run(() -> reachGoal(goal));
  }

  public void stop()
  {
    m_motor1.set(0.0);
  }

  public void ManualClimberUp()
  {
    setManualOutputSafe(0.2);
  }

  public void HoverClimber()
  {
    if (atUpperLimit()) {
      m_motor1.set(0);
      return;
    }
    m_motor1.setVoltage(ClimberConstants.kClimberkG);
  }

  public void ManualClimberDown()
  {
    setManualOutputSafe(-0.15);
  }

  public void ClimberIncrementDown() 
  {
    reachGoal(Constants.ClimberConstants.kMinRealClimberHeightMeters); 
  }

  //Update telemetry
  public void updateTelemetry()
  {
    SmartDashboard.putNumber("Climber Position", getHeight());
  }

  @Override
  public void periodic()
  {
    // Publish telemetry
    encoder1_publisher.set(m_encoder.getPosition());
    output1_publisher.set(m_motor1.getAppliedOutput());
    heightError_publisher.set(m_desiredHeight - getHeight());
    velocity_publisher.set(m_encoder.getVelocity());
    current1_publisher.set(m_motor1.getOutputCurrent());

    //Climber TODO: I'm not sure yet if we'll want to reset the climber position
    //Climber TODO: Either way, reseting logic should be commented out to start
    boolean reset = false;
    if (Math.abs(m_encoder.getVelocity()) < 0.01 && m_motor1.getOutputCurrent() > ClimberConstants.kResetCurrent) {
      if (m_motor1.getAppliedOutput() > 0) {
        m_encoder.setPosition(ClimberConstants.kMaxRealClimberHeightMeters);
      }
      else {
        m_encoder.setPosition(ClimberConstants.kMinRealClimberHeightMeters);
      }
      m_motor1.set(0);
      reset = true;
    }

    SmartDashboard.putNumber("Climber Position (Meters)", m_encoder.getPosition());
    climReset_publisher.set(reset);

    // CLimber TODO: Comment this out to start with
    if ((atUpperLimit() && m_motor1.getAppliedOutput() > 0) ||
    (atLowerLimit() && m_motor1.getAppliedOutput() < 0)) {
      m_motor1.set(0);
    }
  }
}