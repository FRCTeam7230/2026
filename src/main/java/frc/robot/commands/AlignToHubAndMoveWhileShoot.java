// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToHubAndMoveWhileShoot extends Command {
  /** Creates a new AlignToHub. */
  DriveSubsystem m_drive;
  XboxController controller;
  ShooterSubsystem m_shoot;
  PIDController xController = new PIDController(1, 0, 0);
  PIDController yController = new PIDController(1, 0, 0);
  PIDController rotController = new PIDController(0.03, 0, 0);

  //offset calculation variables
  double globalTargetAngle;
  double initialEjectionVelocityAfterOffset; //m/s
  double timeOfFlight;
  double zInitialVelocityRobotRelative = 0;
  double vx0 = Constants.AlignToHubConstants.vx0;
  double x0 = Constants.AlignToHubConstants.x0;

  private final Timer timer = new Timer();
  private double lastTime = 0;
  double radialOffset = 0;

  
  Command drivecommand = null;
  public AlignToHubAndMoveWhileShoot(DriveSubsystem drive, XboxController cont, ShooterSubsystem shoot) {
    m_drive = drive;
    controller = cont;
    m_shoot = shoot;
    xController.setSetpoint(0);
    yController.setSetpoint(0);
    rotController.setSetpoint(0);
    rotController.enableContinuousInput(-180, 180);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    timer.start();
    radialOffset = 0;
    lastTime = timer.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double deltaTime = timer.get() - lastTime;
    lastTime = timer.get();
    radialOffset = controller.getRawAxis(1)*deltaTime*Constants.AlignToHubConstants.kSpeedMulti;
    Pose2d currentPose = m_drive.getPose();
    double[] errors = CalculateHubPID(currentPose, radialOffset);
    double xSpeed = xController.calculate(errors[0]);
    double ySpeed = yController.calculate(errors[1]);
    double rotSpeed = Math.max(Math.min(rotController.calculate(errors[2]),1.5), -1.5);
    SmartDashboard.putNumber("Rotation delivered", rotSpeed);
    //uses controller stick axis to control robot movement while shooting
    m_drive.drive(
        -(xSpeed + (controller.getRawAxis(Constants.ControllerConstants.leftStick_XAXIS) * Constants.AlignToHubConstants.kspeedMult * Math.sin(Math.toRadians(-globalTargetAngle)))),
        -(ySpeed + (controller.getRawAxis(Constants.ControllerConstants.leftStick_XAXIS) * Constants.AlignToHubConstants.kspeedMult * Math.cos(Math.toRadians(-globalTargetAngle)))),
        -rotSpeed,
        true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_drive.setX();
    drivecommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
  public double[] CalculateHubPID(Pose2d pose,double radalOffset) {
    //SAME AS ALIGN TO HUB
    //setting robot x and y  
    double robotX = pose.getX();
	  double robotY = pose.getY();

    //this array is where errorX, errorY, and errorAngle will be stored and eventally returned
    double[] errors = new double[3];
    //initilizes errors
    for (int i = 0; i < errors.length; i++) {
      errors[i] = 0;
    }
    //setting constants to local variables for readability
    double Radius = Constants.AlignToHubConstants.kradius; //meters
    double hubY = Constants.AlignToHubConstants.khubY; // meters
    double hubXBlue = Constants.AlignToHubConstants.khubXBlue;
    double hubXRed = Constants.AlignToHubConstants.khubXRed;
    double hubX;
    //determines hubX based on which alliance we are on
    if (DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue))) {
        hubX = hubXBlue;
    }
    else {
      hubX = hubXRed;
    }

    //error calculations
    //difference in current position to center of hub
    double distanceX = hubX - robotX;
    double distanceY = hubY - robotY;
    //pythagorean theorm to get overall distance
    double distance = Math.sqrt( Math.pow( distanceX, 2) + Math.pow( distanceY, 2) );
    
    double radius = distance + radalOffset;
    double targetVelocityX = Math.sqrt(
      (9.81*Math.pow(distance, 2))
      /(2*Math.tan(Math.toRadians(Constants.AlignToHubConstants.kejectionAngle))*distance - (Constants.AlignToHubConstants.kHubHeight-0.38))
      );

    double targetVelocity =  targetVelocityX / Math.cos(Math.toRadians(Constants.AlignToHubConstants.kejectionAngle)) - m_drive.getSpeeds().vxMetersPerSecond;
    
    if((radius-Radius)>Constants.AlignToHubConstants.kRadiusToleranceBackward){
        radius = Radius + Constants.AlignToHubConstants.kRadiusToleranceBackward;
    } else if((radius-Radius)<Constants.AlignToHubConstants.kRadiusToleranceForward){
      radius = Radius + Constants.AlignToHubConstants.kRadiusToleranceForward;
    }
    //robot position error calculations
    double errorX = distanceX * ( (distance - radius) / distance );
    double errorY = distanceY * ( (distance - radius) / distance );
    //target angle and error angle calculations
    double targetAngle = Math.signum(distanceY) * Math.toDegrees(Math.acos(distanceX / distance));
    globalTargetAngle = targetAngle;
    double errorAngle = targetAngle - pose.getRotation().getDegrees();

    //DIFFERENCE TO ALIGN TO HUB
    //reference this 3D desmos graph for visual clarification: https://www.desmos.com/3d/uvvzkss2ac

    //angle offset calculations

    //this changes the reachSpeed of shooter for a more optimal trajectory
    initialEjectionVelocityAfterOffset = Constants.AlignToHubConstants.kinitialEjectionVelocityBeforeOffset + Math.abs(0.3*zInitialVelocityRobotRelative);
    initialEjectionVelocityAfterOffset += targetVelocity-Constants.AlignToHubConstants.kinitalAtTwoPointSevenFive;
    m_shoot.reachSpeed(initialEjectionVelocityAfterOffset);
    

    //initial x fuel velocity
    vx0 = Constants.AlignToHubConstants.kinitialEjectionVelocityBeforeOffset*Math.cos(Math.toRadians(Constants.AlignToHubConstants.kejectionAngle));
    zInitialVelocityRobotRelative = m_drive.getRobotVelocityY();

    //time of normal trajectory from shooter to hub
    double t1 = (radius - x0)/(Constants.AlignToHubConstants.kinitialEjectionVelocityBeforeOffset*Math.cos(Math.toRadians(Constants.AlignToHubConstants.kejectionAngle)));
    //first angle offset calculation that makes fuel trajectory parallel with the actual target angle (it's theta2 because theta1 is the ejection angle, even though they are in different planes)
    double theta2Rad = (Math.atan( (sz(t1)) / (radius - Constants.AlignToHubConstants.kshooterOffset) )); //rad

    //new x0 due to y-axis rotation by theta2
    double x0New = Constants.AlignToHubConstants.kshooterOffset * Math.cos((theta2Rad));
    //new time which is dependant on theta2Rad (therefore also dependant on zInitialVelocityRobotRelative) because of the y-axis rotation by theta2
    double t2 = (radius - x0New) / ( (vx0 * Math.cos((theta2Rad))) + (zInitialVelocityRobotRelative * Math.sin(theta2Rad)) );
    //second angle offset calculation that corrects for the shooterOffset because it was rotated about the y-axis by theta2, truely aligning the angle with the center of the hub
    double theta3Rad = (Math.atan( (szNew(t2,theta2Rad)) / ( (sxNew(t2,theta2Rad)) - x0New) )); //rad

    //combines both angles to get the overall angleOffset
    double angleOffsetRad = theta2Rad + theta3Rad; // rad

    //applies angleOffset to errorAngle
    double newAngleError = errorAngle - Math.toDegrees(angleOffsetRad); //deg

    //sets each error value based on whether it is within it's respective tolerance
    if (Math.abs(errorX) < Constants.AlignToHubConstants.kerrorXTolerance) {
      errors[0] = 0;
    }
    else {errors[0] = errorX;}
    if (Math.abs(errorY) < Constants.AlignToHubConstants.kerrorYTolerance) {
      errors[1] = 0;
    }
    else {errors[1] = errorY;}
       
    if (Math.abs(newAngleError) < Constants.AlignToHubConstants.kerrorAngleTolerance) {
      errors[2] = 0;
    }
    else {errors[2] = newAngleError;}
      
    SmartDashboard.putNumber("AlignToHub/theta2Deg", Math.toDegrees(theta2Rad));
    SmartDashboard.putNumber("AlignToHub/theta3Deg", Math.toDegrees(theta3Rad));
    SmartDashboard.putNumber("AlignToHub/angleOffset", Math.toDegrees(angleOffsetRad));
    SmartDashboard.putNumber("AlignToHub/zInitialVelocity", zInitialVelocityRobotRelative);
    SmartDashboard.putNumber("AlignToHub/initalEjectionVelocity", initialEjectionVelocityAfterOffset);

    SmartDashboard.putNumber("AlignToHub/TargetAngle",targetAngle);
    SmartDashboard.putNumber("AlignToHub/ErrorX", errors[0]);
    SmartDashboard.putNumber("AlignToHub/ErrorY", errors[1]);
    SmartDashboard.putNumber("AlignToHub/ErrorAngle", errors[2]);

    SmartDashboard.putNumber("AlignToHub/RobotX", robotX);
    SmartDashboard.putNumber("AlignToHub/RobotY", robotY);
    SmartDashboard.putNumber("AlignToHub/RobotAngle", pose.getRotation().getDegrees());
      
    return errors;
  }

  //sx calculates the position of the fuel at any time t
  private double sx(double t) {
    return x0 + vx0*t + .5*Constants.AlignToHubConstants.ax*Math.pow(t,2);
  }
  //sz calculates the position of the fuel at any time t
  private double sz(double t) {
    return Constants.AlignToHubConstants.z0 + zInitialVelocityRobotRelative*t + .5*Constants.AlignToHubConstants.az*Math.pow(t,2);
  }
  //sxNew calculates the position of the fuel at any time t and applies rotation about y-axis by angle theta
  private double sxNew(double t, double theta) { //angle in rad
    return sx(t)*Math.cos((theta)) + sz(t)*Math.sin((theta)); //applies rotation about y-axis by angle theta
  }
  //szNew calculates the position of the fuel at any time t and applies rotation about y-axis by angle theta
  private double szNew(double t, double theta) { //angle in rad
    return sz(t)*Math.cos((theta)) - sx(t)*Math.sin((theta)); //applies rotation about y-axis by angle theta
  }
}
