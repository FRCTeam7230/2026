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
public class AlignToHubWasher extends Command {
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
  public AlignToHubWasher(DriveSubsystem drive, XboxController cont, ShooterSubsystem shoot) {
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
    //cant move while shoot
    m_drive.drive(-xSpeed, -ySpeed, -rotSpeed,true);

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
    double initialRadius = Constants.AlignToHubConstants.kradius; //meters
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
    
    double radius = distance;
    double targetVelocityX = Math.sqrt(
      (9.81*Math.pow(distance*Constants.AlignToHubConstants.kDistMulti, 2))
      /(2*Math.tan(Math.toRadians(Constants.AlignToHubConstants.kejectionAngle))*distance*Constants.AlignToHubConstants.kDistMulti - (Constants.AlignToHubConstants.kHubHeight-Constants.AlignToHubConstants.kShooterHeight))
      );

    double targetVelocity =  targetVelocityX / Math.cos(Math.toRadians(Constants.AlignToHubConstants.kejectionAngle)) - m_drive.getSpeeds().vxMetersPerSecond;
    
    if((radius-initialRadius)>Constants.AlignToHubConstants.kRadiusToleranceBackward){
        radius = initialRadius + Constants.AlignToHubConstants.kRadiusToleranceBackward;
    } else if((radius-initialRadius)<Constants.AlignToHubConstants.kRadiusToleranceForward){
      radius = initialRadius + Constants.AlignToHubConstants.kRadiusToleranceForward;
    }
    //robot position error calculations
    double errorX = distanceX * ( (distance - radius) / distance );
    double errorY = distanceY * ( (distance - radius) / distance );
    //target angle and error angle calculations
    double targetAngle = Math.signum(distanceY) * Math.toDegrees(Math.acos(distanceX / distance));
    globalTargetAngle = targetAngle;
    double errorAngle = targetAngle - pose.getRotation().getDegrees();

    //sets rpm because cant set velocity
    m_shoot.reachSpeed(calculateRPM(radius));

    //sets each error value based on whether it is within it's respective tolerance
    if (Math.abs(errorX) < Constants.AlignToHubConstants.kerrorXTolerance) {
      errors[0] = 0;
    }
    else {errors[0] = errorX;}
    if (Math.abs(errorY) < Constants.AlignToHubConstants.kerrorYTolerance) {
      errors[1] = 0;
    }
    else {errors[1] = errorY;}
       
    if (Math.abs(errorAngle) < Constants.AlignToHubConstants.kerrorAngleTolerance) {
      errors[2] = 0;
    }
    else {errors[2] = errorAngle;}
  
    SmartDashboard.putNumber("AlignToHubWasher/zInitialVelocity", zInitialVelocityRobotRelative);
    SmartDashboard.putNumber("AlignToHubWasher/initalEjectionVelocity", initialEjectionVelocityAfterOffset);

    SmartDashboard.putNumber("AlignToHubWasher/TargetAngle",targetAngle);
    SmartDashboard.putNumber("AlignToHubWasher/ErrorX", errors[0]);
    SmartDashboard.putNumber("AlignToHubWasher/ErrorY", errors[1]);
    SmartDashboard.putNumber("AlignToHubWasher/ErrorAngle", errors[2]);

    SmartDashboard.putNumber("AlignToHubWasher/RobotX", robotX);
    SmartDashboard.putNumber("AlignToHubWasher/RobotY", robotY);
    SmartDashboard.putNumber("AlignToHubWasher/RobotAngle", pose.getRotation().getDegrees());
      
    return errors;
  }

  private double calculateRPM(double radialdistance) {
    //george's physics approximation in an ideal world: m_shoot.LinearVelToRPM(4.2170362293*Math.sqrt(radialdistance));
    //regression: use this link to determine constants https://www.desmos.com/calculator/sqdpqsalxh
    double a  = 0;
    double a1 = 0;
    double a2 = 0;
    double a3 = 0;
    double a4 = 0;
    double a5 = 0;
    double a6 = 0;
    double x = radialdistance;
    //quadratic + sqrt regression
    return a + a1*(x-a2) + a3*(Math.pow(x-a4, 2)) + a5*(Math.pow(x-a6, 0.5));
  }
}
