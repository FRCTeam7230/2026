// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
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
  public static double globalAngleOffsetRad = 0;
  public static void setGlobalAngleOffsetRad0() {
    globalAngleOffsetRad = 0;
  }
  
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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = m_drive.getPose();
    double[] errors = CalculateHubPID(currentPose);
    double xSpeed = xController.calculate(errors[0]);
    double ySpeed = yController.calculate(errors[1]);
    double rotSpeed = Math.max(Math.min(rotController.calculate(errors[2]),1.5), -1.5);
    SmartDashboard.putNumber("Rotation delivered", rotSpeed);
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
  public double[] CalculateHubPID(Pose2d pose) {
      double robotX = pose.getX();
	    double robotY = pose.getY();

      double[] errors = new double[3];
      double radius = Constants.AlignToHubConstants.kradius;
      double hubY = Constants.AlignToHubConstants.khubY; // meters
      double hubXBlue = Constants.AlignToHubConstants.khubXBlue;
      double hubXRed = Constants.AlignToHubConstants.khubXRed;
      double hubX;
      if (DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue))) {
          hubX = hubXBlue;
      }
      else {
          hubX = hubXRed;
      }

      //error calculations
      double distanceX = hubX - robotX;
      double distanceY = hubY - robotY;
      double distance = Math.sqrt( Math.pow( distanceX, 2) + Math.pow( distanceY, 2) );
       
      double errorX = distanceX * ( (distance - radius) / distance );
      double errorY = distanceY * ( (distance - radius) / distance );
      double targetAngle = Math.signum(distanceY) * Math.toDegrees(Math.acos(distanceX / distance));
      globalTargetAngle = targetAngle;
      double errorAngle = targetAngle - pose.getRotation().getDegrees();

      //angle offset calculations
      initialEjectionVelocityAfterOffset = Constants.AlignToHubConstants.kinitialEjectionVelocityBeforeOffset + Math.abs(0.1*zInitialVelocityRobotRelative);
      m_shoot.reachSpeed(initialEjectionVelocityAfterOffset);
      vx0 = Constants.AlignToHubConstants.kinitialEjectionVelocityBeforeOffset*Math.cos(Math.toRadians(Constants.AlignToHubConstants.kejectionAngle));

      timeOfFlight = (radius - x0)/(Constants.AlignToHubConstants.kinitialEjectionVelocityBeforeOffset*Math.cos(Math.toRadians(Constants.AlignToHubConstants.kejectionAngle)));
      zInitialVelocityRobotRelative = m_drive.getRobotVelocityY();

      
      double t1 = timeOfFlight; //time of normal trajectory from shooter to hub
      double theta2Rad = (Math.atan( (sz(t1)) / (radius - Constants.AlignToHubConstants.kshooterOffset) )); //rad

      double x0New = Constants.AlignToHubConstants.kshooterOffset * Math.cos((theta2Rad));
      double t2 = (radius - x0New) / ( (vx0 * Math.cos((theta2Rad))) + (zInitialVelocityRobotRelative * Math.sin(theta2Rad)) );
      double theta3Rad = (Math.atan( (szNew(t2,theta2Rad)) / ( (sxNew(t2,theta2Rad)) - x0New) )); //rad

      double angleOffsetRad = theta2Rad + theta3Rad; // rad. idk if add or sub. pretty sure add
      globalAngleOffsetRad = angleOffsetRad;

		  errors[0] = errorX;
		  errors[1] = errorY;
		  errors[2] = errorAngle - Math.toDegrees(angleOffsetRad); //deg
      
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
  private double sx(double t) {
    return x0 + vx0*t + .5*Constants.AlignToHubConstants.ax*Math.pow(t,2);
  }
  private double sz(double t) {
    return Constants.AlignToHubConstants.z0 + zInitialVelocityRobotRelative*t + .5*Constants.AlignToHubConstants.az*Math.pow(t,2);
  }
  private double sxNew(double t, double theta) { //angle in rad
    return sx(t)*Math.cos((theta)) + sz(t)*Math.sin((theta)); //applies rotation about y-axis by angle theta
  }
  private double szNew(double t, double theta) { //angle in rad
    return sz(t)*Math.cos((theta)) - sx(t)*Math.sin((theta)); //applies rotation about y-axis by angle theta
  }
}
