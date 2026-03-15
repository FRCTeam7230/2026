// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToHub extends Command {
  /** Creates a new AlignToHub. */
  DriveSubsystem m_drive;
  PIDController xController = new PIDController(2, 0, 0.2);
  PIDController yController = new PIDController(2, 0, 0.2);
  PIDController rotController = new PIDController(0.005, 0, 0);
  DoublePublisher distanceErrorPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Align Distance Error").publish(); 
  StructPublisher<Pose2d> targetPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Align target Pose", Pose2d.struct).publish();  

  public AlignToHub(DriveSubsystem drive) {
    m_drive = drive;
    m_drive.ApplyMegatagFilter();
    addRequirements(drive);
    xController.setSetpoint(0);
    yController.setSetpoint(0);
    rotController.setSetpoint(0);
    xController.setTolerance(Constants.AlignConstants.kerrorXTolerance);
    yController.setTolerance(Constants.AlignConstants.kerrorYTolerance);
    rotController.setTolerance(Constants.AlignConstants.kerrorAngleTolerance);
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
    SmartDashboard.putNumberArray("AlignErrors",errors);
    
    double xSpeed = xController.calculate(errors[0]);
    double ySpeed = yController.calculate(errors[1]);
    //double rotSpeed= Math.max(Math.min(rotController.calculate(errors[2]),1.5), -1.5);
    double rotSpeed = rotController.calculate(errors[2]);
    SmartDashboard.putNumber("Rotation delivered", rotSpeed);
    m_drive.drive(-xSpeed, -ySpeed, -rotSpeed,true);
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_drive.setX();
    m_drive.CancelMegatagFilter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(xController.atSetpoint()&&yController.atSetpoint()&&rotController.atSetpoint())
    {
      return false;
    }
    else
    {
      return false;
    }

  }
    public double[] CalculateHubPID(Pose2d pose) {
        double robotX = pose.getX();
		    double robotY = pose.getY();

        double[] errors = new double[3];
        for (int i = 0; i < errors.length; i++) {
          errors[i] = 0;
        }
        double radius = Constants.AlignConstants.kradius;
        double hubY = Constants.AlignConstants.khubY; // meters
        double hubXBlue = Constants.AlignConstants.khubXBlue;
        double hubXRed = Constants.AlignConstants.khubXRed;
        double hubX;
        if (DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue))) {
            hubX = hubXBlue;
        }
        else {
            hubX = hubXRed;
        }

        double distanceX = hubX - robotX;
        double distanceY = hubY - robotY;
        double distance = Math.sqrt( Math.pow( distanceX, 2) + Math.pow( distanceY, 2) );
        double distanceError = radius-distance;
        distanceErrorPublisher.set(distanceError);
        double errorX = distanceX * ( (distance - radius) / distance );
        double errorY = distanceY * ( (distance - radius) / distance );
        double targetAngle = Math.signum(distanceY) * Math.acos(distanceX / distance)*180/Math.PI;
        double errorAngle = targetAngle - pose.getRotation().getDegrees();
        Pose2d targetPose = pose.plus(new Transform2d(errorX,errorY,new Rotation2d(Math.toRadians(errorAngle))));
        targetPosePublisher.set(targetPose);
        if (Math.abs(errorX) < Constants.AlignConstants.kerrorXTolerance) {
      errors[0] = 0;
    }
    else {errors[0] = errorX;}
    if (Math.abs(errorY) < Constants.AlignConstants.kerrorYTolerance) {
      errors[1] = 0;
    }
    else {errors[1] = errorY;}
    if (Math.abs(errorAngle) < Constants.AlignConstants.kerrorAngleTolerance) {
      errors[2] = 0;
    }
    else {errors[2] = errorAngle;}
       SmartDashboard.putNumber("AlignToHub/TargetAngle",targetAngle);

        SmartDashboard.putNumber("AlignToHub/ErrorX", errors[0]);
        SmartDashboard.putNumber("AlignToHub/ErrorY", errors[1]);
        SmartDashboard.putNumber("AlignToHub/ErrorAngle", errorAngle);

       SmartDashboard.putNumber("AlignToHub/RobotX", robotX);
        SmartDashboard.putNumber("AlignToHub/RobotY", robotY);
        SmartDashboard.putNumber("AlignToHub/RobotAngle", pose.getRotation().getDegrees());
        return errors;
    }
}
