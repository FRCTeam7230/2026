// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignToHub extends Command {
  /** Creates a new AlignToHub. */
  DriveSubsystem m_drive;
  PIDController xController = new PIDController(0.5, 0, 0);
  PIDController yController = new PIDController(0.5, 0, 0);
  PIDController rotController = new PIDController(0.015, 0, 0);


  public AutoAlignToHub(DriveSubsystem drive, Pose2d targetPose) {
    m_drive = drive;
    addRequirements(drive);
    xController.setSetpoint(targetPose.getX());
    yController.setSetpoint(targetPose.getY());
    rotController.setSetpoint(targetPose.getRotation().getDegrees());
    xController.setTolerance(Constants.AutoAlignToHubConstants.kxTolerance);
    yController.setTolerance(Constants.AutoAlignToHubConstants.kyTolerance);
    rotController.setTolerance(Constants.AutoAlignToHubConstants.krTolerance);
    rotController.enableContinuousInput(-180, 180);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.updateAllianceOdometry();
    
    
    Pose2d currentPose = m_drive.getPose();

    
    double xSpeed = xController.calculate(currentPose.getX());
    double ySpeed = yController.calculate(currentPose.getY());
    double rotSpeed= Math.max(Math.min(rotController.calculate(currentPose.getRotation().getDegrees()),1.5), -1.5);
    SmartDashboard.putNumber("Rotation delivered", rotSpeed);
    m_drive.drive(-xSpeed, -ySpeed, -rotSpeed,true);
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_drive.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(xController.atSetpoint()&&yController.atSetpoint()&&rotController.atSetpoint())
    {
      return true;
    }
    else
    {
      return false;
    }

  }
}
