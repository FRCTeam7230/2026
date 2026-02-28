// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToPass extends Command {
  PIDController yawController = new PIDController(0.0025, 0, 0); //Need to tune these values
  PIDController yController = new PIDController(0.5, 0, 0);
  DriveSubsystem m_drive;
  /** Creates a new AlignToPass. */
  public AlignToPass(DriveSubsystem drive) {
    m_drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    yawController.setSetpoint(0);
    yawController.setTolerance(1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentY = m_drive.getPose().getY();
    if (currentY<4.035) {
      yController.setSetpoint(2.0175);
      yController.setTolerance(1);
      
    }
    else {
      yController.setSetpoint(6.0525);
      yController.setTolerance(1);
      
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     double currentYaw = m_drive.getPose().getRotation().getDegrees();
     double rotSpeed= Math.max(Math.min(yawController.calculate(currentYaw),1.5), -1.5);
     double currentY = m_drive.getPose().getY();
     double ySpeed = yController.calculate(currentY);
      if(yawController.atSetpoint()) {
        rotSpeed = 0;
     }
      if (yController.atSetpoint()) {
        ySpeed = 0;
      }
     m_drive.drive(0, -ySpeed, -rotSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
}
}
