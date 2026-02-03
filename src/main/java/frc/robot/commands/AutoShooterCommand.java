// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoShooterCommand extends Command {
  private ShooterSubsystem ShooterSubsystem;
  private double speed;
  /** Creates a new AutoShooterCommand. */
  public AutoShooterCommand(ShooterSubsystem Shooting, double speed) {
    ShooterSubsystem = Shooting;
    this.speed = speed;
    addRequirements(ShooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Sets shooter motors to desired speed
    ShooterSubsystem.reachSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Ends command when all shooter motors are at desired speed within tolerance
    if (Math.abs(speed-ShooterSubsystem.getMotorVelocity()[0])<Constants.OuttakeConstants.ShooterTolerance && Math.abs(speed-ShooterSubsystem.getMotorVelocity()[1])<Constants.OuttakeConstants.ShooterTolerance && Math.abs(speed-ShooterSubsystem.getMotorVelocity()[2])<Constants.OuttakeConstants.ShooterTolerance){
      return true;
    }
    else {
      return false;
    }
  }
}
