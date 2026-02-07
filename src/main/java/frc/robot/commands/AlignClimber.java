// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignClimber extends Command {
  PIDController xPIDController = new PIDController(Constants.AlignClimberConstants.kxkp, 0, 0);
  PIDController zPIDController = new PIDController(Constants.AlignClimberConstants.kzkp, 0, 0);
  PIDController yawPIDController = new PIDController(Constants.AlignClimberConstants.kyawkp, 0, 0);
  DriveSubsystem driveSubsystem;
  /** Creates a new AlignClimber. */
  public AlignClimber(DriveSubsystem drive) {
    driveSubsystem = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Sets locations where robot should be to align with climber
    xPIDController.setSetpoint(Constants.AlignClimberConstants.kxSetpoint);
    zPIDController.setSetpoint(Constants.AlignClimberConstants.kzSetpoint);
    yawPIDController.setSetpoint(Constants.AlignClimberConstants.kyawSetpoint);
    xPIDController.setTolerance(Constants.AlignClimberConstants.kxTolerance);
    zPIDController.setTolerance(Constants.AlignClimberConstants.kzTolerance);
    yawPIDController.setTolerance(Constants.AlignClimberConstants.kyawTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Gets the error from the AprilTag and calculates PID to reach setpoints
        double[] ErrorfromApTag = LimelightHelpers.getCameraPose_TargetSpace("idk");
    double tx = ErrorfromApTag[0];
    double tz = ErrorfromApTag[2];
    double tyaw = ErrorfromApTag[4];
    double txcalculation = xPIDController.calculate(tx);
    double tzcalculation = zPIDController.calculate(tz);
    double tyawcalculation = yawPIDController.calculate(tyaw);
    driveSubsystem.driveTagRelative(tzcalculation,txcalculation, -0,tyawcalculation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Ends command when at setpoints
    if (xPIDController.atSetpoint() && zPIDController.atSetpoint() && yawPIDController.atSetpoint()){
      return true;
    }
    else {
    return false;
  }
}
}
