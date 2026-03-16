// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/**
 * This command unjams the hopper by rolling the hopper rollers back and forth at a set frequency. It does not end, an end condition must be set when used.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import edu.wpi.first.wpilibj.Timer;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UnjamHopper extends Command {
  private FeederSubsystem m_feeder;
  /** Creates a new UnjamHopper. */
  Timer timer = new Timer();
  public UnjamHopper(FeederSubsystem feeder) {
    m_feeder = feeder;
    addRequirements(m_feeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(timer.get() < Constants.FeederConstants.kfeederrollerunjamspeed)
    {
      m_feeder.setRollerSpeed(Constants.FeederConstants.rollerSpeed);
    } 
    else if(timer.get() < 2*Constants.FeederConstants.kfeederrollerunjamspeed)
    {
      m_feeder.setRollerSpeed(Constants.FeederConstants.rollerSpeed * -1);
    }
    else
    {
      timer.restart();
    }
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
