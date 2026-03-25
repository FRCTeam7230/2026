package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;

public class AutoScoring extends Command {
    Timer t = new Timer();
    Timer drive_timer = new Timer();
    FeederSubsystem m_feeder;
    DriveSubsystem m_drive;
    public AutoScoring(DriveSubsystem drive){
            //m_feeder = feeder;
            m_drive = drive;
    }
    @Override
    public void initialize(){
        t.reset();
        t.start();
        drive_timer.restart();
        m_feeder.setKickerSpeed(Constants.FeederConstants.kickerSpeed);
    }
    @Override
    public void execute(){
        // if (t.hasElapsed(1.5)){
        //     t.restart();
        // } else if (t.hasElapsed(1)){
        //     //m_feeder.setKickerSpeed(-Constants.FeederConstants.kickerSpeed);
        //     m_feeder.setRollerSpeed(0);
        // } else {
        //     // m_feeder.setKickerSpeed(Constants.FeederConstants.kickerSpeed);
        //     m_feeder.setRollerSpeed(Constants.FeederConstants.rollerSpeed);
        // }
        if (drive_timer.hasElapsed(1)){
            drive_timer.restart();
        } else if (drive_timer.hasElapsed(0.25)&&!drive_timer.hasElapsed(0.75)){
            m_drive.drive(0,-0.1,0,false);//add a y value to move it sideways.
        } else {
             m_drive.drive(0,0.1,0,false);
        }
    }
    @Override
    public boolean isFinished(){
        return false;
    }
    @Override
    public void end(boolean isFinished){
        m_feeder.setKickerSpeed(0);
        m_feeder.setRollerSpeed(0);
        
    }
}
