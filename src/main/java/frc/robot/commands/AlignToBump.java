package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.util.Units;

public class AlignToBump extends Command{//This is the better edition
   // PIDController xController = new PIDController(Constants.AlignConstants.kAlignP, Constants.AlignConstants.kAlignI, Constants.AlignConstants.kAlignD);
  PIDController yController = new PIDController(Constants.AlignConstants.kAlignP, Constants.AlignConstants.kAlignI, Constants.AlignConstants.kAlignD);
  PIDController rotController = new PIDController(Constants.AlignConstants.kRotAlignP, Constants.AlignConstants.kRotAlignI, Constants.AlignConstants.kRotAlignD);
    DriveSubsystem m_drive;
    GenericHID m_controller;
    double robotDiagonalLength = 51;//was 38.3813;
    
    double currentAngle;
    double currentY;
    double currentX;
    double rotSpeed;
    double ySpeed;
    double xSpeed;
    Debouncer m_debouncer;

    int drivingOverTheBumpDirectionMode;
    double bumpSpeed = 1;//This is percentage of the power needed, so it would be (needed speed / max speed). (was 1.7/4.8, but it's actually m/s. oh well.)
    
    double odomError = 0.3;

    double goalAngle = 20;//Angles 38.412, 141.588, -38.412, -141.588 all work for align to bump. But 38.412 is the best for some reason, maybe because of the way the field is set up or the way the robot is built, but it is also possible that there is some error in the code that makes it so that 38.412 works better than the other angles. I will investigate this further in testing.
    boolean autoMode = false;
    public AlignToBump(DriveSubsystem drive) {
        rotController.setSetpoint(0);//This makes the robot face 0 degrees.
        rotController.enableContinuousInput(-180, 180);
        m_drive = drive;
        m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
        this.autoMode = true;
        addRequirements(m_drive);//i removed this when testing it with a button.
    }
    public AlignToBump(DriveSubsystem drive, GenericHID controller){
        this(drive);
        m_controller=controller;
        this.autoMode = false;
    }
     double angle1 = goalAngle;//38.412
    double angle2 = (180-goalAngle);//141.588
    double angle3 = (180+goalAngle);//-38.412
    double angle4 = (360-goalAngle);//-141.588
    @Override
    public void initialize() {
         drivingOverTheBumpDirectionMode = 0;
        if (autoMode){//If autos appears to be smooth and doesn't have a need to realign, then use this.
        drivingOverTheBumpDirectionMode = findDrivingDirection();
       }

        currentAngle = m_drive.getPose().getRotation().getDegrees();
        while (currentAngle > 360 || currentAngle<0) {
            if(currentAngle>360){
            currentAngle -= 360;
            }
            else{
            currentAngle += 360;
            }
        }
         currentY = m_drive.getPose().getY();
        
        if (currentY>Units.inchesToMeters(158.84)){
            yController.setSetpoint(Units.inchesToMeters(218.84));
        } else {
            yController.setSetpoint(Units.inchesToMeters(99.17+3));
        }
        yController.setTolerance(Units.inchesToMeters(2));
        

        //center of field: 325.06 
        //Center of blue bump: 181.56
        //Center of red bump: 468.6

        if (currentAngle>90&&currentAngle<180){
            rotController.setSetpoint(angle2);
        } else if (currentAngle>180&&currentAngle<270){
            rotController.setSetpoint(angle3);
        } else if (currentAngle>270&&currentAngle<360){
            rotController.setSetpoint(angle4);
        } else if (currentAngle>0&&currentAngle<90){
            rotController.setSetpoint(angle1);
        } else {
            //Add a breakpoint here if you want to check if currentAngle is returning weird values.
        }
        SmartDashboard.putNumber("AlignToBump/Target Angle", rotController.getSetpoint());
        SmartDashboard.putNumber("AlignToBump/Target Y", yController.getSetpoint());
            //SmartDashboard.putNumber("AlignToBump/Target X", xController.getSetpoint());
            //SmartDashboard.putData("AlignToBump/xController", xController);
            SmartDashboard.putData("AlignToBump/yController", yController);
            SmartDashboard.putData("AlignToBump/rotController", rotController);
    }
   
    @Override
    public void execute() {
       // SmartDashboard.putNumber("AlignToBump/driveState", drivingOverTheBumpDirectionMode);
        switch (drivingOverTheBumpDirectionMode){
            case 1:
            case 3:
                m_drive.drive(bumpSpeed,0, 0,true); 
            break;
            case 2: case 4:
                m_drive.drive(-bumpSpeed,0, 0,true); 
            break;
            default:
                currentAngle = m_drive.getPose().getRotation().getDegrees()%360;
                rotSpeed = rotController.calculate(currentAngle); 
                currentY = m_drive.getPose().getY();
                ySpeed = yController.calculate(currentY);
                currentX = m_drive.getPose().getX();
                int flipFactor = (DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Red))?-1:1);
                xSpeed = -MathUtil.applyDeadband(
                    Math.pow(
                        m_controller.getRawAxis(
                            Constants.ControllerConstants.MOVE_YAXIS), 2) * Math.signum(m_controller.getRawAxis(Constants.ControllerConstants.MOVE_YAXIS))*flipFactor, OIConstants.kDriveDeadband);
                
                SmartDashboard.putNumber("AlignToBump/Current Angle", currentAngle);
                SmartDashboard.putNumber("AlignToBump/YError", yController.getError());
                SmartDashboard.putNumber("AlignToBump/RotError", rotController.getError());
                SmartDashboard.putNumber("AlignToBump/Rotation Speed", rotSpeed);
                SmartDashboard.putNumber("AlignToBump/X Speed", xSpeed);
                SmartDashboard.putNumber("AlignToBump/Y Speed", ySpeed);
                SmartDashboard.putNumber("AlignToBump/Turn Rate", Math.abs(m_drive.getTurnRate()));
                
                m_drive.drive(
                    xSpeed, ySpeed, rotSpeed, true);
                if (autoMode&&m_debouncer.calculate(Math.abs(rotController.getError())<2*1.5&&Math.abs(m_drive.getTurnRate())<0.018&&yController.atSetpoint())){ //change turn rate to115 1 deg.
                    drivingOverTheBumpDirectionMode = findDrivingDirection();//2, 0.02
                    //drivingOverTheBumpDirectionMode = 50;
                }
        }
    }
    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0,0,0,false);
    }
    @Override 
    public boolean isFinished(){
        switch (drivingOverTheBumpDirectionMode){
            case 1:
                return m_drive.getPose().getX()>4.626+0.5588+odomError;
            case 2:
                return m_drive.getPose().getX()<4.626-0.5588-odomError;
            case 3:
                return m_drive.getPose().getX()>11.915+0.5588+odomError;
            case 4:
                return m_drive.getPose().getX()<11.915-0.5588-odomError;
            case 5:
                return true;
            default:
                return false;
        }
    }
    public int findDrivingDirection(){
        double xPos = m_drive.getPose().getX();//This cannnot be updated in periodic
        if (!autoMode){
            return 0;
        }
        if (xPos<8.256) { //If the robot is on the blue side of the field
                if (xPos<4.626){
                    return 1;
                } else {
                    return 2;
                }
        } else {
            if (xPos<11.915){
                return 3;
            } else {
                return 4;
            }
        }
    }
    
}
