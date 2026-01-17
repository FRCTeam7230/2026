// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double movementDivider = 4; // from 2025 code
  public static final double rotateDivider = 5;

  public static class ControllerConstants{
    //Button configurations for the XBox controller
    public static final int kButton1 = 1; //A
    public static final int kButton2 = 2; //B
    public static final int kButton3 = 3; //X
    public static final int kButton4 = 4; //Y
    public static final int kButton5 = 5; //LB, left button
    public static final int kButton6 = 6; //RB, right button
    public static final int kButton7 = 7; //Screenshare button, probably dont use   //not used
    public static final int kButton8 = 8; //Menu button, probably dont use, also the back button
    public static final int kButton9 = 9; //Pressing down left joystick DO NOT USE
    public static final int kButton10 = 10; //Pressing down right joystick DO NOT USE  //not used

    /**
     * Setting the numbers of the povs to literally anything else will probably break everything 
     */
    public static final int pov0 = 0; //up
    public static final int pov45 = 45; //up right
    public static final int pov90 = 90; //right          probably make it elevator intake
    public static final int pov135 = 135; //down right
    public static final int pov180 = 180; //down
    public static final int pov225 = 225; //down left
    public static final int pov270 = 270; //left
    public static final int pov315 = 315; //up left

    public static final int leftTrigger = -2; //LT, left trigger 
    public static final int rightTrigger = -3; //RT, right trigger

    public static final int leftStick_XAXIS = 0;
    public static final int leftStick_YAXIS = 1;
    public static final int rightStick_XAXIS = 4;
    public static final int rightStick_YAXIS = 5;

    // Xbox controller mappings
    public static final int BRAKE_BUTTON = kButton3;
    public static final int INTAKE_BUTTON = kButton9;

    public static final int SCORE_AUTO = pov180; //score right now
    public static final int MANUAL_ELEVATOR_DOWN = pov270;
    
    public static final int ZERO_HEADING_BUTTON = kButton7; 

    public static final int L1_SCORE = kButton6;
    
    public static final int ELEVATOR_INCREMENT_DOWN = kButton1;
    public static final int ELEVATOR_MAXHEIGHT = pov0;
    public static final int L4_INTAKE = kButton4; //connect
    public static final int L1_INTAKE = kButton5; //connect

    //graveyard
    public static final int MANUAL_L1_DOWN = 67;   //dead
    public static final int MANUAL_L1_UP = 67;     //dead
    public static final int L1_STOW = 67;          //dead
    public static final int SLOW_MODE_LEFT = 67;   //dead
    public static final int SLOW_MODE_RIGHT = 67;  //dead
    public static final int SCORE_LEFT = leftTrigger;       //dead
    public static final int SCORE_RIGHT = rightTrigger; //changed to exist


    public static final int ROBOT_RELATIVE = kButton2;
    public static final int HOVER_L1 = pov90;
    //button 7 is not used
    
    // XBox movement mappings
    public static final int MOVE_XAXIS = leftStick_XAXIS;
    public static final int MOVE_YAXIS = leftStick_YAXIS;
    public static final int MOVE_ZAXIS = rightStick_XAXIS;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24); // updated chassis dimensions
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(30.5); // updated chassis dimensions
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    // Original value: public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontLeftChassisAngularOffset = (Math.PI/2) + Math.PI;

    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;



    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 8;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 14;
    public static final int kRearRightDrivingCanId = 6;

    public static final int kFrontLeftTurningCanId = 7;
    public static final int kRearLeftTurningCanId = 9;
    public static final int kFrontRightTurningCanId = 10;
    public static final int kRearRightTurningCanId = 5;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.079;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1; // updated based on 2025 code
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.8; // updated based on 2025 code
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}