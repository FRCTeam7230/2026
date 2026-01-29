// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.*;
//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.ButtonMappings;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.net.PortForwarder;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  DriveSubsystem m_robotDrive;
  private Boolean fieldRelative = true;

  // XBox controller.
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  BooleanPublisher mode_publisher = NetworkTableInstance.getDefault().getBooleanTopic("Is Field Relative").publish();

  private final SendableChooser<Command> autoChooser;

  /**
   * 
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private boolean isCompetition = true;//What replaces this?

  public RobotContainer() {
    
    //Set up Subsystems
    if (RobotBase.isReal()) {
      m_robotDrive = new DriveSubsystem();
    }
    for(int port = 5800; port<=5809; port++)
    {
      PortForwarder.add(port, "limelight.local",port);
    }


    // Zero/Reset sensors
    m_robotDrive.zeroHeading();
    m_robotDrive.addAngleGyro(180);
    
    // Configure the button bindings
    configureButtonBindings();

    // TODO: This needs to wait until alliance specified! (Trigger?)
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> isCompetition
            ? stream.filter(auto -> auto.getName().startsWith("COMP"))
            : stream
        );

    //autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(Math.pow(m_driverController.getRawAxis(Constants.ControllerConstants.MOVE_YAXIS), 2) * Math.signum(m_driverController.getRawAxis(Constants.ControllerConstants.MOVE_YAXIS)), OIConstants.kDriveDeadband), //Y
                -MathUtil.applyDeadband(Math.pow(m_driverController.getRawAxis(Constants.ControllerConstants.MOVE_XAXIS), 2) * Math.signum(m_driverController.getRawAxis(Constants.ControllerConstants.MOVE_XAXIS)), OIConstants.kDriveDeadband), //X
                -MathUtil.applyDeadband(Math.pow(m_driverController.getRawAxis(Constants.ControllerConstants.MOVE_ZAXIS), 2) * Math.signum(m_driverController.getRawAxis(Constants.ControllerConstants.MOVE_ZAXIS)), OIConstants.kDriveDeadband), //Z
                fieldRelative),
        m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {


    ////// DRIVER CONTROLLER /////
    ButtonMappings.button(m_driverController, Constants.ControllerConstants.BRAKE_BUTTON)
      .whileTrue(new RunCommand(
          () -> m_robotDrive.setX(),
          m_robotDrive));

    ButtonMappings.button(m_driverController,Constants.ControllerConstants.ZERO_HEADING_BUTTON)
      .whileTrue(new RunCommand(
          () -> m_robotDrive.zeroHeading()));

    ButtonMappings.button(m_driverController, Constants.ControllerConstants.ROBOT_RELATIVE)
      .onTrue(Commands.sequence(
              new InstantCommand(() -> fieldRelative = !fieldRelative, m_robotDrive),
              new InstantCommand(() -> mode_publisher.set(fieldRelative))
          ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // Autos auto = new Autos(m_robotDriveSim);
    // return auto.getAutonomousCommand();
    // Create config for trajectory
    /*
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive); 

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    //return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    */
  }
}