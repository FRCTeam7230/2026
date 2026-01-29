// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MegaTagPosition extends SubsystemBase {
  /** Creates a new MegaTagPosition. */
  private DriveSubsystem driveSubsystem;
  public MegaTagPosition(DriveSubsystem robopose) {
    driveSubsystem = robopose;
  }

  public Pose2d FetchBotPose() {
    Optional<Alliance> ally = DriverStation.getAlliance();
if (ally.isPresent()) {
    if (ally.get() == Alliance.Red) {
      int[] validIDs = {2,5,8,9,10,11};
      LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
    }
    if (ally.get() == Alliance.Blue) {
      int[] validIDs = {18,21,24,25,26,27};
      LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
    }
}
else {
  int[] validIDs = {2,5,8,9,10,11,18,21,24,25,26,27};
  LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
}

  LimelightHelpers.SetRobotOrientation("limelight",driveSubsystem.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
  LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
   Pose2d pose = mt2.pose;
   return pose;
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}