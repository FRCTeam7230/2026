// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MegaTagPosition extends SubsystemBase {
  /** Creates a new MegaTagPosition. */
  public MegaTagPosition() {}
  LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
  LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
   
  // if our angular velocity is greater than 360 degrees per second, ignore vision updates
  if(Math.abs(m_gyro.getRate()) > 360)
  {
    doRejectUpdate = true;
  }
  if(mt2.tagCount == 0)
  {
    doRejectUpdate = true;
  }
  if(!doRejectUpdate)
  {
    m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
    m_poseEstimator.addVisionMeasurement(
        mt2.pose,
        mt2.timestampSeconds);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
