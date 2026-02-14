// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
public class FieldManagementPublisher extends SubsystemBase {
  /** Creates a new FieldManagementPublisher. */
  public FieldManagementPublisher() {}
BooleanPublisher hubPublisher = NetworkTableInstance.getDefault().getBooleanTopic("HubActive").publish();
DoublePublisher matchTimePublisher = NetworkTableInstance.getDefault().getDoubleTopic("MatchTime").publish();
  @Override
  public void periodic() {
    int hubState = getHubState();
    if(hubState == 1)
    {
      hubPublisher.set(true);
    }
    else if(hubState == 0||hubState == -1)
    {
      hubPublisher.set(false);
    }
    else if(hubState  ==2)
    {
      hubPublisher.set((DriverStation.getMatchTime()*8)%2==0);
    }
    matchTimePublisher.set(DriverStation.getMatchTime());
  }
  /*
   * 1 = Active
   * 0 = inactive
   * -1 = no data
   * 2 = transition
   */
  public int getHubState() {
  Optional<Alliance> alliance = DriverStation.getAlliance();
  // If we have no alliance, we cannot be enabled, therefore no hub.
  if (alliance.isEmpty()) {
    return -1;
  }
  // Hub is always enabled in autonomous.
  if (DriverStation.isAutonomousEnabled()) {
    return 1;
  }
  // At this point, if we're not teleop enabled, there is no hub.
  if (!DriverStation.isTeleopEnabled()) {
    return -1;
  }

  // We're teleop enabled, compute.
  double matchTime = DriverStation.getMatchTime();
  String gameData = DriverStation.getGameSpecificMessage();
  // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
  if (gameData.isEmpty()) {
    return -1;
  }
  boolean redInactiveFirst = false;
  switch (gameData.charAt(0)) {
    case 'R' -> redInactiveFirst = true;
    case 'B' -> redInactiveFirst = false;
    default -> {
      // If we have invalid game data, assume hub is active.
      return 1;
    }
  }

  // Shift was is active for blue if red won auto, or red if blue won auto.
  boolean shift1Active = switch (alliance.get()) {
    case Red -> !redInactiveFirst;
    case Blue -> redInactiveFirst;
  };

  if (matchTime > 130+3) {
    // Transition shift, hub is active.
    return 1;
  } 
  else if(matchTime>130)
  {
  return shift1Active? 1 : 2;
  }
  else if (matchTime > 105+3) {
    // Shift 1
    return shift1Active? 1 : 0;
  } 
  else if(matchTime>105)
  {
    return shift1Active? 2 : 1;
  }
  else if (matchTime > 80+3) {
    // Shift 2
    return shift1Active?0:1;
  }
  else if(matchTime>80)
  {
    return shift1Active? 1 : 2;
  } 
  else if (matchTime > 55+3) {
    // Shift 3
    return shift1Active? 1 : 0;
  } 
  else if(matchTime>55)
  {
    return shift1Active? 2 : 1;
  }
  else if (matchTime > 30) {
    // Shift 4
    return shift1Active?0:1;
  } else {
    // End game, hub always active.
    return 1;
  }
}
}
