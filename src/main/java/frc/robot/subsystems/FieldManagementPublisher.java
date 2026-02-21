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
DoublePublisher currentShiftClockPublisher = NetworkTableInstance.getDefault().getDoubleTopic("CurrentShiftClock").publish();
DoublePublisher timeLeftInTransitionPublisher = NetworkTableInstance.getDefault().getDoubleTopic("timeLeftInTransitionPublisher").publish();
  @Override
  public void periodic() {
    int hubState = getHubState();
    if(hubState == 1)
    {
      hubPublisher.set(true);
    }
    else if(hubState == 0||hubState == -1 ||hubState == 3)
    {
      hubPublisher.set(false);
    }
    else if(hubState  ==2)
    {
      hubPublisher.set(Math.round(DriverStation.getMatchTime()*4)%2==0);
    }
    matchTimePublisher.set(Math.round(10*DriverStation.getMatchTime())/10.0);
    currentShiftClockPublisher.set(Math.round(10*((DriverStation.getMatchTime()-30)%25))/10.0);
    timeLeftInTransitionPublisher.set(timeLeftInTransition());
  }
  /*
   * 1 = Active
   * 0 = inactive
   * -1 = no data
   * 2 = transition (active to inactive)
   * 3 = transition (inactive to active)
   */
  public static int getHubState() { //changed to static to make it easier to use without importing :)
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

    if (matchTime > 130+3) { // Transition shift, first 7 seconds
      // Transition shift, hub is active.
      return 1;
    } 
    else if(matchTime>130) //  Transition shift, but hub might be deactivating
    {
    return shift1Active? 1 : 2;
    }
    else if (matchTime > 105+3) { // First shift, first 22 seconds
      // Shift 1
      return shift1Active? 1 : 0;
    } 
    else if(matchTime>105) // First shift, but hub might be deactivating
    {
      return shift1Active? 2 : 3; // changed from 0 to 3
    }
    else if (matchTime > 80+3) { // second shift, first 22 seconds
      // Shift 2
      return shift1Active?0:1;
    }
    else if(matchTime>80) // second shift, but hub might be deactivating
    {
      return shift1Active? 3 : 2; // changed from 0 to 3
    } 
    else if (matchTime > 55+3) { // third shift, first 22 seconds
      // Shift 3
      return shift1Active? 1 : 0;
    } 
    else if(matchTime>55) // third shift, but hub might be deactivating
    {
      return shift1Active? 2 : 3; //changed from 0 to 3
    }
    else if (matchTime > 30) { // Fourth shift, full thing
      // Shift 4
      return shift1Active?0:1;
    } else {
      // End game, hub always active.
      return 1;
    }
  }
  public static double timeLeftInTransition() {
    double hubState = getHubState();
    if (!((hubState == 2) || (hubState == 3))) {
      return -1;
    }
    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();

    if (matchTime > 130) { // Transition shift, transition
      return 130 + 3 - matchTime;
    } 
    else if(matchTime>105) // First shift, transition
    {
      return 105 + 3 - matchTime;
    }
    else if(matchTime>80) // second shift, transition
    {
      return 80 + 3 - matchTime;
    } 
    else if(matchTime>55) // third shift, transition
    {
      return 55 + 3 - matchTime;
    }
    // Fourth shift, no transition
    else {
      // End game, no transition
      return -1;
    }

  }
}
