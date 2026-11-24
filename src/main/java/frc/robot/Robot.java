// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private enum ArmState {
    GRABBING(80, 0.5, 20),
    STANDBY(0, 0, 0),
    PLACING(10, 2, 90);

    public final double pivotAngleDeg;
    public final double elevatorHeightM;
    public final double wristPitchDeg;

    private ArmState(double pivotDegrees, double elevatorMeters, double wristDegrees) {
      this.pivotAngleDeg = pivotDegrees;
      this.elevatorHeightM = elevatorMeters;
      this.wristPitchDeg = wristDegrees;
    }
  }

  /** The arm state we are at or are changing from. */
  private ArmState previousArmState = ArmState.STANDBY;
  /** The arm state we are switching to. */
  private ArmState targetArmState = ArmState.STANDBY;

  private XboxController driverController = new XboxController(0);

  /* We'd define the subsystems (pivot, elevator, wrist) here as well */

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drivetrain.driveManual(driverController.getLeftY(), driverController.getRightY());

    if (driverController.getAButtonPressed()) {
      targetArmState = ArmState.PLACING;
    } else if (driverController.getBButtonPressed()) {
      targetArmState = ArmState.STANDBY;
    } else if (driverController.getYButtonPressed()) {
      targetArmState = ArmState.PLACING;
    }

    moveArm();
  }

  /** Run once per periodic loop. Moves the arm to {@link #targetArmState}. */
  private void moveArm() {
    if (previousArmState != targetArmState) {
      if (
        // If we're moving the arm from standby to an outward pose
        previousArmState == ArmState.STANDBY && targetArmState == ArmState.GRABBING
        ||
        previousArmState == ArmState.STANDBY && targetArmState == ArmState.PLACING
      ) {
        moveArmOutToState(targetArmState);
      } else if (
        // If we're moving the arm from an outward pose to standby
        previousArmState == ArmState.GRABBING && targetArmState == ArmState.STANDBY
        ||
        previousArmState == ArmState.PLACING && targetArmState == ArmState.STANDBY
        // If we're moving the arm from an outward pose to another outward pose,
        // we need to move to standby in between.
        ||
        previousArmState == ArmState.PLACING && targetArmState == ArmState.GRABBING
        ||
        previousArmState == ArmState.GRABBING && targetArmState == ArmState.PLACING
      ) {
        moveArmInToState(ArmState.STANDBY);
      }
    }
  }

  /**
   * Moves the arm to the given state in the order of pivot, elevator, wrist
   * @return Whether or not we've reached the given state.
   */
  private boolean moveArmOutToState(ArmState state) {
    pivot.setTarget(state.pivotAngleDeg);
    if (pivot.withinTolerance()) {
      elevator.setTarget(state.elevatorHeightM);
      if (elevator.withinTolerance()) {
        wrist.setTarget(state.wristPitchDeg);
        if (wrist.withinTolerance()) {
          previousArmState = state;
          return true;
        };
      }
    }
    
    return false;
  }

  /**
   * Moves the arm to the given state in the order of wrist, elevator, pivot.
   * @return Whether or not we've reached the given state.
   */
  private boolean moveArmInToState(ArmState state) {
    wrist.setTarget(state.wristPitchDeg);
    if (wrist.withinTolerance()) {
      elevator.setTarget(state.elevatorHeightM);
      if (elevator.withinTolerance()) {
        pivot.setTarget(state.pivotAngleDeg);
        if (pivot.withinTolerance()) {
          previousArmState = state;
          return true;
        }
      }
    }

    return false;
  }
}
