// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import first.robot.swerve.SwerveDrive;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * Holds the robot's hardware. OpModes in this package (like {@link SwerveTeleop}) are found
 * automatically and show up in the Driver Station's OpMode list.
 */
public class Robot extends OpModeRobot {
  public final SwerveDrive drive = new SwerveDrive();

  // How long robotPeriodic takes, and the time between loops. Both should stay well under 20 ms.
  private final DoublePublisher periodicMsPublisher =
      NetworkTableInstance.getDefault().getDoubleTopic("/Robot/periodicMs").publish();
  private final DoublePublisher loopPeriodMsPublisher =
      NetworkTableInstance.getDefault().getDoubleTopic("/Robot/loopPeriodMs").publish();
  private long lastLoopStartNs = System.nanoTime();

  /** Called once at the beginning of the robot program. */
  public Robot() {}

  /** Runs every loop in every mode, enabled or not. */
  @Override
  public void robotPeriodic() {
    long start = System.nanoTime();
    loopPeriodMsPublisher.set((start - lastLoopStartNs) / 1e6);
    lastLoopStartNs = start;

    drive.periodic();

    periodicMsPublisher.set((System.nanoTime() - start) / 1e6);
  }
}
