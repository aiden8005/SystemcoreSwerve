// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static first.robot.Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;
import static first.robot.Constants.SwerveConstants.MODULES;

import first.robot.swerve.SwerveModule;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

/**
 * Measures the drive wheels' real top speed. ROBOT ON BLOCKS, wheels off the ground.
 *
 * <p>Squeeze the right trigger to run every drive wheel at that much raw output (full trigger =
 * full power) with the pods held where they are. The fastest speed each wheel reaches is published
 * under "FreeSpeedTest". Set MAX_SPEED_METERS_PER_SECOND a bit below the slowest wheel's peak.
 */
@Teleop(name = "Free Speed Test", group = "Testing")
public class FreeSpeedTest extends PeriodicOpMode {
  private final Robot robot;
  private final Gamepad gamepad = DriverStation.getGamepad(DRIVER_CONTROLLER_PORT);
  private final NetworkTable results =
      NetworkTableInstance.getDefault().getTable("FreeSpeedTest");
  private final double[] peakMetersPerSecond = new double[MODULES.length];

  public FreeSpeedTest(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void periodic() {
    double throttle = gamepad.getRightTrigger();
    double slowestPeak = Double.MAX_VALUE;
    for (int i = 0; i < MODULES.length; i++) {
      SwerveModule pod = robot.drive.getModule(i);
      if (pod == null) {
        continue;
      }
      pod.setDriveThrottle(throttle);
      double speed = Math.abs(pod.getDriveVelocityMetersPerSecond());
      peakMetersPerSecond[i] = Math.max(peakMetersPerSecond[i], speed);
      slowestPeak = Math.min(slowestPeak, peakMetersPerSecond[i]);
      results.getEntry(pod.getName() + "PeakMetersPerSecond").setDouble(peakMetersPerSecond[i]);
    }
    results.getEntry("throttle").setDouble(throttle);
    results.getEntry("slowestPeakMetersPerSecond").setDouble(slowestPeak);
  }

  @Override
  public void end() {
    robot.drive.stop();
  }
}
