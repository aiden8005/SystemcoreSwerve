// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static first.robot.Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;
import static first.robot.Constants.OperatorConstants.STICK_DEADBAND;
import static first.robot.Constants.SwerveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
import static first.robot.Constants.SwerveConstants.MAX_SPEED_METERS_PER_SECOND;

import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.math.util.MathUtil;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

/**
 * Field-centric swerve driving.
 *
 * <ul>
 *   <li>Left stick: move (up = away from the driver)
 *   <li>Right stick X: rotate
 *   <li>Hold left bumper: robot-centric instead (up = robot's front)
 *   <li>Start: the direction the robot faces now becomes "away from the driver"
 * </ul>
 */
@Teleop(name = "Swerve Drive")
public class SwerveTeleop extends PeriodicOpMode {
  private final Robot robot;
  private final Gamepad gamepad = DriverStation.getGamepad(DRIVER_CONTROLLER_PORT);

  public SwerveTeleop(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void periodic() {
    if (gamepad.getStartButtonPressed()) {
      robot.drive.zeroHeading();
    }

    // Stick Y is negative when pushed up and stick X is positive to the right, but the robot uses
    // +X forward, +Y left, CCW positive, so every axis is negated.
    double forward = shape(-gamepad.getLeftY()) * MAX_SPEED_METERS_PER_SECOND;
    double left = shape(-gamepad.getLeftX()) * MAX_SPEED_METERS_PER_SECOND;
    double ccw = shape(-gamepad.getRightX()) * MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

    boolean fieldRelative = !gamepad.getLeftBumperButton();
    robot.drive.drive(forward, left, ccw, fieldRelative);
  }

  @Override
  public void end() {
    robot.drive.stop();
  }

  /** Deadband, then square (keeping the sign) for finer control near the center of the stick. */
  private static double shape(double axis) {
    double value = MathUtil.applyDeadband(axis, STICK_DEADBAND);
    return Math.copySign(value * value, value);
  }
}
