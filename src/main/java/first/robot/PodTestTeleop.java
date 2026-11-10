// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static first.robot.Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;
import static first.robot.Constants.SwerveConstants.MAX_SPEED_METERS_PER_SECOND;

import first.robot.swerve.SwerveModule;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

/**
 * Drives one pod directly, for checking wiring, directions and gear ratios.
 *
 * <ul>
 *   <li>Left stick: point the pod in the stick's direction (up = straight ahead)
 *   <li>Right trigger: wheel speed
 *   <li>Face buttons choose which pod (down = FrontLeft, right = FrontRight, left = BackLeft, up =
 *       BackRight)
 * </ul>
 */
@Teleop(name = "Pod Test", group = "Testing")
public class PodTestTeleop extends PeriodicOpMode {
  // The stick has to be pushed at least this far before the pod follows it.
  private static final double AIM_THRESHOLD = 0.5;

  private final Robot robot;
  private final Gamepad gamepad = DriverStation.getGamepad(DRIVER_CONTROLLER_PORT);

  private int podIndex = 0;
  private Rotation2d aim = new Rotation2d();

  public PodTestTeleop(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void periodic() {
    selectPod();
    SwerveModule pod = robot.drive.getModule(podIndex);
    if (pod == null) {
      return;
    }

    double up = -gamepad.getLeftY();
    double left = -gamepad.getLeftX();
    if (Math.hypot(up, left) > AIM_THRESHOLD) {
      aim = new Rotation2d(up, left);
    }

    // A tiny minimum speed keeps the pod steering toward "aim" even with the trigger released.
    double speed = Math.max(gamepad.getRightTrigger() * MAX_SPEED_METERS_PER_SECOND, 0.02);
    pod.setTarget(new SwerveModuleVelocity(speed, aim));
  }

  @Override
  public void end() {
    robot.drive.stop();
  }

  private void selectPod() {
    int requested = podIndex;
    if (gamepad.getFaceDownButtonPressed()) {
      requested = 0;
    } else if (gamepad.getFaceRightButtonPressed()) {
      requested = 1;
    } else if (gamepad.getFaceLeftButtonPressed()) {
      requested = 2;
    } else if (gamepad.getFaceUpButtonPressed()) {
      requested = 3;
    }
    if (requested != podIndex) {
      robot.drive.stop();
      podIndex = requested;
    }
  }
}
