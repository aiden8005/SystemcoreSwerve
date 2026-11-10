// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

/**
 * Sets "straight ahead" for every pod. Point all wheels forward by hand (bevel gears facing the
 * same side), then select this OpMode and enable. The motors don't move.
 */
@Teleop(name = "Zero Steering", group = "Testing")
public class ZeroSteering extends PeriodicOpMode {
  private final Robot robot;

  public ZeroSteering(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void start() {
    robot.drive.stop();
    robot.drive.zeroSteering();
    robot.drive.zeroHeading();
  }
}
