// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import first.robot.swerve.SwerveModuleConfig;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>Values marked TODO are placeholders that need to be measured on the real robot.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;

    // Stick values smaller than this are treated as zero, so a drifting stick doesn't creep.
    public static final double STICK_DEADBAND = 0.08;
  }

  public static class SwerveConstants {
    // Robot coordinates (WPILib convention): +X is forward, +Y is left, CCW rotation is positive.

    private static final double METERS_PER_INCH = 0.0254;

    // Distance between the centers of the left and right pods.
    public static final double TRACK_WIDTH_METERS = 13.5 * METERS_PER_INCH;
    // Distance between the centers of the front and back pods.
    public static final double WHEEL_BASE_METERS = 13.5 * METERS_PER_INCH;

    public static final double WHEEL_DIAMETER_METERS = 2.0 * METERS_PER_INCH;

    // A301 rotations per one wheel rotation. Measured 1.77 by rolling the front-left wheel one turn
    // by hand and reading "driveMotorRotations". Roll several turns and divide to refine it.
    public static final double DRIVE_GEAR_RATIO = 1.77;

    // A301 rotations per one full pod rotation: 16T pulley on the A301 driving a 44T pulley on the
    // pod gives 44 / 16 = 2.75. Check with the "Pod Test" OpMode: one full turn of the pod should
    // change "steerMotorRotations" by exactly this much.
    public static final double STEER_GEAR_RATIO = 44.0 / 16.0;

    // Top speed we ever command. 5.0 is an assumed top speed, not measured yet. Run the
    // "Free Speed Test" OpMode on blocks and set this a bit below the slowest wheel's peak.
    public static final double MAX_SPEED_METERS_PER_SECOND = 5.0;
    // Top rotation speed we ever command: one full spin per second.
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI;

    public static final OnboardIMU.MountOrientation IMU_MOUNT = OnboardIMU.MountOrientation.FLAT;

    private static final double HALF_TRACK = TRACK_WIDTH_METERS / 2.0;
    private static final double HALF_BASE = WHEEL_BASE_METERS / 2.0;

    // One entry per pod. Set "installed" to false for pods that aren't plugged in yet; they still
    // count toward the chassis geometry but no motors are created for them.
    //
    // steerInverted: all four measured false (one CCW pod turn = +2.70 to +2.75 steer rotations),
    // which matches the controls no longer being mirrored in Swerve Drive. A wrong value is safe:
    // the pod just mirrors its angle instead of running away.
    public static final SwerveModuleConfig[] MODULES = {
      new SwerveModuleConfig(
          "FrontLeft",
          CANPort.CAN_D18,
          CANPort.CAN_D19,
          new Translation2d(HALF_BASE, HALF_TRACK),
          true, // measured: wheel rolled backward when commanded forward
          false,
          true),
      new SwerveModuleConfig(
          "FrontRight",
          CANPort.CAN_D8,
          CANPort.CAN_D9,
          new Translation2d(HALF_BASE, -HALF_TRACK),
          false,
          false,
          true),
      new SwerveModuleConfig(
          "BackLeft",
          CANPort.CAN_D2,
          CANPort.CAN_D3,
          new Translation2d(-HALF_BASE, HALF_TRACK),
          true, // measured: wheel rolled backward when commanded forward
          false,
          true),
      new SwerveModuleConfig(
          "BackRight",
          CANPort.CAN_D6,
          CANPort.CAN_D7,
          new Translation2d(-HALF_BASE, -HALF_TRACK),
          false,
          false,
          true),
    };
  }
}
