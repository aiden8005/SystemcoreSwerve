// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.swerve;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.geometry.Translation2d;

/**
 * Everything that differs between one swerve pod and another.
 *
 * @param name name used for telemetry, e.g. "FrontLeft"
 * @param drivePort Motioncore port of the A301 that spins the wheel
 * @param steerPort Motioncore port of the A301 that turns the pod
 * @param location pod position relative to the robot center (+X forward, +Y left), in meters
 * @param driveInverted true if the wheel rolls backward when commanded forward
 * @param steerInverted true if turning the pod counterclockwise (seen from above) makes the steer
 *     encoder go negative
 * @param installed false if this pod isn't plugged in yet
 */
public record SwerveModuleConfig(
    String name,
    CANPort drivePort,
    CANPort steerPort,
    Translation2d location,
    boolean driveInverted,
    boolean steerInverted,
    boolean installed) {}
