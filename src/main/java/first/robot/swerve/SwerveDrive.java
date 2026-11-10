// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.swerve;

import static first.robot.Constants.SwerveConstants.IMU_MOUNT;
import static first.robot.Constants.SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
import static first.robot.Constants.SwerveConstants.MODULES;

import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveDriveOdometry;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;

/**
 * The whole swerve chassis: pods, onboard IMU, kinematics and odometry.
 *
 * <p>Pods listed in {@link first.robot.Constants.SwerveConstants#MODULES} with {@code installed =
 * false} still count toward the chassis geometry but have no motors, so the math is the same with
 * one pod connected as with four. Odometry is only meaningful once all four pods are installed.
 */
public class SwerveDrive {
  private static final double LOOP_PERIOD_SECONDS = 0.02;

  // null for pods that aren't installed yet.
  private final SwerveModule[] modules = new SwerveModule[MODULES.length];
  private final SwerveDriveKinematics kinematics;
  // Distance from the robot center to the farthest pod; wheel speed from rotation = omega * this.
  private final double maxModuleRadiusMeters;
  private final SwerveDriveOdometry odometry;
  private final OnboardIMU imu = new OnboardIMU(IMU_MOUNT);

  private final NetworkTable telemetry = NetworkTableInstance.getDefault().getTable("Swerve");
  private final StructArrayPublisher<SwerveModuleVelocity> measuredPublisher =
      telemetry.getStructArrayTopic("MeasuredStates", SwerveModuleVelocity.struct).publish();
  private final StructArrayPublisher<SwerveModuleVelocity> targetPublisher =
      telemetry.getStructArrayTopic("TargetStates", SwerveModuleVelocity.struct).publish();
  private final StructPublisher<Pose2d> posePublisher =
      telemetry.getStructTopic("Pose", Pose2d.struct).publish();

  public SwerveDrive() {
    Translation2d[] locations = new Translation2d[MODULES.length];
    double maxRadius = 0.0;
    for (int i = 0; i < MODULES.length; i++) {
      locations[i] = MODULES[i].location();
      maxRadius = Math.max(maxRadius, locations[i].getNorm());
      if (MODULES[i].installed()) {
        modules[i] = new SwerveModule(MODULES[i]);
      }
    }
    maxModuleRadiusMeters = maxRadius;
    kinematics = new SwerveDriveKinematics(locations);
    odometry = new SwerveDriveOdometry(kinematics, getHeading(), getModulePositions());
  }

  /**
   * Drives the chassis.
   *
   * @param vx forward speed in m/s
   * @param vy left speed in m/s
   * @param omega counterclockwise rotation speed in rad/s
   * @param fieldRelative true if vx/vy are relative to the field (away from the driver) instead of
   *     the robot's front
   */
  public void drive(double vx, double vy, double omega, boolean fieldRelative) {
    // Give rotation priority: reserve the wheel speed the rotation needs, and shrink translation to
    // fit in what's left. Otherwise, with both sticks pushed, every wheel is asked for more than
    // it can do, they all max out at the same speed, and the robot drives straight instead of
    // turning.
    double rotationWheelSpeed = Math.abs(omega) * maxModuleRadiusMeters;
    double translationLimit = Math.max(0.0, MAX_SPEED_METERS_PER_SECOND - rotationWheelSpeed);
    double translationSpeed = Math.hypot(vx, vy);
    if (translationSpeed > translationLimit) {
      double scale = translationLimit / translationSpeed;
      vx *= scale;
      vy *= scale;
    }

    ChassisVelocities velocities = new ChassisVelocities(vx, vy, omega);
    if (fieldRelative) {
      velocities = velocities.toRobotRelative(getHeading());
    }
    velocities = velocities.discretize(LOOP_PERIOD_SECONDS);
    setModuleTargets(kinematics.toSwerveModuleVelocities(velocities));
  }

  /** Sends each installed pod its target, scaling everything down if any pod would be too fast. */
  public void setModuleTargets(SwerveModuleVelocity[] targets) {
    targets = SwerveDriveKinematics.desaturateWheelVelocities(targets, MAX_SPEED_METERS_PER_SECOND);
    for (int i = 0; i < modules.length; i++) {
      if (modules[i] != null) {
        modules[i].setTarget(targets[i]);
      }
    }
  }

  public void stop() {
    for (SwerveModule module : modules) {
      if (module != null) {
        module.stop();
      }
    }
  }

  /** Declares every pod's current direction to be "straight ahead". */
  public void zeroSteering() {
    for (SwerveModule module : modules) {
      if (module != null) {
        module.zeroSteering();
      }
    }
  }

  /** Makes the direction the robot is currently facing the new field "forward". */
  public void zeroHeading() {
    imu.resetYaw();
    odometry.resetPosition(getHeading(), getModulePositions(), getPose());
  }

  /** Robot heading from the onboard IMU, CCW positive. */
  public Rotation2d getHeading() {
    return imu.getRotation2d();
  }

  public Pose2d getPose() {
    return odometry.getPose();
  }

  /**
   * The pod at an index in {@link first.robot.Constants.SwerveConstants#MODULES}, or null if it
   * isn't installed.
   */
  public SwerveModule getModule(int index) {
    return modules[index];
  }

  /** Call once per robot loop, in every mode. */
  public void periodic() {
    odometry.update(getHeading(), getModulePositions());
    publishTelemetry();
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i] != null ? modules[i].getPosition() : new SwerveModulePosition();
    }
    return positions;
  }

  private void publishTelemetry() {
    SwerveModuleVelocity[] measured = new SwerveModuleVelocity[modules.length];
    SwerveModuleVelocity[] targets = new SwerveModuleVelocity[modules.length];
    for (int i = 0; i < modules.length; i++) {
      if (modules[i] != null) {
        measured[i] = modules[i].getVelocity();
        targets[i] = modules[i].getTarget();
        modules[i].publishTelemetry(telemetry);
      } else {
        measured[i] = new SwerveModuleVelocity();
        targets[i] = new SwerveModuleVelocity();
      }
    }
    measuredPublisher.set(measured);
    targetPublisher.set(targets);
    posePublisher.set(getPose());
    telemetry.getEntry("headingDegrees").setDouble(getHeading().getDegrees());
  }
}
