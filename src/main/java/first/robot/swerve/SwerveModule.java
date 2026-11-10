// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.swerve;

import static first.robot.Constants.SwerveConstants.DRIVE_GEAR_RATIO;
import static first.robot.Constants.SwerveConstants.STEER_GEAR_RATIO;
import static first.robot.Constants.SwerveConstants.WHEEL_DIAMETER_METERS;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.A301;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.util.MathUtil;
import org.wpilib.networktables.NetworkTable;

/**
 * One swerve pod: an A301 that spins the wheel and an A301 that turns the pod.
 *
 * <p>Steering uses the steer A301's multi-turn (relative) encoder divided by {@link
 * first.robot.Constants.SwerveConstants#STEER_GEAR_RATIO}. Because the pod is geared, that encoder
 * doesn't know which way the wheel points at power-up, so "straight ahead" has to be set with
 * {@link #zeroSteering()} (see the "Zero Steering" utility OpMode).
 */
public class SwerveModule {
  // Below this wheel speed the pod holds its current angle instead of snapping back to 0.
  private static final double MIN_STEER_SPEED_METERS_PER_SECOND = 0.01;

  private static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;

  private final SwerveModuleConfig config;
  private final A301 driveMotor;
  private final A301 steerMotor;

  // +1, or -1 if positive steer motor rotation turns the pod clockwise (seen from above).
  private final double steerSign;
  // +1, or -1 if positive drive motor rotation rolls the wheel backward.
  private final double driveSign;

  private SwerveModuleVelocity target = new SwerveModuleVelocity();

  // Last steering setpoint in pod rotations (unwrapped, CCW positive). Re-seeded from the measured
  // angle whenever it's invalid, e.g. after the motors were stopped or the steering was zeroed.
  private double steerSetpointPodRotations = 0.0;
  private boolean steerSetpointValid = false;

  // Result of the most recent command to each motor, for telemetry.
  private REVLibError lastDriveResult = REVLibError.kOk;
  private REVLibError lastSteerResult = REVLibError.kOk;

  public SwerveModule(SwerveModuleConfig config) {
    this.config = config;
    // CAN ID is autodetected, so only the Motioncore port is needed.
    driveMotor = new A301(config.drivePort());
    steerMotor = new A301(config.steerPort());

    // Directions are flipped in software (steerSign/driveSign) instead of with setInverted, so the
    // encoder we read and the setpoint we command always agree with the A301's onboard loop.
    steerSign = config.steerInverted() ? -1.0 : 1.0;
    driveSign = config.driveInverted() ? -1.0 : 1.0;
    driveMotor.setIdleMode(A301.IdleMode.kBrake);
    steerMotor.setIdleMode(A301.IdleMode.kBrake);
  }

  public String getName() {
    return config.name();
  }

  /** Raw steer A301 position in its own rotations (multi-turn), before steerSign is applied. */
  public double getSteerMotorRotations() {
    return steerMotor.getRelativeEncoderPosition().get();
  }

  /** Pod rotations since zeroing, CCW positive, not wrapped. */
  private double getPodRotations() {
    return steerSign * getSteerMotorRotations() / STEER_GEAR_RATIO;
  }

  /** Which way the wheel is pointing, where 0 is straight ahead and CCW is positive. */
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(getPodRotations());
  }

  /** Wheel ground speed in meters per second. */
  public double getDriveVelocityMetersPerSecond() {
    double motorRpm = driveSign * driveMotor.getEncoderVelocity().get();
    return motorRpm / 60.0 / DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE_METERS;
  }

  /** Total distance the wheel has rolled, in meters. */
  public double getDriveDistanceMeters() {
    double motorRotations = driveSign * driveMotor.getRelativeEncoderPosition().get();
    return motorRotations / DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE_METERS;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveDistanceMeters(), getAngle());
  }

  public SwerveModuleVelocity getVelocity() {
    return new SwerveModuleVelocity(getDriveVelocityMetersPerSecond(), getAngle());
  }

  /** The last target passed to {@link #setTarget}, after optimization. */
  public SwerveModuleVelocity getTarget() {
    return target;
  }

  /**
   * Drives the pod toward a wheel speed and direction.
   *
   * @param desired wheel speed (m/s) and wheel angle
   */
  public void setTarget(SwerveModuleVelocity desired) {
    if (!steerSetpointValid) {
      // First command since enable/zeroing: start the setpoint from where the pod actually is.
      steerSetpointPodRotations = getPodRotations();
      steerSetpointValid = true;
    }

    if (Math.abs(desired.velocity) < MIN_STEER_SPEED_METERS_PER_SECOND) {
      // Nothing to do: stop the wheel but keep pointing the same way.
      target = new SwerveModuleVelocity(0.0, target.angle);
      lastDriveResult = driveMotor.setVelocity(0.0);
      return;
    }

    // Never turn more than 90 degrees; reverse the wheel instead. This is decided against the last
    // commanded direction, not the measured one: a pod that lags behind (friction, late encoder
    // data) would otherwise sit near the 90 degree boundary and flip its target 180 degrees back
    // and forth every loop.
    Rotation2d commanded = Rotation2d.fromRotations(steerSetpointPodRotations);
    target = desired.optimize(commanded);

    // Move the setpoint to the equivalent angle closest to the old one, so it never unwinds a full
    // turn just because the target crossed +/-180 degrees.
    steerSetpointPodRotations +=
        MathUtil.inputModulus(
            target.angle.getRotations() - steerSetpointPodRotations, -0.5, 0.5);
    lastSteerResult =
        steerMotor.setRelativePosition(steerSign * steerSetpointPodRotations * STEER_GEAR_RATIO);

    // Slow the wheel down while the pod is still turning toward the target, using the real angle,
    // so it doesn't push sideways.
    target = target.cosineScale(getAngle());
    double wheelRps = target.velocity / WHEEL_CIRCUMFERENCE_METERS;
    lastDriveResult = driveMotor.setVelocity(driveSign * wheelRps * DRIVE_GEAR_RATIO * 60.0);
  }

  /**
   * Runs the drive motor at a raw output with no speed control, holding the pod's current
   * direction. For testing only, e.g. measuring top speed.
   *
   * @param throttle -1.0 to 1.0
   */
  public void setDriveThrottle(double throttle) {
    if (!steerSetpointValid) {
      steerSetpointPodRotations = getPodRotations();
      steerSetpointValid = true;
    }
    lastSteerResult =
        steerMotor.setRelativePosition(steerSign * steerSetpointPodRotations * STEER_GEAR_RATIO);
    driveMotor.setThrottle(driveSign * throttle);
  }

  /** Stops both motors. */
  public void stop() {
    target = new SwerveModuleVelocity(0.0, target.angle);
    steerSetpointValid = false;
    driveMotor.setThrottle(0.0);
    steerMotor.setThrottle(0.0);
  }

  /** Declares the pod's current direction to be "straight ahead". */
  public void zeroSteering() {
    steerMotor.setRelativeEncoderPosition(0.0);
    target = new SwerveModuleVelocity(0.0, new Rotation2d());
    steerSetpointValid = false;
  }

  /** Publishes this pod's state for debugging. */
  public void publishTelemetry(NetworkTable table) {
    NetworkTable t = table.getSubTable(config.name());
    t.getEntry("angleDegrees").setDouble(getAngle().getDegrees());
    t.getEntry("targetAngleDegrees").setDouble(target.angle.getDegrees());
    t.getEntry("steerMotorRotations").setDouble(getSteerMotorRotations());
    t.getEntry("speedMetersPerSecond").setDouble(getDriveVelocityMetersPerSecond());
    t.getEntry("targetSpeedMetersPerSecond").setDouble(target.velocity);
    t.getEntry("distanceMeters").setDouble(getDriveDistanceMeters());
    t.getEntry("driveMotorRotations").setDouble(driveMotor.getRelativeEncoderPosition().get());
    t.getEntry("drive/lastCommandResult").setString(String.valueOf(lastDriveResult));
    t.getEntry("steer/lastCommandResult").setString(String.valueOf(lastSteerResult));
    publishMotorHealth(t.getSubTable("drive"), driveMotor);
    publishMotorHealth(t.getSubTable("steer"), steerMotor);
    // "Fresh" = a status reading arrived recently. Flickering is OK; stuck false means no data.
    t.getEntry("driveStatusFresh").setBoolean(driveMotor.getBusVoltage().isValid());
    t.getEntry("steerStatusFresh").setBoolean(steerMotor.getBusVoltage().isValid());
    t.getEntry("steerEncoderFresh").setBoolean(steerMotor.getRelativeEncoderPosition().isValid());
  }

  /** Publishes the readings needed to tell a healthy A301 from one that is ignoring commands. */
  // Only uses cached status-frame getters, which take ~0.01 ms each. Don't add
  // getFirmwareVersion() here: it waits on a CAN round trip (~134 ms per call), and calling it
  // every loop for 8 motors made the robot loop take over a second.
  private static void publishMotorHealth(NetworkTable t, A301 motor) {
    t.getEntry("busVoltage").setDouble(motor.getBusVoltage().get());
    t.getEntry("appliedOutput").setDouble(motor.getAppliedOutput().get());
    t.getEntry("motorCurrent").setDouble(motor.getMotorCurrent().get());
    t.getEntry("temperature").setDouble(motor.getMotorTemperature().get());
    t.getEntry("faults").setString(describeFaults(motor.getFaults().get()));
    t.getEntry("stickyFaults").setString(describeFaults(motor.getStickyFaults().get()));
    t.getEntry("warnings").setString(describeWarnings(motor.getWarnings().get()));
  }

  private static String describeFaults(A301.Faults f) {
    if (f == null) {
      return "none received";
    }
    StringBuilder s = new StringBuilder();
    if (f.other) s.append("other ");
    if (f.motorType) s.append("motorType ");
    if (f.sensor) s.append("sensor ");
    if (f.can) s.append("can ");
    if (f.temperature) s.append("temperature ");
    if (f.gateDriver) s.append("gateDriver ");
    if (f.escEeprom) s.append("escEeprom ");
    if (f.firmware) s.append("firmware ");
    return s.length() == 0 ? "ok" : s.toString().trim();
  }

  private static String describeWarnings(A301.Warnings w) {
    if (w == null) {
      return "none received";
    }
    StringBuilder s = new StringBuilder();
    if (w.brownout) s.append("brownout ");
    if (w.overcurrent) s.append("overcurrent ");
    if (w.escEeprom) s.append("escEeprom ");
    if (w.extEeprom) s.append("extEeprom ");
    if (w.sensor) s.append("sensor ");
    if (w.stall) s.append("stall ");
    if (w.hasReset) s.append("hasReset ");
    if (w.other) s.append("other ");
    return s.length() == 0 ? "ok" : s.toString().trim();
  }
}
