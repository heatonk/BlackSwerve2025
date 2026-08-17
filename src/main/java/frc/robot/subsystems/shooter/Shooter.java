package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Shooter subsystem: two flywheels that launch the ball, plus a feeder that pushes the ball into
 * the spinning flywheels. The actual hardware lives behind {@link ShooterIO}, so we can swap in a
 * real Spark implementation, a physics sim, or a replay log without changing this class — same
 * pattern as Drive uses for its swerve modules.
 *
 * <p>Architecture is goal/state-based: commands set a goal (an enum value), and {@link #periodic}
 * is the single place that translates goals into voltages and writes to hardware. Nothing outside
 * this class can poke the motors directly.
 */
public class Shooter extends SubsystemBase {
  // === Tunable voltage setpoints. Single source of truth for shooter speeds —
  // change them here, not in RobotContainer.
  private static final double SHOOT_VOLTS = 10.0;
  private static final double FEED_VOLTS = 6.0;
  private static final double EJECT_VOLTS = -4.0;

  // === RPM safety cap. NEO free speed at 12V is ~5676 RPM, so this sits just
  // below it as a fault-case net. If a flywheel reaches this speed and the active
  // goal would push it further in that direction, we zero the voltage; braking
  // (opposite-sign) voltage is always allowed so a runaway wheel can be slowed.
  private static final double MAX_FLYWHEEL_RPM = 5500.0;
  private static final double MAX_FLYWHEEL_RAD_PER_SEC = MAX_FLYWHEEL_RPM * 2.0 * Math.PI / 60.0;

  /** Discrete operating modes for the flywheels. */
  public enum FlywheelGoal {
    STOPPED,
    SHOOT
  }

  /** Discrete operating modes for the feeder. */
  public enum FeederGoal {
    STOPPED,
    FEED,
    EJECT
  }

  // The IO layer (real, sim, or replay) is injected by RobotContainer.
  private final ShooterIO io;

  // AdvantageKit-generated; periodic() fills it from io.updateInputs and logs it.
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // One alert per motor so the dashboard tells us exactly which Spark dropped off.
  private final Alert[] flywheelDisconnectedAlerts =
      new Alert[] {
        new Alert("Shooter flywheel motor 0 disconnected.", AlertType.kError),
        new Alert("Shooter flywheel motor 1 disconnected.", AlertType.kError),
      };
  private final Alert feederDisconnectedAlert =
      new Alert("Shooter feeder motor disconnected.", AlertType.kError);

  // The current goals. Commands mutate these via their start/end actions; periodic()
  // reads them every loop and drives the hardware. Two independent fields because
  // the flywheels and feeder are independent actuators (driver's two-stage shoot
  // requires holding "shoot" and "feed" at the same time).
  private FlywheelGoal flywheelGoal = FlywheelGoal.STOPPED;
  private FeederGoal feederGoal = FeederGoal.STOPPED;

  // Driver's throttle handle. Returns the raw axis value each loop; periodic()
  // maps that to a 0..1 scaler and multiplies SHOOT_VOLTS by it. Default supplier
  // returns -1.0 (full throttle on a Logitech-style stick) so behavior is
  // unchanged if RobotContainer never calls setThrottleSupplier.
  private DoubleSupplier throttleSupplier = () -> -1.0;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // -- Read hardware and log inputs.
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // -- Compute the driver's throttle scaler. Flight stick throttle typically
    //    reports -1.0 (fully up/forward, "full power") to +1.0 (fully down/back,
    //    "no power") on Logitech-style sticks. We remap that to a 0..1 factor
    //    that multiplies SHOOT_VOLTS. Clamped so an out-of-range axis reading
    //    can't over- or under-drive the flywheels.
    double throttleScale = MathUtil.clamp((1.0 - throttleSupplier.getAsDouble()) / 2.0, 0.0, 1.0);

    // -- Translate the current goals into requested voltages. Only the shoot
    //    voltage is throttled; feed and eject use their fixed setpoints because
    //    they're binary "on/off" actions, not speed-tunable.
    double flywheelVoltsRequested =
        switch (flywheelGoal) {
          case STOPPED -> 0.0;
          case SHOOT -> SHOOT_VOLTS * -throttleScale;
        };
    double feederVoltsRequested =
        switch (feederGoal) {
          case STOPPED -> 0.0;
          case FEED -> FEED_VOLTS;
          case EJECT -> EJECT_VOLTS;
        };

    // -- Apply the RPM cap to the flywheel request.
    double flywheelVoltsApplied = flywheelVoltsRequested;
    boolean overspeed = false;
    for (double v : inputs.flywheelVelocityRadPerSec) {
      if (Math.abs(v) >= MAX_FLYWHEEL_RAD_PER_SEC && v * flywheelVoltsRequested > 0.0) {
        overspeed = true;
        break;
      }
    }
    if (overspeed) {
      flywheelVoltsApplied = 0.0;
    }

    // -- Push final setpoints to hardware. This is the only place in the class
    //    that writes to the IO layer; everything else just sets goals.
    io.setFlywheelVoltage(flywheelVoltsApplied);
    io.setFeederVoltage(feederVoltsRequested);

    // -- Logging. Goals show up as enum names in AdvantageScope; voltages let
    //    you see the RPM cap engaging.
    Logger.recordOutput("Shooter/FlywheelGoal", flywheelGoal);
    Logger.recordOutput("Shooter/FeederGoal", feederGoal);
    Logger.recordOutput("Shooter/ThrottleScale", throttleScale);
    Logger.recordOutput("Shooter/FlywheelVoltsRequested", flywheelVoltsRequested);
    Logger.recordOutput("Shooter/FlywheelVoltsApplied", flywheelVoltsApplied);
    Logger.recordOutput("Shooter/FeederVoltsApplied", feederVoltsRequested);
    Logger.recordOutput("Shooter/FlywheelOverspeedActive", overspeed);

    // -- Refresh disconnect alerts from the freshly-read connection state.
    for (int i = 0; i < flywheelDisconnectedAlerts.length; i++) {
      flywheelDisconnectedAlerts[i].set(!inputs.flywheelConnected[i]);
    }
    feederDisconnectedAlert.set(!inputs.feederConnected);
  }

  // ==========================================================================
  // Public command API.
  //
  // These commands intentionally do NOT require `this` subsystem. The standard
  // FRC pattern uses subsystem requirements to prevent two commands from
  // fighting over the same hardware, but here the flywheels and feeder are
  // independent actuators — there's no real resource conflict, and requiring
  // `this` would prevent the two-stage shoot (shoot() and feed() held at the
  // same time). Using Commands.startEnd(..) without requirements lets them
  // compose.
  // ==========================================================================

  /** Spin the flywheels up to the shoot setpoint while scheduled; stop on cancel. */
  public Command shoot() {
    return Commands.startEnd(
        () -> flywheelGoal = FlywheelGoal.SHOOT, () -> flywheelGoal = FlywheelGoal.STOPPED);
  }

  /** Run the feeder forward at the feed setpoint while scheduled; stop on cancel. */
  public Command feed() {
    return Commands.startEnd(
        () -> feederGoal = FeederGoal.FEED, () -> feederGoal = FeederGoal.STOPPED);
  }

  /** Reverse the feeder at the eject setpoint while scheduled; stop on cancel. */
  public Command eject() {
    return Commands.startEnd(
        () -> feederGoal = FeederGoal.EJECT, () -> feederGoal = FeederGoal.STOPPED);
  }

  /**
   * Wire an axis supplier (typically {@code joystick::getThrottle}) to scale the flywheel shoot
   * voltage. Called once from RobotContainer at construction.
   *
   * <p>Expected axis convention: -1.0 = full throttle (up/forward), +1.0 = off (down/back). This
   * matches Logitech-style flight sticks. If your specific stick reports the opposite, pass in
   * {@code () -> -joystick.getThrottle()} to flip the sign at the call site rather than changing
   * this class.
   */
  public void setThrottleSupplier(DoubleSupplier throttleSupplier) {
    this.throttleSupplier = throttleSupplier;
  }
}
