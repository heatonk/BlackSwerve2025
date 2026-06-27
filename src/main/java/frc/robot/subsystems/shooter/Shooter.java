package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Shooter subsystem: two flywheels that launch the ball, plus a feeder that pushes the ball into
 * the spinning flywheels. The actual hardware lives behind {@link ShooterIO}, so we can swap in a
 * real Spark implementation, a physics sim, or a replay log without changing this class — same
 * pattern as Drive uses for its swerve modules.
 */
public class Shooter extends SubsystemBase {
  // Maximum allowed flywheel speed. A NEO's free speed at 12V is ~5676 RPM, so
  // this is a soft safety net just below that — set above your highest planned
  // operating speed so the cap only acts in fault cases, not during normal shots.
  // If a flywheel hits this speed we zero the voltage; the wheel coasts back
  // under the cap and voltage resumes on the next loop (20ms).
  private static final double MAX_FLYWHEEL_RPM = 5500.0;
  private static final double MAX_FLYWHEEL_RAD_PER_SEC = MAX_FLYWHEEL_RPM * 2.0 * Math.PI / 60.0;

  // The IO layer (real, sim, or replay) is injected by RobotContainer.
  private final ShooterIO io;

  // The "AutoLogged" class is generated from ShooterIOInputs by AdvantageKit's
  // annotation processor at build time. It knows how to serialize itself to the log.
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // One alert per motor so the dashboard tells us exactly which Spark dropped off.
  private final Alert[] flywheelDisconnectedAlerts =
      new Alert[] {
        new Alert("Shooter flywheel motor 0 disconnected.", AlertType.kError),
        new Alert("Shooter flywheel motor 1 disconnected.", AlertType.kError),
      };
  private final Alert feederDisconnectedAlert =
      new Alert("Shooter feeder motor disconnected.", AlertType.kError);

  // Voltage the active command wants to apply to the flywheels. periodic() reads
  // this every loop, enforces the RPM cap, then forwards the result to the IO
  // layer. Storing the request as state (instead of writing to IO directly) is
  // what makes the cap actually work — it gets re-checked at 50Hz instead of
  // only when the command starts/ends.
  private double flywheelVoltsRequested = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Read sensor values from hardware and publish them to the log under "Shooter/...".
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // Apply the RPM cap to the flywheel voltage request. The cap only engages
    // when the request would push an already-overspeeding wheel further in its
    // current direction; braking voltage (opposite sign) is always allowed so
    // we can actively slow a runaway wheel.
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
    io.setFlywheelVoltage(flywheelVoltsApplied);

    // Log requested vs applied so you can see the cap engaging in AdvantageScope.
    Logger.recordOutput("Shooter/FlywheelVoltsRequested", flywheelVoltsRequested);
    Logger.recordOutput("Shooter/FlywheelVoltsApplied", flywheelVoltsApplied);
    Logger.recordOutput("Shooter/FlywheelOverspeedActive", overspeed);

    // Refresh the dashboard alerts from the freshly-read connection state.
    for (int i = 0; i < flywheelDisconnectedAlerts.length; i++) {
      flywheelDisconnectedAlerts[i].set(!inputs.flywheelConnected[i]);
    }
    feederDisconnectedAlert.set(!inputs.feederConnected);
  }

  /** Run the flywheels open-loop at the given voltage. Positive = shooting direction. */
  public void runFlywheels(double volts) {
    flywheelVoltsRequested = volts;
  }

  /** Run the feeder open-loop at the given voltage. Positive = feeds ball into flywheels. */
  public void runFeeder(double volts) {
    io.setFeederVoltage(volts);
  }

  /** Stop all motors. */
  public void stop() {
    flywheelVoltsRequested = 0.0;
    io.setFeederVoltage(0.0);
  }

  /** Convenience command: spin only the flywheels at {@code volts} while scheduled. */
  public Command runFlywheelsCommand(double volts) {
    return startEnd(() -> runFlywheels(volts), () -> runFlywheels(0.0));
  }

  /** Convenience command: spin only the feeder at {@code volts} while scheduled. */
  public Command runFeederCommand(double volts) {
    return startEnd(() -> runFeeder(volts), () -> runFeeder(0.0));
  }

  /**
   * Convenience command: spin the flywheels and the feeder together while scheduled. Useful as a
   * one-button "shoot" command.
   */
  public Command shootCommand(double flywheelVolts, double feederVolts) {
    return startEnd(
        () -> {
          runFlywheels(flywheelVolts);
          runFeeder(feederVolts);
        },
        this::stop);
  }
}
