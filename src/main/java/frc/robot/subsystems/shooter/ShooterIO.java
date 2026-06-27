package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction layer for the shooter. The shooter has two flywheel motors (which spin the
 * wheels that launch the ball) and one feeder motor (which pushes the ball into the spinning
 * flywheels). Same AdvantageKit pattern as ModuleIO in the drivebase code.
 */
public interface ShooterIO {

  /**
   * The set of values that get logged every loop. AdvantageKit generates a
   * "ShooterIOInputsAutoLogged" class from this at build time.
   *
   * <p>Flywheel arrays have one entry per flywheel motor (index 0 and 1) so we can spot a single
   * misbehaving Spark in the logs. The feeder is a single motor so its fields are scalars.
   */
  @AutoLog
  public static class ShooterIOInputs {
    // --- Flywheel motors (two of them) ---
    public boolean[] flywheelConnected = new boolean[] {false, false};
    public double[] flywheelVelocityRadPerSec = new double[] {0.0, 0.0};
    public double[] flywheelAppliedVolts = new double[] {0.0, 0.0};
    public double[] flywheelCurrentAmps = new double[] {0.0, 0.0};

    // --- Feeder motor (just one) ---
    public boolean feederConnected = false;
    public double feederVelocityRadPerSec = 0.0;
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;
  }

  /** Pull the latest sensor values from hardware into the given inputs object. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run both flywheel motors at the given voltage (open loop, no PID). */
  public default void setFlywheelVoltage(double volts) {}

  /** Run the feeder motor at the given voltage (open loop, no PID). */
  public default void setFeederVoltage(double volts) {}
}
