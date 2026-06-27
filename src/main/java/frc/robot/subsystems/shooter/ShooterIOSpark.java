package frc.robot.subsystems.shooter;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

/**
 * Real-hardware implementation of {@link ShooterIO}. Two NEO brushless motors drive the flywheels
 * and one NEO drives the feeder, each via a Spark MAX.
 */
public class ShooterIOSpark implements ShooterIO {
  // CAN IDs — change to match your wiring.
  private static final int[] FLYWHEEL_CAN_IDS = new int[] {10, 11};
  private static final int FEEDER_CAN_ID = 12;

  // Current limits: 40A is a fine starting point for a NEO on a flywheel. The
  // feeder is doing less mechanical work, so it gets a lower limit.
  private static final int FLYWHEEL_CURRENT_LIMIT = 40;
  private static final int FEEDER_CURRENT_LIMIT = 30;

  // Flywheels: two Sparks driven together.
  private final SparkBase[] flywheelSparks = new SparkBase[2];
  private final RelativeEncoder[] flywheelEncoders = new RelativeEncoder[2];
  private final Debouncer[] flywheelDebouncers = new Debouncer[2];

  // Feeder: single Spark.
  private final SparkBase feederSpark;
  private final RelativeEncoder feederEncoder;
  private final Debouncer feederDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ShooterIOSpark() {
    // --- Flywheel config: coast mode so wheels spin down naturally after a shot.
    var flywheelConfig = new SparkMaxConfig();
    flywheelConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(FLYWHEEL_CURRENT_LIMIT)
        .voltageCompensation(12.0);

    for (int i = 0; i < 2; i++) {
      flywheelSparks[i] = new SparkMax(FLYWHEEL_CAN_IDS[i], MotorType.kBrushless);
      flywheelEncoders[i] = flywheelSparks[i].getEncoder();
      flywheelDebouncers[i] = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

      // tryUntilOk retries the config call a few times if CAN is flaky at startup.
      final int idx = i;
      tryUntilOk(
          flywheelSparks[idx],
          5,
          () ->
              flywheelSparks[idx].configure(
                  flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    // --- Feeder config: brake mode so the ball stops where we tell it to,
    // instead of coasting into the flywheels. Inverted so a positive voltage
    // pushes the ball INTO the flywheels (matches the contract on setFeederVoltage).
    var feederConfig = new SparkMaxConfig();
    feederConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(FEEDER_CURRENT_LIMIT)
        .voltageCompensation(12.0);

    feederSpark = new SparkMax(FEEDER_CAN_ID, MotorType.kBrushless);
    feederEncoder = feederSpark.getEncoder();
    tryUntilOk(
        feederSpark,
        5,
        () ->
            feederSpark.configure(
                feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // --- Update flywheel inputs ---
    for (int i = 0; i < 2; i++) {
      // sparkStickyFault is set by ifOk if any read fails. Reset it per-motor
      // so each motor's "connected" flag reflects only its own reads.
      sparkStickyFault = false;

      final int idx = i;
      ifOk(
          flywheelSparks[idx],
          flywheelEncoders[idx]::getVelocity,
          // getVelocity returns RPM; convert to rad/sec so logs stay in SI units.
          (value) -> inputs.flywheelVelocityRadPerSec[idx] = value * 2.0 * Math.PI / 60.0);
      ifOk(
          flywheelSparks[idx],
          new DoubleSupplier[] {
            flywheelSparks[idx]::getAppliedOutput, flywheelSparks[idx]::getBusVoltage
          },
          // Applied output is a [-1, 1] duty cycle; multiply by bus voltage to get volts.
          (values) -> inputs.flywheelAppliedVolts[idx] = values[0] * values[1]);
      ifOk(
          flywheelSparks[idx],
          flywheelSparks[idx]::getOutputCurrent,
          (value) -> inputs.flywheelCurrentAmps[idx] = value);

      inputs.flywheelConnected[idx] = flywheelDebouncers[idx].calculate(!sparkStickyFault);
    }

    // --- Update feeder inputs ---
    sparkStickyFault = false;
    ifOk(
        feederSpark,
        feederEncoder::getVelocity,
        (value) -> inputs.feederVelocityRadPerSec = value * 2.0 * Math.PI / 60.0);
    ifOk(
        feederSpark,
        new DoubleSupplier[] {feederSpark::getAppliedOutput, feederSpark::getBusVoltage},
        (values) -> inputs.feederAppliedVolts = values[0] * values[1]);
    ifOk(feederSpark, feederSpark::getOutputCurrent, (value) -> inputs.feederCurrentAmps = value);
    inputs.feederConnected = feederDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    // Both flywheels run together at the same voltage.
    for (var spark : flywheelSparks) {
      spark.setVoltage(volts);
    }
  }

  @Override
  public void setFeederVoltage(double volts) {
    feederSpark.setVoltage(volts);
  }
}
