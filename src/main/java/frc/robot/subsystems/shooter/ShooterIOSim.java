package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics-sim implementation of {@link ShooterIO}. Two flywheel motors + one feeder motor, each
 * modeled as an independent NEO so the logs look the same shape as on the real robot.
 */
public class ShooterIOSim implements ShooterIO {
  // 1:1 gearing and a small moment of inertia is a reasonable default. Tune if
  // you want sim spin-up time to match the real robot.
  private static final double FLYWHEEL_MOI_KG_M2 = 0.004;
  private static final double FEEDER_MOI_KG_M2 = 0.002;
  private static final double GEARING = 1.0;

  private final DCMotorSim[] flywheelSims = new DCMotorSim[2];
  private final DCMotorSim feederSim;

  private double flywheelAppliedVolts = 0.0;
  private double feederAppliedVolts = 0.0;

  public ShooterIOSim() {
    var gearbox = DCMotor.getNEO(1);
    for (int i = 0; i < 2; i++) {
      flywheelSims[i] =
          new DCMotorSim(
              LinearSystemId.createDCMotorSystem(gearbox, FLYWHEEL_MOI_KG_M2, GEARING), gearbox);
    }
    feederSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, FEEDER_MOI_KG_M2, GEARING), gearbox);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // --- Flywheels ---
    for (int i = 0; i < 2; i++) {
      // Push the latest commanded voltage into the sim and advance it one loop (20ms).
      flywheelSims[i].setInputVoltage(MathUtil.clamp(flywheelAppliedVolts, -12.0, 12.0));
      flywheelSims[i].update(0.02);

      inputs.flywheelConnected[i] = true;
      inputs.flywheelVelocityRadPerSec[i] = flywheelSims[i].getAngularVelocityRadPerSec();
      inputs.flywheelAppliedVolts[i] = flywheelAppliedVolts;
      inputs.flywheelCurrentAmps[i] = Math.abs(flywheelSims[i].getCurrentDrawAmps());
    }

    // --- Feeder ---
    feederSim.setInputVoltage(MathUtil.clamp(feederAppliedVolts, -12.0, 12.0));
    feederSim.update(0.02);

    inputs.feederConnected = true;
    inputs.feederVelocityRadPerSec = feederSim.getAngularVelocityRadPerSec();
    inputs.feederAppliedVolts = feederAppliedVolts;
    inputs.feederCurrentAmps = Math.abs(feederSim.getCurrentDrawAmps());
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    flywheelAppliedVolts = volts;
  }

  @Override
  public void setFeederVoltage(double volts) {
    feederAppliedVolts = volts;
  }
}
