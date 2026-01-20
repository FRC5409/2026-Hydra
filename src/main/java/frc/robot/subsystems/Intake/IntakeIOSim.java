package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Distance;

public class IntakeIOSim implements IntakeIO {

  private static final double MAX_VOLTAGE = 12.0;
  private double appliedVoltage = 0.0;
  private Distance position = Meters.of(0);

  @Override
  public void setVoltage(double voltage) {
    appliedVoltage = voltage;
    System.out.println("IntakeIOSim.setVoltage: voltage=" + voltage);
  }

  @Override
  public void setSetpoint(Distance setpoint) {
    position = setpoint;
    System.out.println("IntakeIOSim.setSetpoint: setpoint=" + setpoint.in(Meters));
  }

  @Override
  public Distance getPosition() {
    return position;
  }

  @Override
  public void stopMotor() {
    appliedVoltage = 0.0;
  }

  @Override
  public void updateInputs(IntakeIO.intakeInputs inputs) {
    inputs.extensionPosition = position.in(Meters);
    inputs.extensionVolts = appliedVoltage;

    inputs.extensionCurrent = Math.abs(inputs.extensionVolts) / MAX_VOLTAGE * 40.0; // crude estimate
    inputs.extensionTemp = 20.0 + inputs.extensionCurrent * 0.05; // crude estimate
    inputs.extensionConnection = true;

    inputs.rollerVolts = appliedVoltage;
    inputs.rollerCurrent = Math.abs(inputs.rollerVolts) / MAX_VOLTAGE * 20.0; // crude estimate
    inputs.rollerTemp = 20.0 + inputs.rollerCurrent * 0.05; // crude estimate
    inputs.rollerConnection = true;
    // System.out.println("IntakeIOSim.updateInputs: volts=" + inputs.extensionVolts + " pos=" + inputs.extensionPosition);
  }
}
