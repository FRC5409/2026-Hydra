package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

public class IntakeIOSim implements IntakeIO {

  private static final double MAX_VOLTAGE = 12.0;
  private double appliedVoltage = 0.0;
  private Distance position = Meters.of(0);

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

    Mechanism2d mech = new Mechanism2d(20, 20);
    var arm = mech.getRoot("IntakeSim", 10, 10);
    
  }

  public void setVoltageCommand(double voltage) {
    double clamped = Math.max(-MAX_VOLTAGE, Math.min(MAX_VOLTAGE, voltage));
    appliedVoltage = clamped;
  }

  public void setSetpointCommand(Distance setpoint) {
    position = setpoint;
  }

  public Distance getPosition() {
    return position;
  }

  public void stopMotorCommand() {
    appliedVoltage = 0.0;
  }
}