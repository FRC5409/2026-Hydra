package frc.robot.subsystems.Serializer;

import static edu.wpi.first.units.Units.Volts;

public class SerializerIOSim implements SerializerIO {

    private double volts = 0.0;

    public SerializerIOSim() {}

    @Override
    public void setMotorVoltage(double voltage) {
        volts = voltage;
    }

    @Override
    public void stopMotor() {
        volts = 0.0;
    }

    @Override
    public void updateInputs(SerializerInputs inputs) {
        inputs.isFloorMotorConnected = true;
        inputs.isFeederMotorConnected = true;
        inputs.floorAppliedVoltage = Volts.of(volts);
        inputs.feederAppliedVoltage = Volts.of(volts);
    }



}
