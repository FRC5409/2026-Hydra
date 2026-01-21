package frc.robot.subsystems.Serializer;

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
        inputs.floorMotorConnection = true;
        inputs.feederMotorConnection = true;
        inputs.floorAppliedVoltage = volts;
        inputs.feederAppliedVoltage = volts;
    }



}
