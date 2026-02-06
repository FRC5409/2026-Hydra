package frc.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;

public class SerializerIOSim implements SerializerIO {

    private double indexerVoltage = 0.0;
    private double feederVoltage = 0.0;

    public SerializerIOSim() {}

    @Override
    public void setIndexerMotorVoltage(double voltage) {
        indexerVoltage = voltage;
    }

    @Override
    public void setFeederMotorVoltage(double voltage) {
        feederVoltage = voltage;
    }

    @Override
    public void stopIndexerMotor() {
        indexerVoltage = 0.0;
    }

    @Override
    public void stopFeederMotor() {
        feederVoltage = 0.0;
    }

    @Override
    public AngularVelocity getIndexerVelocity() {
        return RotationsPerSecond.of(indexerVoltage);
    }

    @Override
    public AngularVelocity getFeederVelocity() {
        return RotationsPerSecond.of(feederVoltage);
    }

    @Override
    public void updateInputs(SerializerInputs inputs) {
        inputs.isIndexerMotorConnected = true;
        inputs.isFeederMotorConnected = true;
        inputs.indexerAppliedVoltage = Volts.of(indexerVoltage);
        inputs.feederAppliedVoltage = Volts.of(feederVoltage);
        inputs.indexerMotorVelocity = RotationsPerSecond.of(indexerVoltage); 
        inputs.feederMotorVelocity = RotationsPerSecond.of(feederVoltage);
    }



}
