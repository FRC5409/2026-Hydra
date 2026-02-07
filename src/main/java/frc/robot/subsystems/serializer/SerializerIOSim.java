package frc.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SerializerIOSim implements SerializerIO {

    private double indexerVoltage = 0.0;

    public SerializerIOSim() {
        
    }


    @Override
    public void setIndexerMotorVoltage(double voltage) {
        indexerVoltage = voltage;
    }

    

    

    @Override
    public void stopIndexerMotor() {
        indexerVoltage = 0.0;
    }

    

    @Override
    public AngularVelocity getIndexerVelocity() {
        return RotationsPerSecond.of(indexerVoltage);
    }

    

    @Override
    public void updateInputs(SerializerInputs inputs) {

        inputs.isIndexerMotorConnected = true;
        inputs.indexerAppliedVoltage = Volts.of(indexerVoltage);
        inputs.indexerMotorVelocity = getIndexerVelocity();
    }



}
