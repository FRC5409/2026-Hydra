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
    private double feederVoltage = 0.0;
    private double feederCurrent = 0.0;
    private final FlywheelSim feederSim;
    private final DCMotor motor = DCMotor.getKrakenX44(1);
    private final PIDController controller;
    private boolean running;

    public SerializerIOSim() {
        feederSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                        motor,
                        1,
                        1
                ),
            motor
        );

        feederSim.update(0.01);
        controller = new PIDController(
            SerializerConstants.FeederConstants.TALONFX_PID.kP, 
            SerializerConstants.FeederConstants.TALONFX_PID.kI,
            SerializerConstants.FeederConstants.TALONFX_PID.kD);
        running = false;
    }

    private void setAngularVelocity(double velocityRotationsPerSec) {
        feederSim.setAngularVelocity(velocityRotationsPerSec*(2*Math.PI));
    }

    @Override
    public void setIndexerMotorVoltage(double voltage) {
        indexerVoltage = voltage;
        running = true;
    }

    @Override
    public void setFeederMotorVoltage(double voltage) {
        feederVoltage = voltage;
    }

    @Override
    public void runFeederRPS(double velocity) {
        setAngularVelocity(velocity);
    }

    @Override
    public void stopIndexerMotor() {
        indexerVoltage = 0.0;
    }

    @Override
    public void stopFeederMotor() {
        //feederVoltage = 0.0;
        setAngularVelocity(0);
    }

    @Override
    public AngularVelocity getIndexerVelocity() {
        return RotationsPerSecond.of(indexerVoltage);
    }

    @Override
    public AngularVelocity getFeederVelocity() {
        return RotationsPerSecond.of(feederSim.getAngularVelocityRPM()/60);
    }

    @Override
    public void updateInputs(SerializerInputs inputs) {
        if (running) {
            feederVoltage = MathUtil.clamp(
                    controller.calculate(feederSim.getAngularVelocityRPM()),
                    12,
                    -12
            );

            feederCurrent = feederSim.getCurrentDrawAmps();
        }

        inputs.isIndexerMotorConnected = true;
        inputs.isFeederMotorConnected = true;
        inputs.indexerAppliedVoltage = Volts.of(indexerVoltage);
        inputs.feederAppliedVoltage = Volts.of(feederVoltage);
        inputs.indexerMotorVelocity = getIndexerVelocity();
        inputs.feederMotorVelocity = getFeederVelocity();
    }



}
