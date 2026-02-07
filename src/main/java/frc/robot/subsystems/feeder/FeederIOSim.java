package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.serializer.SerializerConstants;
import frc.robot.subsystems.serializer.SerializerIO.SerializerInputs;

public class FeederIOSim implements FeederIO {
    private double feederVoltage = 0.0;
    private double feederCurrent = 0.0;
    private final FlywheelSim feederSim;
    private final DCMotor motor = DCMotor.getKrakenX44(1);
    private final PIDController controller;
    private boolean running;

    public FeederIOSim() {
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
            FeederConstants.TALONFX_PID.kP, 
            FeederConstants.TALONFX_PID.kI,
            FeederConstants.TALONFX_PID.kD);
        running = false;
    }

    private void setAngularVelocity(double velocityRotationsPerSec) {
        feederSim.setAngularVelocity(velocityRotationsPerSec*(2*Math.PI));
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
    public void stopFeederMotor() {
        //feederVoltage = 0.0;
        setAngularVelocity(0);
    }

    @Override
    public AngularVelocity getFeederVelocity() {
        return RotationsPerSecond.of(feederSim.getAngularVelocityRPM()/60);
    }

    
    @Override
    public void updateInputs(FeederInputs inputs) {
        if (running) {
            feederVoltage = MathUtil.clamp(
                    controller.calculate(feederSim.getAngularVelocityRPM()),
                    12,
                    -12
            );

            feederCurrent = feederSim.getCurrentDrawAmps();
        }

        inputs.isFeederMotorConnected = true;
        inputs.feederAppliedVoltage = Volts.of(feederVoltage);
        inputs.feederMotorVelocity = getFeederVelocity();
    }

}
