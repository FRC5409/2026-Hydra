package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FeederIOSim implements FeederIO {
    private double simVoltage = 0.0;
    private double simCurrent = 0.0;
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
    public void setMotorVoltage(double voltage) {
        simVoltage = voltage;
    }

    @Override
    public void runRPS(double velocity) {
        setAngularVelocity(velocity);
    }

    @Override
    public void stopMotor() {
        //feederVoltage = 0.0;
        setAngularVelocity(0);
        controller.reset();
    }

    @Override
    public AngularVelocity getVelocityRPS() {
        //return RotationsPerSecond.of(feederVoltage);
        return RotationsPerSecond.of(feederSim.getAngularVelocityRPM()/60);
    }

    
    @Override
    public void updateInputs(FeederInputs inputs) {
        if (running) {
            simVoltage = MathUtil.clamp(
                    controller.calculate(feederSim.getAngularVelocityRPM()),
                    -12,
                    12
            );

            simCurrent = feederSim.getCurrentDrawAmps();
        }

        inputs.isFeederMotorConnected = true;
        inputs.feederAppliedVoltage = Volts.of(simVoltage);
        inputs.feederMotorVelocity = getVelocityRPS();
        inputs.feederAppliedCurrent = Amps.of(simCurrent);
    }

}
