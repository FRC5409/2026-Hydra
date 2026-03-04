package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class FeederIOSim implements FeederIO {
    private final FlywheelSim feederSim;
    private final DCMotor motor = DCMotor.getKrakenX44(1);
    private final PIDController controller;
    private boolean running;
    private double numberOfRotations;

    // gets the setpoint for logging
    private double simSetpoint;

    public FeederIOSim() {
        feederSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                        motor,
                        0.002, // just for sim
                        1 
                ),
            motor
        );

        controller = new PIDController(
            FeederConstants.SIM_PID.kP, 
            FeederConstants.SIM_PID.kI,
            FeederConstants.SIM_PID.kD);
        running = false;
    }

    /**
     * Sets the flywheel sim's voltage manually
     * @param voltage the voltage to set
     */
    @Override
    public void setMotorVoltage(double voltage) {
        feederSim.setInputVoltage(voltage);
        running = true;
    }

    /**
     * Sets the setpoint in RPS
     * @param velocity the target velocity setpoint
     */
    @Override
    public void runRPS(Supplier<AngularVelocity> velocity) {
        controller.setSetpoint(velocity.get().in(RotationsPerSecond));
        simSetpoint = velocity.get().in(RotationsPerSecond);
        running = true;
    }

    /**
     * Sets the setpoint to 0.0 RPS to stop the motor
     */
    @Override
    public void stopMotor() {
        controller.setSetpoint(0.0);
        running = false;
    }

    /** Returns the velocity in RPS
     * Gets the feeder sim's velocity in RPM and divides it by 60 to get RPS 
     */
    @Override
    public AngularVelocity getVelocityRPS() {
        return RotationsPerSecond.of(feederSim.getAngularVelocityRPM()/60);
    }

    
    @Override
    public void updateInputs(FeederInputs inputs) {
        double simVoltage = 0.0;
        
        // Calculates PID output
        if (running) {
            simVoltage = MathUtil.clamp(
                    controller.calculate(getVelocityRPS().in(RotationsPerSecond)),
                    -RoboRioSim.getVInVoltage(),
                    RoboRioSim.getVInVoltage()
            );
        }
        feederSim.setInputVoltage(simVoltage);
        feederSim.update(0.02);
        
        inputs.isMotorConnected = true;
        inputs.appliedVoltage = Volts.of(feederSim.getInputVoltage());
        inputs.motorVelocity = getVelocityRPS();
        inputs.appliedCurrent = Amps.of(feederSim.getCurrentDrawAmps());
        numberOfRotations += getVelocityRPS().in(RotationsPerSecond)*0.02;
        inputs.motorPosition = Rotations.of(numberOfRotations);
        inputs.setpoint = simSetpoint;

    }

}
