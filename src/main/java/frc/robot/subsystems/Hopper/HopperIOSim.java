package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class HopperIOSim implements HopperIO {
    private boolean running;
    private ElevatorSim hopperSim;
    private PIDController pid;
    private double inputVoltage = 0.0;

    public HopperIOSim() {

        hopperSim = new ElevatorSim(
            DCMotor.getKrakenX60(1), 
            HopperConstants.kGearing, 
            HopperConstants.HOPPER_MASS.in(Kilograms), 
            HopperConstants.HOPPER_DRUMRADIUS.in(Meters), 
            HopperConstants.HOPPER_MIN_EXTENSION, 
            HopperConstants.HOPPER_MAX_EXTENSION, 
            false, 
            0.0
            );

        pid = new PIDController(HopperConstants.SIM_PID.kP, HopperConstants.SIM_PID.kI, HopperConstants.SIM_PID.kD);
        running = false;
    }

    @Override
    public void setMotorVoltage(double voltage) {
        hopperSim.setInputVoltage(voltage);
        inputVoltage = voltage;
        running = true;
    }

    @Override
    public void stopMotor() {
        hopperSim.setInputVoltage(0.0);
        running = false;
    }

    @Override
    public void setSetpoint(Distance setpoint) {
        pid.setSetpoint(setpoint.in(Meters));
        running = true;
    }

    public Distance getPosition() {
        return Meters.of(hopperSim.getPositionMeters());
    }

    
    public void updateInputs(HopperInputs inputs) {
        double volts = 0.0;
        double current = 0.0;
        if (running) {

            /* PID control */
            volts = MathUtil.clamp(
                pid.calculate(hopperSim.getPositionMeters()) * 12, 
                -RoboRioSim.getVInVoltage(), 
                RoboRioSim.getVInVoltage()
            );

            /* MANUAL control */
            //volts = MathUtil.clamp(manualVoltage, -RoboRioSim.getVInVoltage(), RoboRioSim.getVInVoltage());

            current = hopperSim.getCurrentDrawAmps();

        }
        hopperSim.setInputVoltage (volts);
        hopperSim.update(0.02);

        inputs.mainMotorConnection = true;
        inputs.mainAppliedVoltage = Volts.of(volts);
        inputs.mainAppliedCurrent = Amps.of(current);
        inputs.mainMotorTemp = 0.0;
        inputs.mainMotorPosition = Meters.of(hopperSim.getPositionMeters());

        inputs.followerMotorConnection = true;
        inputs.followerAppliedVoltage = Volts.of(volts);
        inputs.followerAppliedCurrent = Amps.of(current);
        inputs.followerMotorTemp = 0.0;
        inputs.followerMotorPosition = Meters.of(hopperSim.getPositionMeters());
    }



}
