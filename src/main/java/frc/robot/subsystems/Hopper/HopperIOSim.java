package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.Meters;

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
    private double manualVoltage = 0.0;

    public HopperIOSim() {
        hopperSim = new ElevatorSim(
            DCMotor.getFalcon500(2),
            10, //Test value, needs to be tuned
            0.1, //Test value
            0.0127, //Test value
            0.0, 
            0.3, 
            false,
            0.0
        );
        pid = new PIDController(0.001, 0.0, 0); //needs tuning
        running = false;
    }

    @Override
    public void setMotorVoltage(double voltage) {
        hopperSim.setInputVoltage(voltage);
        manualVoltage = voltage;
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
            current = hopperSim.getCurrentDrawAmps();

            /* MANUAL control */
            //volts = MathUtil.clamp(manualVoltage, -RoboRioSim.getVInVoltage(), RoboRioSim.getVInVoltage());
        }
        hopperSim.setInputVoltage (volts);
        hopperSim.update(0.02);

        inputs.mainMotorConnection = true;
        inputs.mainMotorVoltage = volts;
        inputs.mainMotorCurrent = Math.abs(current);
        inputs.mainMotorTemp = 0.0;
        inputs.mainMotorPosition = hopperSim.getPositionMeters();

        inputs.followerMotorConnection = true;
        inputs.followerMotorVoltage = volts;
        inputs.followerMotorCurrent = Math.abs(current);
        inputs.followerMotorTemp = 0.0;
        inputs.followerMotorPosition = hopperSim.getPositionMeters();
    }



}
