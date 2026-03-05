package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.intake.IntakeConstants.Extension;

public class IntakeIOSim implements IntakeIO {
  
    private final ElevatorSim extensionSim;
    private final PIDController pid;

    private boolean running;
    private double rollerVoltage = 0.0;

    public IntakeIOSim() {
      
      RoboRioSim.setVInVoltage(12.0);

      extensionSim = new ElevatorSim(
            DCMotor.getKrakenX44(1), 
            Extension.GEARING, 
            Extension.INTAKE_MASS.in(Kilograms), 
              0.1,
            Extension.EXTENSION_MIN_DISTANCE.in(Meters), 
            Extension.EXTENSION_MAX_DISTANCE.in(Meters), 
            false, 
            Extension.EXTENSION_MIN_DISTANCE.in(Meters)
        );

      pid = new PIDController(
        Extension.PID.getP(),
        Extension.PID.getI(),
        Extension.PID.getD()
      );

      running = false;
    }
/**
 * Sets the voltage of the extension motor to the specified value. This method can be used to manually control the extension of the intake by applying a specific voltage to the motor. Positive voltage should extend the intake, while negative voltage should retract it.
 * @param voltage The voltage to set the extension motor to, in volts.
 */
    @Override
    public void setExtensionVoltage(double voltage) {
      extensionSim.setInputVoltage(voltage);
      running = voltage != 0;
    }
/**
 * Sets the voltage of the roller motor to the specified value. This method can be used to manually control the spinning of the intake roller by applying a specific voltage to the motor. Positive voltage should spin the roller in one direction, while negative voltage should spin it in the opposite direction.
 * @param voltage The voltage to set the roller motor to, in volts.
 */
    @Override
    public void setRollerVoltage(double voltage) {
      rollerVoltage = voltage;
    }
/**
 * Sets the position setpoint for the extension motor. This method can be used to control the extension of the intake by setting a desired position for the extension motor to reach. The position is specified as a Distance object, which can be used for feedback control to move the intake to the desired position during operation.
 * @param position The position to set the extension motor to, in meters.
 */
    @Override
    public void setSetpoint(Distance position) {
      pid.setSetpoint(position.in(Meters));
      running = true;
    }
/**
 * Gets the current position of the intake extension. This method will return the current position of the intake extension as a Distance object, which can be used for feedback control or monitoring the intake's position during operation.
 * @return The current position of the intake extension, in meters.
 */
    @Override
    public Distance getPosition() {
      return Meters.of(extensionSim.getPositionMeters());
    } 
/**
 * Stops the extension motor by setting its voltage to 0.0 volts. This command will cause the intake to stop moving when executed, but it will not change the current setpoint of the extension motor, so if the intake is extended or retracted and then this command is executed, the intake will hold its position rather than moving back to a default position.
 * @return A command that stops the extension motor when executed.
 */
    @Override
    public void stopMotor() {
      pid.reset();
      running = false;
      rollerVoltage = 0.0;
      setExtensionVoltage(0);
    }
/**
 * Updates the inputs of the intake subsystem by reading sensor values and other relevant information from the hardware. This method should be called periodically to ensure that the inputs are up to date and can be used for feedback control or monitoring the state of the intake during operation.
 * @param inputs The inputs object to update with the latest sensor values and other relevant information.
 */
    @Override
    public void updateInputs(IntakeInputs inputs) {

        double volts = 0.0;

        if (running) {
            double pidOut = pid.calculate(extensionSim.getPositionMeters()); // Calculate the PID output based on the current position of the extension
            double maxV = Math.max(Extension.MAX_VOLTAGE.in(Volts), RoboRioSim.getVInVoltage()); // Get the maximum voltage of the system (either the max voltage of the extension or the current voltage of the RoboRio, whichever is lower)
            volts = MathUtil.clamp(pidOut, -maxV, maxV); // Clamp the voltage to the maximum voltage of the system
        }

      extensionSim.setInputVoltage(volts);
      extensionSim.update(0.02);

      inputs.extensionPosition = extensionSim.getPositionMeters();
      inputs.isExtended = extensionSim.getPositionMeters() >= Extension.EXTENSION_MAX_DISTANCE.in(Meters) - 0.01;
      inputs.isRetracted = extensionSim.getPositionMeters() <= Extension.EXTENSION_MIN_DISTANCE.in(Meters) + 0.01;
      inputs.extensionVelocity = MetersPerSecond.of(extensionSim.getVelocityMetersPerSecond());
      inputs.extensionCurrent = Amps.of(extensionSim.getCurrentDrawAmps());
      inputs.extensionTorqueCurrent = Amps.of(extensionSim.getCurrentDrawAmps() * 0.5);
      inputs.isExtensionRunning = running;
      inputs.extensionVolts = Volts.of(volts);
      inputs.extensionTemp = 25.0;

      inputs.rollerCurrent = Amps.of(rollerVoltage / 12.0 * 20.0);
      inputs.rollerVolts = Volts.of(rollerVoltage);
      inputs.rollerTemp = 25.0;
      inputs.rollerVelocity = RotationsPerSecond.of(rollerVoltage / 12.0 * 5000.0);

      inputs.isRollerConnected = true;
      inputs.isExtensionConnected = true;

      pid.setPID(Extension.PID.getP(), Extension.PID.getI(), Extension.PID.getD()); // Update the PID constants in case they were changed during tuning
    }
}