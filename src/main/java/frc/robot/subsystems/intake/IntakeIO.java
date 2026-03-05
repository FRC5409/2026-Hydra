package frc.robot.subsystems.intake;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface IntakeIO {

    @AutoLog
    public class IntakeInputs{

        public boolean isRollerConnected = false;
        public Voltage rollerVolts = Volts.of(0.0);
        public Current rollerCurrent = Amps.of(0.0);
        public double rollerTemp = 0.0;
        public AngularVelocity rollerVelocity = RotationsPerSecond.of(0.0);

        public boolean isExtensionConnected = false;
        public boolean isExtensionRunning = false;
        public Voltage extensionVolts = Volts.of(0.0);
        public Current extensionCurrent = Amps.of(0.0);
        public double extensionTemp = 0.0;
        public LinearVelocity extensionVelocity = MetersPerSecond.of(0.0);
        public double extensionPosition = 0.0;
        public Current extensionTorqueCurrent = Amps.of(0.0);

        public boolean isExtended = false;
        public boolean isRetracted = true;
        public boolean isCrashDetected = false;

    }
/**
 * Sets the voltage of the extension motor to the specified value. This method can be used to manually control the extension of the intake by applying a specific voltage to the motor. Positive voltage should extend the intake, while negative voltage should retract it.
 * @param voltage The voltage to set the extension motor to, in volts. Should be between -12 and 12.
 */
    public default void setExtensionVoltage(double voltage) {}
/**
 * Sets the voltage of the roller motor to the specified value. This method can be used to manually control the spinning of the intake roller by applying a specific voltage to the motor. Positive voltage should spin the roller in one direction, while negative voltage should spin it in the opposite direction.
 * @param voltage The voltage to set the roller motor to, in volts. Should be between -12 and 12.
 */
    public default void setRollerVoltage(double voltage) {}
/**
 * Updates the inputs of the intake subsystem by reading sensor values and other relevant information from the hardware. This method should be called periodically to ensure that the inputs are up to date and can be used for feedback control or monitoring the state of the intake during operation.
 * @param inputs The inputs object to update with the latest sensor values and other relevant information.
 */
    public default void updateInputs(IntakeInputs inputs) {}
/**
 * Gets the current drawn by the extension motor. This method can be used to monitor the current being drawn by the extension motor, which can provide information about the load on the motor and help detect potential issues such as stalling or overloading. The current is returned as a Current object, which can be used for feedback control or monitoring purposes.
 * @return The current drawn by the extension motor, in amps.
 */
    public default Current getMotorCurrent() {
        return Amps.of(0.0);
    }
/**
* Moves intake to setpoint
* @param position The position to set the extension motor to, in meters.
 */
    public default void setSetpoint(Distance position) {}
/**
* Sets both motors to coastMode
* @return A command that sets the extension motor to coast mode when executed.
 */
    public default void coastMode() {}
/**
* Sets both motors to brakeMode
* @return A command that sets the extension motor to brake mode when executed.
 */
    public default void brakeMode() {}
/**
* Extends the intake 
* @return A command that extends the intake when executed.
 */
    public default void extend() {}
/**
* Retracts the intake
* @return A command that retracts the intake when executed.
 */
    public default void retract() {}
/**
* Stops the motor
* @return A command that stops the extension motor when executed.
 */
    public default void stopMotor() {}
/**
* Gets the position of the intake extension
* @return The current position of the intake extension, in meters.
 */
    public default Distance getPosition() {  return null; }

    }

  
