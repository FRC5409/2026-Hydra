package frc.robot.subsystems.intake;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.intake.IntakeConstants.Extension;;

public final class IntakeIOTalonFX implements IntakeIO {
  
    private final TalonFX rollerMotor;
    private final TalonFX extensionMotor;

    private final PositionVoltage positionControl;

    private final StatusSignal<Angle>       rollerPositionSignal;
    private final StatusSignal<Temperature> rollerTemperatureSignal;
    private final StatusSignal<Voltage>     rollerVoltageSignal;
    private final StatusSignal<Current>     rollerCurrentSignal;
    private final StatusSignal<AngularVelocity>       rollerVelocitySignal;

    private final StatusSignal<Angle>       extensionPositionSignal;
    private final StatusSignal<Temperature> extensionTemperatureSignal;
    private final StatusSignal<Voltage>     extensionVoltageSignal;
    private final StatusSignal<Current>     extensionCurrentSignal;
    private final StatusSignal<AngularVelocity>       extensionVelocitySignal;
    private final StatusSignal<Current>    extensionTorqueCurrentSignal;

    public IntakeIOTalonFX(int rollerMotorId, int extensionMotorId) {

        rollerMotor = new TalonFX(rollerMotorId);
        extensionMotor = new TalonFX(extensionMotorId);
        rollerMotor.set(0.0);
        extensionMotor.set(0.0);

        positionControl = new PositionVoltage(0.0);
        positionControl.withSlot(0);

        TalonFXConfiguration extensionConfigurator = new TalonFXConfiguration()
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Extension.GEARING));

        if (frc.robot.Constants.IS_TUNING) { // If we are tuning, use the PID values from the Extension class, otherwise use the default TalonFX PID values
            if (Extension.INTAKE_IS_TUNING) {
                extensionConfigurator.Slot0 = new Slot0Configs()
                    .withKP(Extension.PID.getP())
                    .withKI(Extension.PID.getI())
                    .withKD(Extension.PID.getD());
            } else {
                extensionConfigurator.Slot0 = new Slot0Configs()
                    .withKP(Extension.TALONFX_PID.kP)
                    .withKI(Extension.TALONFX_PID.kI)
                    .withKD(Extension.TALONFX_PID.kD);
            }
        } else{
            extensionConfigurator.Slot0 = new Slot0Configs()
                .withKP(Extension.TALONFX_PID.kP)
                .withKI(Extension.TALONFX_PID.kI)
                .withKD(Extension.TALONFX_PID.kD);
        }
        extensionMotor.getConfigurator().apply(extensionConfigurator);

        extensionPositionSignal    = extensionMotor.getPosition();
        extensionTemperatureSignal = extensionMotor.getDeviceTemp();
        extensionVoltageSignal     = extensionMotor.getMotorVoltage();
        extensionCurrentSignal     = extensionMotor.getSupplyCurrent();
        extensionVelocitySignal    = extensionMotor.getVelocity();
        extensionTorqueCurrentSignal = extensionMotor.getTorqueCurrent();

        rollerPositionSignal    = rollerMotor.getPosition();
        rollerTemperatureSignal = rollerMotor.getDeviceTemp();
        rollerVoltageSignal     = rollerMotor.getMotorVoltage();
        rollerCurrentSignal     = rollerMotor.getSupplyCurrent();
        rollerVelocitySignal    = rollerMotor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(

            Extension.UPDATE_FREQUENCY,   
            extensionPositionSignal,
            extensionTemperatureSignal,
            extensionVoltageSignal,
            extensionCurrentSignal,

            rollerPositionSignal,
            rollerTemperatureSignal,
            rollerVoltageSignal,
            rollerCurrentSignal
        );

        rollerMotor.optimizeBusUtilization();
        extensionMotor.optimizeBusUtilization();

    }
/**
 * Sets the voltage of the roller motor to the specified value. This method can be used to manually control the spinning of the intake roller by applying a specific voltage to the motor. Positive voltage should spin the roller in one direction, while negative voltage should spin it in the opposite direction.
 * @param voltage The voltage to set the roller motor to, in volts.
 */
    public void setRollerVoltage(double voltage) {
        rollerMotor.setVoltage(voltage);
    }
/**
 * Sets the voltage of the extension motor to the specified value. This method can be used to manually control the extension of the intake by applying a specific voltage to the motor. Positive voltage should extend the intake, while negative voltage should retract it.
 * @param voltage The voltage to set the extension motor to, in volts.
 */
    public void setExtensionVoltage(double voltage) {
        extensionMotor.setVoltage(voltage);
    }
/**
 * Sets the position setpoint for the extension motor. This method can be used to control the extension of the intake by setting a desired position for the extension motor to reach. The position is specified as a Distance object, which can be used for feedback control to move the intake to the desired position during operation.
 * @param position The position to set the extension motor to, in meters.
 */
    public void setSetpoint(Distance position) {
        extensionMotor.setControl(positionControl.withPosition(position.in(Meters)).withSlot(0));
    }
/**
 * Stops the extension motor by setting its voltage to 0.0 volts. This command will cause the intake to stop moving when executed, but it will not change the current setpoint of the extension motor, so if the intake is extended or retracted and then this command is executed, the intake will hold its position rather than moving back to a default position.
 * @return A command that stops the extension motor when executed.
 */
    public void coastMode() {
        rollerMotor.setNeutralMode(NeutralModeValue.Coast);
        extensionMotor.setNeutralMode(NeutralModeValue.Coast);
    }
/**
 * Sets the brake mode of the extension motor. In brake mode, the motor will resist being moved when no voltage is applied, which can help hold the intake in place when extended or retracted. This method can be used to switch the extension motor to brake mode for better control and stability of the intake during operation.
 * @return A command that sets the extension motor to brake mode when executed.
 */
    public void brakeMode() {
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);
        extensionMotor.setNeutralMode(NeutralModeValue.Brake);
    }   
/**
 * Stops the extension motor by setting its voltage to 0.0 volts. This method can be used to stop the movement of the intake extension when executed, but it will not change the current setpoint of the extension motor, so if the intake is extended or retracted and then this method is executed, the intake will hold its position rather than moving back to a default position.
 * @return A command that stops the extension motor when executed.
 */
    public void stopMotor() {
        rollerMotor.set(0.0);
        extensionMotor.set(0.0);
    }
/**
 * Gets the current position of the intake extension. This method will return the current position of the intake extension as a Distance object, which can be used for feedback control or monitoring the intake's position during operation. The exact behavior of this method will depend on how it is implemented in the concrete class that implements the IntakeIO interface, but it should provide an accurate measurement of the current position of the intake extension.
 * @return The current position of the intake extension, in meters.
 */
    public Distance getPosition() {
        return Meters.of(extensionPositionSignal.getValueAsDouble());
    }
/**
 * Updates the inputs of the intake subsystem by reading sensor values and other relevant information from the hardware. This method should be called periodically to ensure that the inputs are up to date and can be used for feedback control or monitoring the state of the intake during operation.
 * @param inputs The inputs object to update with the latest sensor values and other relevant information.
 */
    @Override
    public void updateInputs(IntakeIO.IntakeInputs inputs) {

        inputs.isExtensionConnected = true;
        inputs.extensionVolts = Volts.of(extensionVoltageSignal.getValueAsDouble());
        inputs.extensionCurrent = Amps.of(extensionCurrentSignal.getValueAsDouble());
        inputs.extensionTorqueCurrent = Amps.of(extensionTorqueCurrentSignal.getValueAsDouble());
        inputs.extensionTemp = extensionTemperatureSignal.getValueAsDouble();
        inputs.extensionPosition =  Units.rotationsToRadians(extensionPositionSignal.getValueAsDouble());
        inputs.extensionVelocity = MetersPerSecond.of(extensionVelocitySignal.getValueAsDouble());
        inputs.isExtensionRunning = Math.abs(extensionVoltageSignal.getValueAsDouble()) > 0.1;
        inputs.isExtended = inputs.extensionPosition >= Extension.EXTENSION_MAX_DISTANCE.in(Meters) - 0.01;
        inputs.isRetracted = inputs.extensionPosition <= Extension.EXTENSION_MIN_DISTANCE.in(Meters) + 0.01;

        inputs.isRollerConnected = true;
        inputs.rollerVolts = Volts.of(rollerVoltageSignal.getValueAsDouble());
        inputs.rollerCurrent = Amps.of(rollerCurrentSignal.getValueAsDouble());
        inputs.rollerTemp = rollerTemperatureSignal.getValueAsDouble();
        inputs.rollerVelocity = RotationsPerSecond.of(rollerVelocitySignal.getValueAsDouble());

    }
}