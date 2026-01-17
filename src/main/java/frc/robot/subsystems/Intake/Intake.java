package frc.robot.subsystems.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands; // added import

// import com.ctre.phoenix.motorcontrol.can.TalonFX; // or your vendor import
// import com.ctre.phoenix.motorcontrol.TalonFXConfiguration; // adjust per version

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final intakeInputsAutoLogged inputs;

    public Intake(IntakeIO io) {
        this.io = io;
        inputs = new intakeInputsAutoLogged();
    }

    public Command intakeCommand(double voltage) {
        return Commands.runOnce(() -> io.setVoltage(voltage), this);
    }

    public Command brakemodeCommand() {
        return Commands.runOnce(() -> io.brakeMode(), this);
    }

    public Command extendCommand() {
        return Commands.runOnce(() -> io.extend(), this);
    }

    public Command retractCommand() {
        return Commands.runOnce(() -> io.retract(), this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }
}

public final class IntakeIOTalonFX implements IntakeIO { // ensure IntakeIO is the correct interface
    // missing fields
    private final Object motor; // replace Object with TalonFX from your vendor lib
    private final DoubleSupplier positionSignal;

    public IntakeIOTalonFX(int canId) {
        // motor = new TalonFX(canId); // instantiate using your vendor constructor
        // TalonFXConfiguration config = new TalonFXConfiguration();
        // config.slot0.kP = ...; // if your version exposes slot0
        // apply configuration: either motor.configAllSettings(config) or TalonFXConfigurator.apply(motor, config)
        // positionSignal = () -> motor.getSelectedSensorPosition(); // replace with correct read API
    }

    @Override
    public void setVoltageCommand(double voltage) {
        // if your TalonFX supports setVoltage:
        // motor.setVoltage(voltage);
        // otherwise convert to percent output:
        // motor.set(ControlMode.PercentOutput, voltage / RobotController.getBatteryVoltage());
    }

    @Override
    public void setSetpointCommand(Distance position) {
        // use your position-control API (e.g., motor.set(ControlMode.Position, ticks) or motor.setControl(...))
    }

    @Override
    public Distance getPosition() {
        return Meters.of(positionSignal.getAsDouble());
    }

    @Override
    public void updateInputs(intakeInputsAutoLogged inputs) {
        // populate inputs from motor sensors
    }
}