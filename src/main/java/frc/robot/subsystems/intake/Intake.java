package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.Extension;
import frc.robot.utils.Checkmate;
import frc.robot.utils.Checkmate.TestResult;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final IntakeIO intakeIO;
    private final IntakeInputsAutoLogged inputs;

    public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
        this.inputs = new IntakeInputsAutoLogged();

        Checkmate.register("Extension extend-retract", () -> {
            try {

                intakeIO.setSetpoint(Extension.EXTENSION_DISTANCE);
                Thread.sleep(1500);
                intakeIO.updateInputs(inputs);
                double posExtend = inputs.extensionPosition;

                if (posExtend < Extension.EXTENSION_DISTANCE.in(Meters) * 0.9) {
                    return TestResult.fail("Extension failed to extend, position: " + posExtend);
                }

                intakeIO.setSetpoint(Extension.EXTENSION_MIN_DISTANCE);
                Thread.sleep(1500);
                intakeIO.updateInputs(inputs);
                double posRetract = inputs.extensionPosition;

                if (posRetract > Extension.EXTENSION_MIN_DISTANCE.in(Meters) + 0.05) {
                    return TestResult.fail("Extension failed to retract, position: " + posRetract);
                }

                return TestResult.success(
                    String.format("Extension ok (extend=%.3f retract=%.3f)", posExtend, posRetract));

            } catch (Exception ex) {
                try { intakeIO.setSetpoint(Extension.EXTENSION_MIN_DISTANCE); } catch (Exception ignore) {}
                return TestResult.fail("Exception during extension check: " + ex.getMessage());
            }
        });

        Checkmate.register("Intake roller", () -> {
            try {

                intakeIO.setRollerVoltage(30.0);
                Thread.sleep(1000);
                intakeIO.updateInputs(inputs);
                double current = inputs.rollerCurrent.in(Amps);

                if (Math.abs(current) < 25.0) {

                    return TestResult.fail("Intake roller failed to spin up, current: " + current);

                }

                intakeIO.setRollerVoltage(0.0);
                return TestResult.success("Intake roller ok, current: " + current);

            } catch (Exception ex) {
                try { intakeIO.setRollerVoltage(0.0); } catch (Exception ignore) {}
                return TestResult.fail("Exception during intake roller check: " + ex.getMessage());
            }
        });
    }

    public Command intakeCommand(double voltage) {
        return Commands.runOnce(() -> intakeIO.setRollerVoltage(voltage), this);
    }

    public Command brakemodeCommand() {
        return Commands.runOnce(() -> intakeIO.brakeMode(), this);
    }

    public Command extendCommand() {
         return Commands.runOnce(() -> intakeIO.setSetpoint(Extension.EXTENSION_DISTANCE), this);
    }

    public Command retractCommand() {
        return Commands.runOnce(() -> intakeIO.setSetpoint(Extension.EXTENSION_MIN_DISTANCE), this);
    }

    @Override
    public void periodic() {
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
}