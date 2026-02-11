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

            final double extendTarget = Extension.EXTENSION_DISTANCE.in(Meters);
            final double retractTarget = Extension.EXTENSION_MIN_DISTANCE.in(Meters);
            final double timeoutSec = 5.0;
            final long sleepMs = 20;

            try {
                intakeIO.setSetpoint(Extension.EXTENSION_DISTANCE);
                double elapsed = 0.0;
                while (elapsed < timeoutSec) {
                    intakeIO.updateInputs(inputs);
                    if (inputs.extensionPosition >= extendTarget - 0.02) {
                        break;
                    }

                    Thread.sleep(sleepMs);
                    elapsed += sleepMs / 1000.0;
                }

                if (inputs.extensionPosition < extendTarget - 0.02) {
                    return TestResult.fail(String.format("Extension failed to extend (pos=%.3f target=%.3f)",
                            inputs.extensionPosition, extendTarget));
                }

                intakeIO.setSetpoint(Extension.EXTENSION_MIN_DISTANCE);
                elapsed = 0.0;

                while (elapsed < timeoutSec) {
                    intakeIO.updateInputs(inputs);
                    if (inputs.extensionPosition <= retractTarget + 0.02) {
                        break;
                    }

                    Thread.sleep(sleepMs);
                    elapsed += sleepMs / 1000.0;

                }

                if (inputs.extensionPosition > retractTarget + 0.02) {
                    return TestResult.fail(String.format("Extension failed to retract (pos=%.3f target=%.3f)",
                            inputs.extensionPosition, retractTarget));
                }

                return TestResult
                        .success(String.format("Extension ok (extend=%.3f retract=%.3f)", extendTarget, retractTarget));

            } 
            catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
                try {
                    intakeIO.setSetpoint(Extension.EXTENSION_MIN_DISTANCE);
                } 
                catch (Exception ignore) {
                }
                return TestResult.fail("Interrupted during extension check");
            } 
            catch (Exception ex) {
                try {
                    intakeIO.setSetpoint(Extension.EXTENSION_MIN_DISTANCE);
                } 
                catch (Exception ignore) {
                }
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
            } 
            catch (Exception ex) {
                try {
                    intakeIO.setRollerVoltage(0.0);
                } 
                catch (Exception ignore) {
                }
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