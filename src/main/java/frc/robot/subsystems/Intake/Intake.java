package frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final IntakeIO intakeIO;
    private final intakeInputsAutoLogged inputs;

    public Intake(IntakeIO io) {

        this.intakeIO = io;
        inputs = new intakeInputsAutoLogged();

    }

    public Command intakeCommand(double voltage) {

        System.out.println("Running Intake at voltage: " + voltage);
        return Commands.runOnce(() -> intakeIO.setRollerVoltage(voltage), this);

    }

    public Command brakemodeCommand() {

        return Commands.runOnce(() -> intakeIO.brakeMode(), this);

    }

    public Command extendCommand() {

        System.out.println("Extending Intake");
         return Commands.runOnce(() -> intakeIO.setSetpoint(Meters.of(0.5)), this);

    }

    public Command retractCommand() {

        System.out.println("Retracting Intake");
        return Commands.runOnce(() -> intakeIO.setSetpoint(Meters.of(0.0)), this);

    }

    @Override
    public void periodic() {

        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

    }
}