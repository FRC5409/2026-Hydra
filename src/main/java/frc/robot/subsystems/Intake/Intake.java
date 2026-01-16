package frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands; // added import


public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIO.intakeInputs inputs;

    public Intake(IntakeIO io) {
        this.io = io;
        inputs = new IntakeIO.intakeInputs();
    }

    public Command setVoltageCommand(double voltage) {
        // include 'this' so the command requires this subsystem
        return Commands.runOnce(() -> setVoltage(voltage), this);
    }

    // add implementation (forward to IO). Replace with your actual IO method.
    public void setVoltage(double voltage) {
        io.setVoltage(voltage); // implement or adapt to your IntakeIO API
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }
}