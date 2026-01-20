package frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;


public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final intakeInputsAutoLogged inputs;

    Mechanism2d mech = new Mechanism2d(20, 20);

    public Intake(IntakeIO io) {
        this.io = io;
        inputs = new intakeInputsAutoLogged();

    }

    public Command intakeCommand(double voltage) {
                System.out.println("Intaking with voltage: " + voltage);
                return Commands.runOnce(() -> io.setVoltage(voltage), this);

    }

    public Command brakemodeCommand() {
        return Commands.runOnce(() -> io.brakeMode(), this);
    }

    public Command extendCommand() {
         System.out.println("Extending Intake");
         return Commands.runOnce(() -> io.setSetpointCommand(Meters.of(0.3)), this);
    }

    public Command retractCommand() {
        System.out.println("Retracting Intake");
        return Commands.runOnce(() -> io.setSetpointCommand(Meters.of(0.0)), this);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
}