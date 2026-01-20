package frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;


public class Intake extends SubsystemBase {
    private final IntakeIO intakeIO;
    private final intakeInputsAutoLogged inputs;

    Mechanism2d mech = new Mechanism2d(20, 20);

    public Intake(IntakeIO io) {
        this.intakeIO = io;
        inputs = new intakeInputsAutoLogged();

    }

    public Command intakeCommand(double voltage) {
                // System.out.println("Intaking with voltage: " + voltage);
                return Commands.runOnce(() -> intakeIO.setVoltage(voltage), this);
    }

    public Command brakemodeCommand() {
        return Commands.runOnce(() -> intakeIO.brakeMode(), this);
    }

    public Command extendCommand() {
        //  System.out.println("Extending Intake");
         return Commands.runOnce(() -> intakeIO.setSetpoint(Meters.of(100)), this);
    }

    public Command retractCommand() {
        // System.out.println("Retracting Intake");
        return Commands.runOnce(() -> intakeIO.setSetpoint(Meters.of(0.0)), this);
    }

    public void periodic() {
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

    }
}