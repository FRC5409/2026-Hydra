package frc.robot.subsystems.serializer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Serializer extends SubsystemBase{
    private SerializerIO io;
    // private final SerializerInputsAutoLogged inputs;

    public Serializer(SerializerIO io) {
        this.io = io;
        // inputs = new SerializerInputsAutoLogged();
    }

    public Command feedToLauncher() {
        return Commands.runOnce(() ->
            io.setMotorVoltage(4.0), this); //test value
    }

    public Command feedToIntake() {
        return Commands.runOnce(() ->
            io.setMotorVoltage(-4.0), this); //test value
    }

    public Command stop() {
        return Commands.runOnce(() ->
            io.stopMotor(), this);
    }

    public Command runFeederVoltage(double voltage){
        return Commands.runOnce(() -> {
            io.setFeederMotorVoltage(voltage);
        });
    }

    public Command runIndexerVoltage(double voltage){
        return Commands.runOnce(() -> {
            io.setIndexerMotorVoltage(voltage);
        });
    }


    public void periodic() {
        // io.updateInputs(inputs);
        // Logger.processInputs("Serializer", inputs);
    }


}