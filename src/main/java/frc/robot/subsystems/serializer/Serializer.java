package frc.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Checkmate;
import frc.robot.utils.Checkmate.TestResult;

public class Serializer extends SubsystemBase{
    private SerializerIO io;
    private final SerializerInputsAutoLogged inputs;

    public Serializer(SerializerIO io) {
        this.io = io;
        inputs = new SerializerInputsAutoLogged();
        
        Checkmate.register("Indexer spins", () -> {
            Command cmd = this.setIndexerVoltage(2);
            cmd.initialize();
            cmd.execute();
            if(this.getIndexerVelocity().in(RotationsPerSecond) > 0) {
                return TestResult.success("Indexer spins the right way");
            } else if (this.getIndexerVelocity().in(RotationsPerSecond) < 0) {
                return TestResult.fail("Indexer spins the wrong way");
            } else {
                return TestResult.fail("Indexer is not spinning!");
            }
        });

        Checkmate.register("Feeder spins", () -> {
            Command cmd = this.setFeederVoltage(2);
            cmd.initialize();
            cmd.execute();
            if(this.getFeederVelocity().in(RotationsPerSecond) > 0) {
                return TestResult.success("Feeder spins the right way");
            } else if (this.getFeederVelocity().in(RotationsPerSecond) < 0) {
                return TestResult.fail("Feeder spins the wrong way");
            } else {
                return TestResult.fail("Feeder is not spinning!");
            }
        });

    }

    public Command setFeederVoltage(double voltage){
        return Commands.runOnce(() -> {
            io.setFeederMotorVoltage(voltage);
        }, this);
    }

    public Command setIndexerVoltage(double voltage){
        return Commands.runOnce(() -> {
            io.setIndexerMotorVoltage(voltage);
        }, this);
    }

    public Command stopIndexer() {
        return Commands.runOnce(() -> {
            io.stopIndexerMotor();
        }, this);
    }

    public Command stopFeeder() {
        return Commands.runOnce(() -> {
            io.stopFeederMotor();
        }, this);
    }

    public Command zeroIndexerEncoder() {
        return Commands.runOnce(() -> {
            io.zeroIndexerEncoder();
        }, this);
    }

    public Command zeroFeederEncoder() {
        return Commands.runOnce(() -> {
            io.zeroFeederEncoder();
        }, this);
    }

    public AngularVelocity getIndexerVelocity() {
        return io.getIndexerVelocity();
    }

    public AngularVelocity getFeederVelocity() {
        return io.getFeederVelocity();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Serializer", inputs);
    }


}