package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Checkmate;
import frc.robot.utils.Checkmate.TestResult;

public class Feeder extends SubsystemBase {
    private FeederInputsAutoLogged inputs;
    private FeederIO io;
    public Feeder(FeederIO io) {
        this.io = io;
        inputs = new FeederInputsAutoLogged();

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

    public Command runFeederRPS(double RPS) {
        return Commands.runOnce(() -> io.runFeederRPS(RPS));
    }

    public Command stopFeeder() {
        return Commands.runOnce(() -> {
            io.stopFeederMotor();
        }, this);
    }

    public Command zeroFeederEncoder() {
        return Commands.runOnce(() -> {
            io.zeroFeederEncoder();
        }, this);
    }

    public AngularVelocity getFeederVelocity() {
        return io.getFeederVelocity();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);
    }
}
