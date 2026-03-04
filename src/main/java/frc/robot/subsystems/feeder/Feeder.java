package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Checkmate;
import frc.robot.utils.Checkmate.TestResult;

public class Feeder extends SubsystemBase {

    private final FeederInputsAutoLogged inputs;
    private final FeederIO io;
    private static AngularVelocity targetRPS;
    
    /** Creates a new Feeder */
    public Feeder(FeederIO io) {
        
        // Puts the PID and RPS values to SmartDashboard
        SmartDashboard.putData("Feeder/PID", FeederConstants.PID);
        SmartDashboard.putNumber("Feeder/RPS", 0.0);

        this.io = io;
        inputs = new FeederInputsAutoLogged();

        // Checkmate Command to check if the Feeder spins the right way
        // Starts spinning the feeder, then checks if the velocity is greater than 0
        Checkmate.register("Should spin to move FUEL towards launcher", () -> {
            Command cmd = this.runRPS(() -> RotationsPerSecond.of(10));
            cmd.initialize();
            cmd.execute();
            if(this.getVelocity().in(RotationsPerSecond) > 0) {
                return TestResult.success("Feeder spins the right way");
            } else if (this.getVelocity().in(RotationsPerSecond) < 0) {
                return TestResult.fail("Feeder spins the wrong way");
            } else {
                return TestResult.fail("Feeder is not spinning!");
            }
        });
    }

    /**
     * Sets manual voltage
     * @param voltage the assigned voltage
     */
    public Command setVoltage(double voltage){
        return Commands.runOnce(() -> {
            io.setMotorVoltage(voltage);
        }, this);
    }

    /**
     * Runs the feeder at a supplied RPS - can be modified via SmartDashboard
     * @param RPS the assigned velocity in RPS
     * @return
     */
    public Command runRPS(Supplier<AngularVelocity> RPS) {
        return Commands.runOnce(() -> {
            io.runRPS(RPS);
        }, this);
    }

    /**
     * Stops the motor
     */
    public Command stopMotor() {
        return Commands.runOnce(() -> {
            io.stopMotor();
        }, this);
    }

    
    /**
     * Zeroes encoders if needed
     */
    public Command zeroEncoder() {
        return Commands.runOnce(() -> {
            io.zeroEncoder();
        }, this);
    }

    /**
     * Gets the velocity
     * @return the Feeder's current velocity in RPS
     */
    public AngularVelocity getVelocity() {
        return io.getVelocityRPS();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);

        // Constantly runs the feeder at the supplied RPS
        targetRPS = RotationsPerSecond.of(SmartDashboard.getNumber("Feeder/RPS", 0));
    }
}
