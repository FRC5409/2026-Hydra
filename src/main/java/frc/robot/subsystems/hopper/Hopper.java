package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Checkmate;
import frc.robot.utils.Checkmate.TestResult;

public class Hopper extends SubsystemBase {
    private final HopperIO io;
    private final HopperInputsAutoLogged inputs;

    public Hopper(HopperIO io) {
        this.io = io;
        inputs = new HopperInputsAutoLogged();

        Checkmate.register("Hopper extends fully", () -> {
            Command cmd = this.fullExtend();
            cmd.initialize();
            cmd.execute();
            double extensionLength = this.getPosition().in(Meters);
            if (extensionLength == 0.3) {
                return TestResult.success();
            } else if (extensionLength== 0.0) {
                return TestResult.fail("Hopper did not start! " + 
                                (inputs.isMainMotorConnected ? "(Motor connected)" : "(Motor not connected)"));
            } else {
                return TestResult.fail("Hopper not extending fully! Current Position: " + io.getPosition());
            }
        });
    }

    /** 
     * Extends hopper 0.3 metres out 
     */
    public Command fullExtend() {
        return Commands.runOnce(
            () -> io.setSetpoint(HopperConstants.HOPPER_MAX_EXTENSION), this
        );
    }

    /** 
     * Retracts hopper all the way to 0.0m
     */
    public Command fullRetract() {
        return Commands.runOnce(
            () -> io.setSetpoint(HopperConstants.HOPPER_MIN_EXTENSION), this
        );
    }

    /** 
     * Positive voltage extends, Negative voltage retracts (MAX of 0.3m and MIN of 0.0m)
     */
    public Command manualMove(double voltage) {
        return Commands.runOnce(() -> io.setMotorVoltage(voltage), this);
    }

    public Command pumpRepeatedly() {
            return Commands.sequence(
                Commands.runOnce(() -> io.setSetpoint(HopperConstants.PUMP_EXTENSION_POINT), this),
                Commands.waitUntil(() -> getPosition().isNear(HopperConstants.PUMP_EXTENSION_POINT, HopperConstants.TOLERANCE)),
                Commands.runOnce(() -> io.setSetpoint(HopperConstants.PUMP_RETRACTION_POINT), this),
                Commands.waitUntil(() -> getPosition().isNear(HopperConstants.PUMP_RETRACTION_POINT, HopperConstants.TOLERANCE))
            ).repeatedly();
    }

    public Command stopMotor() {
        return Commands.runOnce(
            () -> io.stopMotor(), this
        );
    }

    public Command zeroEncoder() {
        return Commands.runOnce(() -> io.zeroEncoder(), this);
    }

    public Distance getPosition() {
        return io.getPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);
    }
}
