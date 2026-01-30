package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Hopper.HopperInputsAutoLogged;

public class Hopper extends SubsystemBase {
    private final HopperIO io;
    private final HopperInputsAutoLogged inputs;
    private final LoggedMechanismRoot2d root;
    private final LoggedMechanismLigament2d slider;
    private final LoggedMechanism2d mechanism;

    public Hopper(HopperIO io) {
        this.io = io;
        inputs = new HopperInputsAutoLogged();
        mechanism = new LoggedMechanism2d(2, 2);
        root = mechanism.getRoot("Hopper", 1, 1);
        slider = new LoggedMechanismLigament2d("Arm", 0.3, 0);
        root.append(slider);
    }

    /** 
     * Extends hopper 0.3 metres out 
     */
    public Command fullExtend() {
        return Commands.runOnce(
            () -> io.setSetpoint(Meters.of(0.3))
        );
    }

    /** 
     * Retracts hopper all the way to 0.0m
     */
    public Command fullRetract() {
        return Commands.runOnce(
            () -> io.setSetpoint(Meters.of(0.0))
        );
    }

    /** 
     * Positive voltage extends, Negative voltage retracts (MAX of 0.3m and MIN of 0.0m)
     */
    public Command manualMove(double voltage) {
        return Commands.runOnce(() -> io.setMotorVoltage(voltage));
    }

    public Command stopMotor() {
        return Commands.runOnce(
            () -> io.stopMotor()
        );
    }

    public Distance getPosition() {
        return io.getPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);
        slider.setLength(io.getPosition().in(Meters));
        Logger.recordOutput("Hopper Slider/Mech", mechanism);
    }
}
