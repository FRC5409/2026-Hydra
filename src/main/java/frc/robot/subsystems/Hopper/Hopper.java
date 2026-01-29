package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

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

    public Command fullExtend() {
        return Commands.runOnce(
            () -> io.setSetpoint(Meters.of(0.3))
        );
    }

    public Command fullRetract() {
        return Commands.runOnce(
            () -> io.setSetpoint(Meters.of(0.0))
        );
    }

    public Command manualExtend() {
        return Commands.runOnce(() -> io.setMotorVoltage(2), this);
    }

    public Command manualRetract() {
        return Commands.runOnce(() -> io.setMotorVoltage(-2), this);
    }

    public Command stopMotor() {
        return Commands.runOnce(
            () -> io.stopMotor()
        );
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
