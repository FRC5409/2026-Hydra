package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Launcher.LauncherIO.LauncherInputs;

public class Launcher extends SubsystemBase{
    private final LauncherIO io;
    private final LauncherInputsAutoLogged inputs;

    public Launcher(LauncherIO io) {
        this.io = io;
        inputs = new LauncherInputsAutoLogged();
    }

    public Angle getHoodPos() {
        return io.getHoodPos();
        
    }

    public Command moveHood(Angle angle) {
        String angleAsString = angle.toString();
        return Commands.sequence(
            Commands.runOnce(() -> io.setHoodPos(angle), this),
            Commands.waitUntil(() -> io.getHoodPos().isNear(Radians.of(Integer.getInteger(angleAsString)), 1.0))
        );
    }

    public Command setVoltage(double volts) {
        return Commands.runOnce(() -> io.setVoltage(volts), this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // Logger.processInputs("Launcher", inputs);
    }
}
