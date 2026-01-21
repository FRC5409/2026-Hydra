package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Radians;

// import java.lang.System.Logger;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Launcher.LauncherIO.LauncherInputs;

public class Launcher extends SubsystemBase{
    private final LauncherIO io;
    private final LauncherInputsAutoLogged inputs;

    private static Pose3d launcherMech;
        StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
        .getStructTopic("MyPose", Pose3d.struct).publish();
        StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();

    public Launcher(LauncherIO io) {
        this.io = io;
        inputs = new LauncherInputsAutoLogged();

        launcherMech = new Pose3d();
    }

    public Angle getHoodPos() {
        return io.getHoodPos();
        
    }

    public Command launchFuel() {
        return Commands.runOnce(() -> io.launchFuel());
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
        Logger.processInputs("Launcher", inputs);
        Logger.recordOutput("Launcher Mech", launcherMech);

        launcherMech = new Pose3d(7, 3, 0, new Rotation3d());

        publisher.set(launcherMech);
        arrayPublisher.set(new Pose3d[] {launcherMech, launcherMech});   
    }
}
