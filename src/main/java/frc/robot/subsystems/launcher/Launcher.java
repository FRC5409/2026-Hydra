package frc.robot.subsystems.launcher;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.RotationsPerSecond;

public class Launcher extends SubsystemBase{
    private final LauncherIO io;
    private final LauncherInputsAutoLogged inputs;

    private static Pose3d launcherMech;
    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
        .getStructTopic(
            "MyPose", Pose3d.struct).publish();
    StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();

    public Launcher(LauncherIO io) {
        this.io = io;
        inputs = new LauncherInputsAutoLogged();

        launcherMech = new Pose3d();
    }

    // Voltage
    public Command launcherSetVoltage(double volts) {
        return Commands.runOnce(() -> io.launcherSetVoltage(volts), this);
    }

    public Command hoodSetVoltage(double volts) {
        return Commands.runOnce(() -> io.hoodSetVoltage(volts), this);
    }

    // Run system
    public Command runRPS(double velocity) {
        return Commands.runOnce(() -> io.runRPS(velocity));
    }

    public Command launchFuel(Distance distance) {
        var x = LauncherInterpolator.interpolate(distance);

        Logger.recordOutput("Launcher/targetSpeed", x.speed());
        Logger.recordOutput("Launcher/targetAngle", x.angle());

        return Commands.runOnce(() -> io.runRPS(-x.speed().in(RotationsPerSecond)), this);
    }

    public Command setHoodPos(Angle angle) {
        Logger.recordOutput("Hood/getHoodPos", io.getHoodPos());
        Logger.recordOutput("Hood/hoodPosition", inputs.hoodPosition);

        // return Commands.runOnce(() -> io.setHoodPos(angle));

        return Commands.sequence(
            Commands.runOnce(() -> io.setHoodPos(angle)),
            Commands.waitUntil(() -> io.getHoodPos().isNear(angle, 360)
            )
        );
    }

    // Getters
    public Angle getHoodPos() {
        return io.getHoodPos();
    }

    public double getDistance() {
        return io.getDistance();
    }

    // Stops
    public Command stopLauncher() {
        return Commands.runOnce(io::stopLauncher, this);
    }

    public Command stopHood() {
        return Commands.runOnce(io::stopHood, this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Launcher", inputs);
        Logger.recordOutput("Launcher Mech", launcherMech);

        SmartDashboard.putData("Launcher/PID", LauncherConstants.Launcher.PID);

        launcherMech = new Pose3d(7, 3, 0, new Rotation3d());

        publisher.set(launcherMech);
        arrayPublisher.set(new Pose3d[] {launcherMech, launcherMech});   
    }
}
