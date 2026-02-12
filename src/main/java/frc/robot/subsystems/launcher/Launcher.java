package frc.robot.subsystems.launcher;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

// import java.lang.System.Logger;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
// import java.lang.System.Logger;
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

import static edu.wpi.first.units.Units.*;

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

        // create the logged fields
        logInterpolation(Meters.of(0), null);
    }

    
    public Command setServoPos(double pos) {
        return Commands.runOnce(() -> io.setServoPos(pos));
    }

    public double getServoPos() {
        return io.getServoPos();
    }

    public Angle getHoodPos() {
        return io.getHoodPos();
    }

    public double getDistance() {
        return io.getDistance();
    }

    public Command launchFuel(Supplier<Distance> distance) {
        // ptr. to config; anon. fn. req. stable addr.
        AtomicReference<Optional<LauncherInterpolator.LaunchConfig>> config = new AtomicReference<>(Optional.empty());
        return Commands.runOnce(() -> {
            LauncherInterpolator.LaunchConfig c = LauncherInterpolator.interpolate(distance.get());
            logInterpolation(distance.get(), c);
            config.set(Optional.of(c)); // update ptr. for use in next cmd.
        }).andThen(runVelocity(RotationsPerSecond.of(
                config.get()
                      .map(c -> c.speed().in(RotationsPerSecond))
                      .orElse(0.0)
        )));
    }

    private void logInterpolation(Distance distance, LauncherInterpolator.LaunchConfig config) {
        Logger.recordOutput("Launcher/TargetDistance", distance);
        Logger.recordOutput("Launcher/DidInterpolationSucceed", config != null);
        Logger.recordOutput("Launcher/TargetSpeed", config == null ? RotationsPerSecond.of(0) : config.speed());
        Logger.recordOutput("Launcher/TargetAngle", config == null ? Radians.of(0) : config.angle());
    }

    public Command moveHood(Angle angle) {
        return Commands.sequence(
            Commands.runOnce(() -> io.setHoodPos(angle), this),
            Commands.waitUntil(() -> io.getHoodPos().isNear(angle, 1.0))
        );
    }

    public Command setVoltage(DoubleSupplier volts) {
        return Commands.runOnce(() -> io.setVoltage(volts.getAsDouble()), this);
    }

    public Command runVelocity(Supplier<AngularVelocity> velocity) {
        return Commands.runOnce(() -> io.runVelocity(velocity.get().in(RotationsPerSecond)), this);
    }

    public Command runVelocity(AngularVelocity velocity) {
        return Commands.runOnce(() -> io.runVelocity(velocity.in(RotationsPerSecond)), this);
    }

    public Command stop() {
        return Commands.runOnce(io::stop, this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Launcher", inputs);
        Logger.recordOutput("Launcher Mech", launcherMech);

        // SmartDashboard.putData("Launcher/PID", LauncherConstants.PID);

        launcherMech = new Pose3d(7, 3, 0, new Rotation3d());

        publisher.set(launcherMech);
        arrayPublisher.set(new Pose3d[] {launcherMech, launcherMech});
    }
}
