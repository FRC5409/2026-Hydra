package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Launcher extends SubsystemBase {
    private final LauncherIO               io;
    private final LauncherInputsAutoLogged inputs;

    private static Pose3d launcherMech;
    private static Pose3d hooPose3d;

    public Launcher(LauncherIO io) {
        this.io = io;
        inputs = new LauncherInputsAutoLogged();

        launcherMech = new Pose3d();
        hooPose3d = new Pose3d();

        // create the logged fields
        logInterpolation(Meters.of(0), null);
    }

    // Voltage
    public Command launcherSetVoltage(double volts) {
        return Commands.runOnce(() -> io.launcherSetVoltage(volts), this);
    }

    public Command hoodSetVoltage(double volts) {
        return Commands.runOnce(() -> io.hoodSetVoltage(volts), this);
    }

    public Command runRPS(Supplier<AngularVelocity> velocity) {
        return Commands.runOnce(() -> io.runRPS(velocity));
    }

    public Command launchFuel(Supplier<Distance> distance) {
        // ptr. to config; anon. fn. req. stable addr.
        AtomicReference<Optional<LauncherInterpolator.LaunchConfig>> config = new AtomicReference<>(Optional.empty());
        return Commands.runOnce(() -> {
            LauncherInterpolator.LaunchConfig c = LauncherInterpolator.interpolate(distance.get());
            logInterpolation(distance.get(), c);
            config.set(Optional.of(c)); // update ptr. for use in next cmd.
        }).andThen(runRPS(() -> RotationsPerSecond.of(
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

    public Command setHoodPos(Angle angle) {
        return Commands.sequence(
                Commands.runOnce(() -> io.setHoodPos(angle)),
                Commands.waitUntil(() -> io.getHoodPos().isNear(angle, 0.01)
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

        Logger.recordOutput("Hood/getHoodPos", io.getHoodPos());
        Logger.recordOutput("Hood/hoodPosition", inputs.hoodPosition);

        SmartDashboard.putData("Launcher/PID", LauncherConstants.Launcher.PID);

        launcherMech = new Pose3d(new Translation3d(), new Rotation3d(inputs.launcherRPM, 0, 0));
        hooPose3d = new Pose3d(new Translation3d(), new Rotation3d(0, getHoodPos().in(Radians), 0));
    }
}
