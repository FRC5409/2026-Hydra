package frc.robot.subsystems.launcher;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.launcher.interpolator.LaunchConfig;
import frc.robot.subsystems.launcher.interpolator.LaunchStrategy;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Launcher extends SubsystemBase {
    private final LauncherIO                io;
    private final LauncherInputsAutoLogged  inputs;

    private LaunchStrategy strategy;

    public Launcher(LauncherIO io) {
        this.io = io;
        inputs = new LauncherInputsAutoLogged();

        // create the logged fields
        logInterpolation(Meters.of(0), null);
        setStrategy(LauncherConstants.Launcher.DEFAULT_LAUNCH_STRATEGY);
    }

    public Command runVelocity(Supplier<AngularVelocity> velocity) {
        return Commands.runOnce(() -> io.runVelocity(velocity));
    }

    public Command launchFuel(Supplier<Distance> distance) {
        // ptr. to config; anon. fn. req. stable addr.
        AtomicReference<Optional<LaunchConfig>> config = new AtomicReference<>(Optional.empty());
        return Commands.runOnce(() -> {
            LaunchConfig c = strategy.interpolate(distance.get());
            logInterpolation(distance.get(), c);
            config.set(Optional.of(c)); // update ptr. for use in next cmd.
        }).andThen(runVelocity(() -> RotationsPerSecond.of(
                config.get()
                      .map(c -> c.speed().in(RotationsPerSecond))
                      .orElse(0.0)
        )));
    }

    private void logInterpolation(Distance distance, LaunchConfig config) {
        Logger.recordOutput("Launcher/TargetDistance", distance);
        Logger.recordOutput("Launcher/DidInterpolationSucceed", config != null);
        Logger.recordOutput("Launcher/TargetSpeed", config == null ? RotationsPerSecond.of(0) : config.speed());
        Logger.recordOutput("Launcher/TargetAngle", config == null ? Radians.of(0) : config.angle());
    }

    private Distance computeHoodExtension(Angle angle) {
        return (Distance)angle
                .timesConversionFactor(LauncherConstants.Hood.MM_PER_DEG)
                .minus(LauncherConstants.Hood.OFFSET_MM);
    }

    public Command setHoodAngle(Supplier<Angle> angle) {
        return Commands.runOnce(() -> io.setHoodExtension(computeHoodExtension(angle.get())));
    }

    // Getters
    public Angle getHoodAngle() {
        return (Angle)io.getHoodExtension()
                .plus(LauncherConstants.Hood.OFFSET_MM)
                .divideRatio(LauncherConstants.Hood.MM_PER_DEG);
    }

    public AngularVelocity getVelocity() {
        return io.getVelocity();
    }

    public Distance getUltrasonicDistance() {
        return io.getUltrasonicVolts().timesConversionFactor(LauncherConstants.Ultrasonic.MM_PER_VOLT);
    }

    // Stops
    public Command stopLauncher() {
        return Commands.runOnce(io::stopLauncher, this);
    }

    public Command stopHood() {
        return Commands.runOnce(io::stopHood, this);
    }

    public void setStrategy(LaunchStrategy strategy) {
        this.strategy = strategy;
        this.strategy.setLauncher(this);
        Logger.recordOutput("Launcher/LaunchStrategy", strategy.getName());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Launcher", inputs);
        SmartDashboard.putData("Launcher/PID", LauncherConstants.Launcher.PID);
    }
}
