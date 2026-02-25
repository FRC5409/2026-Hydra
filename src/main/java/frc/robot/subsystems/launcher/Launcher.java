package frc.robot.subsystems.launcher;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.launcher.interpolator.LaunchConfig;
import frc.robot.subsystems.launcher.interpolator.LaunchStrategy;
import frc.robot.util.MathUtils;
import frc.robot.utils.Checkmate;
import org.littletonrobotics.junction.Logger;

import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Launcher extends SubsystemBase {
    private final LauncherIO                io;
    private final LauncherInputsAutoLogged  inputs;
    private final AtomicReference<Distance> hoodSetpoint = new AtomicReference<>(Millimeters.of(0.0));

    private LaunchStrategy strategy;

    public Launcher(LauncherIO io) {
        this.io = io;
        inputs = new LauncherInputsAutoLogged();

        // create the logged fields
        logInterpolation(Meters.of(0), null);
        setStrategy(LauncherConstants.Launcher.DEFAULT_LAUNCH_STRATEGY);

        Checkmate.register(
                "Should launch fuel", () -> {
                    Distance d = Meters.of(2.0);
                    var config = strategy.interpolate(d);

                    CommandScheduler.getInstance().schedule(this.launchFuel(() -> d));

                    return MathUtils.withinTolerance(
                            getVelocity().in(RotationsPerSecond), config.speed().in(RotationsPerSecond), 0.05) ?
                           Checkmate.TestResult.success() :
                           Checkmate.TestResult.fail(
                                   "Launcher not fast enough (" + getVelocity().in(RotationsPerSecond) + " RPS)");
                });
    }

    public Command runVelocity(Supplier<AngularVelocity> velocity) {
        return Commands.runOnce(() -> io.runVelocity(velocity));
    }

    /**
     * Defers a command that interpolates a {@link LaunchConfig} and then sets the velocity and hood angle of the
     * launcher, based on the active {@link LaunchStrategy}.
     *
     * @param distance supplier to get the distance that fuel should be shot from
     *
     * @return defered command that launches fuel
     */
    public Command launchFuel(Supplier<Distance> distance) {
        return Commands.defer(
                () -> {
                    LaunchConfig c = strategy.interpolate(distance.get());
                    logInterpolation(distance.get(), c);

                    return runVelocity(c::speed).alongWith(setHoodAngle(c::angle));
                }, Set.of(this));
    }

    private void logInterpolation(Distance distance, LaunchConfig config) {
        Logger.recordOutput("Launcher/Interpolator/TargetDistance", distance);
        Logger.recordOutput("Launcher/Interpolator/DidInterpolationSucceed", config != null);
        Logger.recordOutput(
                "Launcher/Interpolator/TargetSpeed", config == null ? RotationsPerSecond.of(0) : config.speed());
        Logger.recordOutput("Launcher/Interpolator/TargetAngle", config == null ? Radians.of(0) : config.angle());
    }

    private Distance computeHoodExtension(Angle angle) {
        // clamp between min and max
        double theta = angle.in(Degrees);
        theta = Math.max(LauncherConstants.Hood.MIN_ANGLE_DEG, Math.min(LauncherConstants.Hood.MAX_ANGLE_DEG, theta));

        return (Distance)Degrees.of(theta)
                                .timesConversionFactor(LauncherConstants.Hood.MM_PER_DEG)
                                .minus(LauncherConstants.Hood.OFFSET_MM);
    }

    public Command setHoodAngle(Supplier<Angle> angle) {
        return Commands.runOnce(() -> hoodSetpoint.set(computeHoodExtension(angle.get())));
    }

    // Getters
    public Angle getHoodAngle() {
        return Degrees.of(
                io.getHoodExtension()
                  .plus(LauncherConstants.Hood.OFFSET_MM)
                  .divideRatio(LauncherConstants.Hood.MM_PER_DEG)
                  .in(Degrees));
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

    public void setStrategy(LaunchStrategy strategy) {
        this.strategy = strategy;
        this.strategy.setLauncher(this);
        Logger.recordOutput("Launcher/Interpolator/LaunchStrategy", strategy.getName());
    }

    @Override
    public void periodic() {
        // update hood
        if (DriverStation.isEnabled()) io.updateHood(hoodSetpoint.get());

        // update inputs
        io.updateInputs(inputs);
        Logger.processInputs("Launcher", inputs);
        SmartDashboard.putData("Launcher/PID", LauncherConstants.Launcher.PID);
    }
}
