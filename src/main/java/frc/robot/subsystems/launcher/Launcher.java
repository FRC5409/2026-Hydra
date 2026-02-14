package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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

    public Launcher(LauncherIO io) {
        this.io = io;
        inputs = new LauncherInputsAutoLogged();

        launcherMech = new Pose3d();

        // create the logged fields
        logInterpolation(Meters.of(0), null);
    }

    // Voltage
    public Command launcherSetVoltage(double volts) {
        return Commands.runOnce(() -> io.launcherSetVoltage(volts), this);
    }

    // public Command hoodSetVoltage(double volts) {
    //     return Commands.runOnce(() -> io.hoodSetVoltage(volts), this);
    // }

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

    // public Command setHoodPos(Angle angle) {
    //     Logger.recordOutput("Hood/getHoodPos", io.getHoodPos());
    //     Logger.recordOutput("Hood/hoodPosition", inputs.hoodPosition);

    //     // return Commands.runOnce(() -> io.setHoodPos(angle));

    //     return Commands.sequence(
    //             Commands.runOnce(() -> io.setHoodPos(angle)),
    //             Commands.waitUntil(() -> io.getHoodPos().isNear(angle, 360)
    //             )
    //     );

    // }

    public Command setHoodPos(Distance setpoint) {
        // TODO: CHECK WHY THIS IS NECESSARY
        Logger.recordOutput("Hood/getHoodPos", io.getHoodPos());
        Logger.recordOutput("Hood/hoodPosition", inputs.hood1Position);

        // return Commands.runOnce(() -> io.setHoodPos(angle));

        // return Commands.sequence(
        //         Commands.runOnce(() -> io.setHoodPos(angle)),
        //         Commands.waitUntil(() -> io.getHoodPos().isNear(angle, 360)
        //         )
        // );

        // return Commands.run( () -> io.setHoodPos(setpoint));
        return Commands.run( () -> io.setHoodPos(setpoint));
        // .until(() -> inputs.targetHood1PositionMM == inputs.hood1Position);
        // .until(() -> io.getHoodPos().isNear(setpoint, 0.01));
    }

    public Command setHoodAngle(Angle setpoint) {
        // TODO: CHECK WHY THIS IS NECESSARY
        Logger.recordOutput("Hood/getHoodPos", io.getHoodPos());
        Logger.recordOutput("Hood/hoodPosition", inputs.hood1Position);

        // return Commands.runOnce(() -> io.setHoodPos(angle));

        // return Commands.sequence(
        //         Commands.runOnce(() -> io.setHoodPos(angle)),
        //         Commands.waitUntil(() -> io.getHoodPos().isNear(angle, 360)
        //         )
        // );

        // return Commands.runOnce( () -> io.setHoodAngle(setpoint));
        return Commands.run( () -> io.setHoodAngle(setpoint));
        // return Commands.runOnce(() -> io.setH)

        // .until(() -> io.getHoodPos().isNear(setpoint, 0.01));
    }

    public Command setHoodSpeed(double speed){
        // return Commands.run(() -> io.setHoodSpeed(speed));
        return Commands.runOnce(() -> io.setHoodSpeed(speed));
    }

    public Command setHoodPWM(int pwmMS){
        // return Commands.runOnce(() -> io.setHoodPWM(pwmMS));
        return Commands.run(() -> io.setHoodPWM(pwmMS));
    }

    // Getters
    public Distance getHoodPos() {
        return io.getHoodPos();
    }

    public double getDistance() {
        return io.getDistance();
    }

    // Stops
    public Command stopLauncher() {
        return Commands.runOnce(io::stopLauncher, this);
    }

    // public Command stopHood() {
    //     return Commands.runOnce(io::stopHood, this);
    // }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Launcher", inputs);
        Logger.recordOutput("Launcher Mech", launcherMech);

        SmartDashboard.putData("Launcher/PID", LauncherConstants.Launcher.PID);

        launcherMech = new Pose3d(7, 3, 0, new Rotation3d());
    }
}
