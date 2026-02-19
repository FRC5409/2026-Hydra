package frc.robot.subsystems.launcher;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public interface LauncherIO {

    @AutoLog
    class LauncherInputs {
        // Launcher
        public boolean         isLauncherConnected = false;
        public double          launcherTemperature = 0.0;
        public Voltage         launcherVoltage     = Volts.of(0.0);
        public Current         launcherCurrent     = Amps.of(0);
        public AngularVelocity launcherVelocity    = RotationsPerSecond.of(0.0);

        public boolean         isLauncherFollowerConnected = false;
        public double          launcherFollowerTemperature = 0.0;
        public Voltage         launcherFollowerVoltage     = Volts.of(0.0);
        public Current         launcherFollowerCurrent     = Amps.of(0);
        public AngularVelocity launcherFollowerVelocity    = RotationsPerSecond.of(0.0);

        // Hood
        public boolean         isHoodConnected = false;
        public double          hoodTemperature = 0.0;
        public Voltage         hoodVoltage     = Volts.of(0.0);
        public Current         hoodCurrent     = Amps.of(0);
        public AngularVelocity hoodVelocity    = RotationsPerSecond.of(0.0);
        public Angle           hoodPosition    = Radians.of(0.0);
    }

    // Launcher
    default void launcherSetVoltage(double volts) {}

    default void runVelocity(Supplier<AngularVelocity> velocity) {}

    default void stopLauncher() {}

    // Hood
    default void hoodSetVoltage(double volts) {}

    default void setHoodPos(Angle pos) {}

    default Angle getHoodPos() {
        return Degrees.of(0);
    }

    default AngularVelocity getVelocity() {
        return RotationsPerSecond.of(0);
    }

    default void stopHood() {}

    // Ultrasonic sensor
    default double getDistance() {
        return 0.0;
    }

    // Shared 
    default void updateInputs(LauncherInputs inputs) {}
}