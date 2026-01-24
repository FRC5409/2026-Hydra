package frc.robot.subsystems.launcher;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface LauncherIO {

    @AutoLog
    class LauncherInputs {
        public boolean isLauncherConnected = false;
        public boolean isHoodConnected     = false;

        public double temperatureLauncher = 0.0;
        public double temperatureHood     = 0.0;

        public Voltage         launcherVoltage = Volts.of(0.0);
        public Voltage         hoodVoltage     = Volts.of(0.0);
        public Current         launcherCurrent = Current.ofBaseUnits(0.0, Amps);
        public Current         hoodCurrent     = Current.ofBaseUnits(0.0, Amps);
        public AngularVelocity launcherSpeed   = RadiansPerSecond.of(0.0);
        public AngularVelocity hoodSpeed       = RadiansPerSecond.of(0.0);

        public Angle targetHoodPosition = Degrees.of(0.0);
        public Angle hoodPosition       = Degrees.of(0.0);
    }

    default void setVoltage(double volts) {}

    default void runVelocity(double velocity) {}

    default void launchFuel() {}

    default void setHoodPos(Angle pos) {}

    default Angle getHoodPos() {
        return Degrees.of(0);
    }

    default void stop() {}

    default void updateInputs(LauncherInputs inputs) {}
}