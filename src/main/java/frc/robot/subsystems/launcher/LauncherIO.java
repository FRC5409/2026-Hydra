package frc.robot.subsystems.launcher;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface LauncherIO {

    @AutoLog
    class LauncherInputs {
        // Launcher
        public boolean isLauncherConnected = false;
        public double temperatureLauncher = 0.0;
        public Voltage         launcherVoltage = Volts.of(0.0);
        public Current         launcherCurrent = Current.ofBaseUnits(0.0, Amps);
        public AngularVelocity launcherSpeedRadians   = RadiansPerSecond.of(0.0);
        public double launcherRPM = Units.radiansPerSecondToRotationsPerMinute(launcherSpeedRadians.baseUnitMagnitude());

        public boolean isLauncherFollowerConnected = false;
        public double launcherFollowerTemperature = 0.0;
        public Voltage         launcherFollowerVoltage = Volts.of(0.0);
        public Current         launcherFollowerCurrent = Current.ofBaseUnits(0.0, Amps);
        public AngularVelocity launcherFollowerSpeedRadians   = RadiansPerSecond.of(0.0);
        public double launcherFollowerRPM = Units.radiansPerSecondToRotationsPerMinute(launcherSpeedRadians.baseUnitMagnitude());
        
        // Hood
        public boolean isHoodConnected     = false;
        public double temperatureHood     = 0.0;
        public Voltage         hoodVoltage     = Volts.of(0.0);
        public Current         hoodCurrent     = Current.ofBaseUnits(0.0, Amps);
        public AngularVelocity hoodSpeedRadians       = RadiansPerSecond.of(0.0);
        public Angle hoodPosition       = Degrees.of(0.0);
        public Angle targetHoodPosition = Degrees.of(0.0);

        public double velocitySetpoint = 0.0;
    }

    // Launcher
    default void launcherSetVoltage(double volts) {}

    default void runRPS(double velocity) {}

    default void launchFuel(Distance distance) {}

    default void stopLauncher() {}

    // Hood
    default void hoodSetVoltage(double volts) {}

    default void setHoodPos(Angle pos) {}

    default Angle getHoodPos() {return Degrees.of(0);}

    default void stopHood() {}

    // Shared 
    default void updateInputs(LauncherInputs inputs) {}
}