package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.PrimitiveIterator.OfDouble;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
public interface LauncherIO {

    @AutoLog
    public class LauncherInputs {
        public boolean launcherConnected = false;
        public boolean hoodConnected = false;

        public double temperatureLauncher = 0.0;
        public double temperatureHood = 0.0;

        public Voltage volatgeLauncher = Volts.of(0.0);
        public Voltage voltageHood = Volts.of(0.0);
        public Current currentLauncher = Current.ofBaseUnits(0.0, Amps);
        public Current currentHood = Current.ofBaseUnits(0.0, Amps);
        public AngularVelocity speedLauncherRadians = RadiansPerSecond.of(0.0);
        public AngularVelocity speedHoodRadians = RadiansPerSecond.of(0.0);

        public Angle targetHoodPosition = Degrees.of(0.0);
        public Angle hoodPosition = Degrees.of(0.0);
    }

    public default void setVoltage(double volts) {}

    public default void runVelocity(double velocity) {}

    public default void launchFuel() {}

    public default void setHoodPos(Angle pos) {}

    public default Angle getHoodPos() {return Degrees.of(0);}

    public default void stop() {}

    public default void updateInputs(LauncherInputs inputs) {}
}