package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
public interface LauncherIO {

    @AutoLog
    public class LauncherInputs {
        public boolean launcherConnected = false;
        public boolean hoodConnected = false;

        public double temperatureLauncher = 0.0;
        public double temperatureHood = 0.0;

        public double volatgeLauncher = 0.0;
        public double voltageHood = 0.0;
        public double currentLauncher = 0.0;
        public double currentHood = 0.0;

        public double speedLauncher = 0.0;
        public double speedHood = 0.0;

        public double targetHoodPosition = 0.0;
        public double hoodPosition = 0.0;
    }

    public default void setVoltage(double volts) {}

    public default void launchFuel() {}

    public default void setHoodPos(Angle pos) {}

    public default Angle getHoodPos() {return Degrees.of(0);}

    public default void stop() {}

    public default void updateInputs(LauncherInputs inputs) {}
}