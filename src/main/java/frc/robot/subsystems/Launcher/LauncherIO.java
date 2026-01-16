package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
public interface LauncherIO {

    @AutoLog
    public class LauncherInputs {
        public boolean isConnected = false;

        public double temperature = 0.0;

        public double volatge = 0.0;
        public double current = 0.0;

        public double speed = 0.0;

        public double targetPosition = 0.0;
        public double position = 0.0;
    }

    public default void setVoltage(double volts) {}

    public default void prepareFuel() {} // ?

    public default void shootFuel() {} // ?

    public default void setHoodPos(Angle pos) {}

    public default Angle getHoodPos() {return Degrees.of(0);}

    public default void stop() {}

    public default void updateInputs(LauncherIO inputs) {}
    
}