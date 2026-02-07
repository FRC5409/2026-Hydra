package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface FeederIO {
    @AutoLog
    public class FeederInputs {
        public boolean isFeederMotorConnected = false;
        public Voltage feederAppliedVoltage = Volts.of(0.0);
        public Current feederAppliedCurrent = Amps.of(0.0);
        public double feederMotorTemperature = 0.0;
        public Angle feederMotorPosition = Degrees.of(0.0);
        public AngularVelocity feederMotorVelocity = RotationsPerSecond.of(0.0);
    }

    public default void updateInputs(FeederInputs inputs) {}

    public default void setMotorVoltage(double voltage) {}

     public default void runRPS(double velocity) {}

     public default void stopMotor() {}

     public default void zeroFeederEncoder() {}
     
     public default AngularVelocity getVelocityRPS() {return RotationsPerSecond.of(0);}

     
}