package frc.robot.subsystems.serializer;

import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;


public interface SerializerIO {
    @AutoLog
    public class SerializerInputs {
        public boolean isFloorMotorConnected = false;
        public Voltage floorAppliedVoltage = Volts.of(0.0);
        public Current floorAppliedCurrent = Amps.of(0.0);
        public double floorMotorTemperature = 0.0;
        public Distance floorMotorPosition = Meters.of(0.0);
        
        public boolean isFeederMotorConnected = false;
        public Voltage feederAppliedVoltage = Volts.of(0.0);
        public Current feederAppliedCurrent = Amps.of(0.0);
        public double feederMotorTemperature = 0.0;
        public Distance feederMotorPosition = Meters.of(0.0);
    }

    public default void updateInputs(SerializerInputs inputs) {}

    public default void setMotorVoltage(double voltage) {}

    public default void setIndexerMotorVoltage(double voltage) {}

    public default void setFeederMotorVoltage(double voltage) {}

    public default void stopMotor() {}

    public default void zeroEncoder() {}
}
