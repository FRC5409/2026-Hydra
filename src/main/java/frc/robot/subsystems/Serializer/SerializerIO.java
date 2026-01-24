package frc.robot.subsystems.Serializer;

import org.littletonrobotics.junction.AutoLog;


public interface SerializerIO {
    @AutoLog
    public class SerializerInputs {
        public boolean floorMotorConnection = false;
        public double floorAppliedVoltage = 0.0;
        public double floorAppliedCurrent = 0.0;
        public double floorMotorTemperature = 0.0;
        public double floorMotorPosition = 0.0;
        
        public boolean feederMotorConnection = false;
        public double feederAppliedVoltage = 0.0;
        public double feederAppliedCurrent = 0.0;
        public double feederMotorTemperature = 0.0;
        public double feederMotorPosition = 0.0;
    }

    public default void updateInputs(SerializerInputs inputs) {}

    public default void setMotorVoltage(double voltage) {}

    public default void setIndexerMotorVoltage(double voltage) {}

    public default void setFeederMotorVoltage(double voltage) {}

    public default void stopMotor() {}

    public default void zeroEncoder() {}
}
