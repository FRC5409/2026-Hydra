package frc.robot.subsystems.Serializer;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;

public interface SerializerIO {
    @AutoLog
    public class SerializerInputs {
        public boolean mainMotorConnection = false;
        public double mainAppliedVoltage = 0.0;
        public double mainAppliedCurrent = 0.0;
        public double mainMotorTemperature = 0.0;
        public double mainMotorPosition = 0.0;
        
        public boolean followerMotorConnection = false;
        public double followerAppliedVoltage = 0.0;
        public double followerAppliedCurrent = 0.0;
        public double followerMotorTemperature = 0.0;
        public double followerMotorPosition = 0.0;
    }

    public default void updateInputs(SerializerInputs inputs) {}

    public default void setMotorVoltage(double voltage) {}

    public default void stopMotor() {}

    public default void zeroEncoder() {}

    public default Distance getPosition() {return Meters.of(0.0);}

    public default void setSetpoint(Distance setpoint) {}
}
