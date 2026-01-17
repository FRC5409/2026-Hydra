package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;

public interface HopperIO {
    @AutoLog
    public class HopperInputs {
        public boolean mainMotorConnection = false;
        public double mainMotorCurrent = 0.0;
        public double mainMotorVoltage = 0.0;
        public double mainMotorTemp = 0.0;
        public double mainMotorPosition = 0.0;

        public boolean followerMotorConnection = false;
        public double followerMotorCurrent = 0.0;
        public double followerMotorVoltage = 0.0;
        public double followerMotorTemp = 0.0;
        public double followerMotorPosition = 0.0;
    }

    public default void updateInputs(HopperInputs inputs) {}
    public default void setMotorVoltage(double voltage) {}
    public default void stopMotor() {}
    public default void zeroEncoder() {}
    public default void setSetpoint(Distance setpoint) {}
    public default Distance getPosition() {return Meters.of(0.0);}

}