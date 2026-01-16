package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;

public interface HopperIO {
    @AutoLog
    public class HopperInputs {
        public boolean motorConnection = false;
        public double motorCurrent = 0.0;
        public double motorVoltage = 0.0;
        public double motorTemp = 0.0;
        public double motorPosition = 0.0;
    }

    public default void updateInputs(HopperInputs inputs) {}
    public default void setMotorVoltage(double voltage) {}
    public default void stopMotor() {}
    public default void zeroEncoder() {}
    public default void setSetpoint(Distance setpoint) {}
    public default Distance getPosition() {return Meters.of(0.0);}

}