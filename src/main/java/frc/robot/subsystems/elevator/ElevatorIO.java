package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.Meters;

public interface ElevatorIO {
    @AutoLog
    class ElevatorInputs {
        public boolean  isMainMotorConnected = false;
        public Voltage  mainAppliedVoltage   = Units.Volts.of(0.0);
        public Current  mainAppliedCurrent   = Units.Amps.of(0.0);
        public double   mainMotorTemperature = 0.0; // Celsius
        public Distance mainMotorPosition    = Units.Meters.of(0.0);
    }

    default void updateInputs(ElevatorInputs inputs) {}

    default void setMotorVoltage(double voltage) {}

    default void stopMotor() {}

    default void zeroEncoder() {}

    default Distance getPosition() {
        return Meters.of(0.0);
    }

    default void setSetpoint(Distance setpoint) {}
}