package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public interface HopperIO {
    /**
     * The hopper's inputs to be logged to AdvantageScope
     */
    @AutoLog
    public class HopperInputs {
        public boolean isMotorConnected = false;
        public Current appliedCurrent = Amps.of(0.0); // Current that we send to the motor
        public Current torqueCurrent = Amps.of(0.0); // Current read by how the motor torques
        public Voltage appliedVoltage = Volts.of(0.0);
        public double motorTemp = 0.0;
        public Distance motorPosition = Inches.of(0.0);
        public Distance motorPositionIntakeZero = Inches.of(0.0); // The motor's position relative to the intake
        public Distance setpoint = Inches.of(0.0);
        public boolean isCrashDetected = false;
    }

    public default void updateInputs(HopperInputs inputs) {}
    public default void setMotorVoltage(double voltage) {}
    public default void stopMotor() {}
    public default void zeroEncoder() {}

    public default void brakeMode() {}
    public default void coastMode() {}
    
    public default void setSetpoint(Distance setpoint) {}
    public default Distance getPosition() {return Inches.of(0.0);}
    public default Distance getPositionIntakeZero() {return Inches.of(0.0);}
    public default Distance getSetpoint() {return Inches.of(0.0);}

}