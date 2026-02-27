package frc.robot.subsystems.fuelGauage;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public interface FuelGauageIO {

    @AutoLog
    class FuelGauageInputs {
        public Distance distance = Inches.of(0.0);

        public Voltage voltage = Volts.of(0.0);
    }

    default Distance getDistance() {return Inches.of(0.0);}

    default void updateInputs(FuelGauageInputs inputs) {}
}