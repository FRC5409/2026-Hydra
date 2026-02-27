package frc.robot.subsystems.fuelGauage;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AnalogInput;

public class FuelGauageAnalogInput implements FuelGauageIO {
    private final AnalogInput ultrasonic;

    public FuelGauageAnalogInput (int channel) {
        ultrasonic = new AnalogInput(channel);
    }

    @Override
    public Distance getDistance() {
        return Inches.of(ultrasonic.getValue());
    }

    @Override
    public void updateInputs(FuelGauageInputs inputs) {
        inputs.distance = getDistance();
    }
}
