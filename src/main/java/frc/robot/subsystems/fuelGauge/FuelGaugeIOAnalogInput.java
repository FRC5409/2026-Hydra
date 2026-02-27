package frc.robot.subsystems.fuelGauge;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.subsystems.fuelGauge.FuelGaugeIO.FuelGaugeInputs;

public class FuelGaugeIOAnalogInput implements FuelGaugeIO {
    private final AnalogInput ultrasonic;

    public FuelGaugeIOAnalogInput (int channel) {
        ultrasonic = new AnalogInput(channel);
    }

    @Override
    public Distance getDistance() {
        return Inches.of(ultrasonic.getValue());
    }

    @Override
    public void updateInputs(FuelGaugeInputs inputs) {
        inputs.distance = getDistance();
    }
}
