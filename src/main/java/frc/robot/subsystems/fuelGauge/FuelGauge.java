package frc.robot.subsystems.fuelGauge;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelGauge extends SubsystemBase{
    private final FuelGaugeIO io;
    private final FuelGaugeInputsAutoLogged inputs;

    public FuelGauge(FuelGaugeIO io) {
        this.io = io;
        inputs = new FuelGaugeInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Fuel Gauge", inputs);
    }
}