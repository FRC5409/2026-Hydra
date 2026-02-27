package frc.robot.subsystems.fuelGauage;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelGauage extends SubsystemBase{
    private final FuelGauageIO io;
    private final FuelGauageInputsAutoLogged inputs;

    public FuelGauage(FuelGauageIO io) {
        this.io = io;
        inputs = new FuelGauageInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Fuel Gauge", inputs);
    }
}