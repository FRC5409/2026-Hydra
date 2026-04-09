package frc.robot.util;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

/**
 * Utility for logging system-wide power consumption
 *
 * @author Logan Dhillon
 */
public class PowerLogger {
    private final PowerDistributionIO power;

    private Current peakCurrent     = Amps.of(0);
    private Voltage minVoltage      = Volts.of(Double.MAX_VALUE);
    private double  energyWattHours = 0.0;
    private double  lastTimestamp   = System.currentTimeMillis() / 1000.0;

    public PowerLogger(PowerDistributionIO pdh) {
        this.power = pdh;
    }

    public void update() {
        Voltage volts = power.getBatteryVoltage();
        Current current = power.getTotalCurrent();
        Power power = volts.times(current);

        double now = System.currentTimeMillis() / 1000.0;
        double dt = now - lastTimestamp;
        lastTimestamp = now;

        if (current.gt(peakCurrent)) peakCurrent = current;
        if (volts.lt(minVoltage)) minVoltage = volts;

        energyWattHours += (power.in(Watts) * dt) / 3600.0;

        Logger.recordOutput("Power/BatteryVoltage", volts);
        Logger.recordOutput("Power/TotalCurrent", current);
        Logger.recordOutput("Power/Wattage", power);

        Logger.recordOutput("Power/PeakCurrent", peakCurrent);
        Logger.recordOutput("Power/EnergyWattHours", energyWattHours, "Wh");
        Logger.recordOutput("Power/MinVoltage", minVoltage);
    }

    public interface PowerDistributionIO {
        Current getTotalCurrent();

        Voltage getBatteryVoltage();
    }
}
