package frc.robot.util;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

/**
 * Utility for logging system-wide power consumption
 *
 * @author Logan Dhillon
 */
public class PowerLogger {
    private final PowerDistributionIO power;

    public PowerLogger(PowerDistributionIO pdh) {
        this.power = pdh;
    }

    public void update() {
        Voltage volts = power.getBatteryVoltage();
        Current current = power.getTotalCurrent();

        Logger.recordOutput("Power/BatteryVoltage", volts);
        Logger.recordOutput("Power/TotalCurrent", current);
        Logger.recordOutput("Power/Wattage", volts.times(current));
    }

    public interface PowerDistributionIO {
        Current getTotalCurrent();

        Voltage getBatteryVoltage();
    }
}
