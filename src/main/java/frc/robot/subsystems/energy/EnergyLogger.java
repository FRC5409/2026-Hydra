package frc.robot.subsystems.energy;

import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class EnergyLogger {
  private double totalCurrent = 0.0;
  private double totalPower = 0.0;
  private double totalEnergy = 0.0;
  private double batteryVoltage = EnergyConstants.BATTERY_VOLTAGE;

  private final Map<String, Double> subsystemEnergyTotals = new HashMap<>();
  private final Map<String, Double> subsystemCurrentTotals = new HashMap<>();
  private final Map<String, Double> subsystemPowerTotals = new HashMap<>();

  public void setBatteryVoltage(double volts) {
    this.batteryVoltage = volts;
  }

  public void reportCurrentUsage(String name, double... amps) {
    double current = 0.0;
    for (double a : amps) current += Math.abs(a);
    
    double power = current * batteryVoltage;
    double energyJoules = power * EnergyConstants.LOOP_PERIOD_SECS;

    totalCurrent += current;
    totalPower += power;
    totalEnergy += energyJoules;

    subsystemEnergyTotals.merge(name, energyJoules, Double::sum);
    subsystemCurrentTotals.merge(name, current, Double::sum);
    subsystemPowerTotals.merge(name, power, Double::sum);

    Logger.recordOutput("Energy/" + name + "/Current", current, "amps");
    Logger.recordOutput("Energy/" + name + "/Power", power, "watts");
    Logger.recordOutput("Energy/" + name + "/Energy", energyJoules / 3600.0, "watt hours");

    double cumulativeSubsystemWh = subsystemEnergyTotals.get(name) / 3600.0;
    Logger.recordOutput("Energy/" + name + "/CumulativeEnergy", cumulativeSubsystemWh, "watt hours");
  }

  public void periodicAfterScheduler() {
    Logger.recordOutput("Energy/Total/Current", totalCurrent, "amps");
    Logger.recordOutput("Energy/Total/Power", totalPower, "watts");
    Logger.recordOutput("Energy/Total/TotalEnergy", totalEnergy / 3600.0, "watt hours");
    
    totalCurrent = 0.0;
    totalPower = 0.0;
  }

  public void resetTotals() {
    totalEnergy = 0.0;
    subsystemEnergyTotals.clear();
  }

  public double getTotalCurrent() { return totalCurrent; }
  public double getTotalPower() { return totalPower; }
  public double getTotalEnergy() { return totalEnergy; }
}