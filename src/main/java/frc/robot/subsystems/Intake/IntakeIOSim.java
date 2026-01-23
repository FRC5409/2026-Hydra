package frc.robot.subsystems.Intake;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.Intake.IntakeConstants.kExtension;


public class IntakeIOSim implements IntakeIO {
  
    private final ElevatorSim extensionSim;
    private final PIDController pid;
    private boolean running;
    private double rollerVoltage = 0.0;

    public IntakeIOSim() {

      // Ensure simulator has a battery voltage available
      RoboRioSim.setVInVoltage(12.0);

      extensionSim = new ElevatorSim(
            DCMotor.getKrakenX60(1), 
            kExtension.kGearing, 
            kExtension.INTAKE_MASS.in(Kilograms), 
            kExtension.INTAKE_DRUMRADIUS.in(Meters), 
            kExtension.INTAKE_MIN_DISTANCE.in(Meters), 
            kExtension.INTAKE_MAX_DISTANCE.in(Meters), 
            false, 
            kExtension.INTAKE_MIN_DISTANCE.in(Meters)
        );

      pid = new PIDController(
            kExtension.SIM_PIDConstants.kP, 
            kExtension.SIM_PIDConstants.kI, 
            kExtension.SIM_PIDConstants.kD
        );

      running = false;

    }

    @Override
    public void setExtensionVoltage(double voltage) {

      extensionSim.setInputVoltage(voltage);
      running = voltage != 0;

    }

    @Override
    public void setRollerVoltage(double voltage) {

      rollerVoltage = voltage;

    }

    @Override
    public void setSetpoint(Distance position) {

      pid.setSetpoint(position.in(Meters));
      running = true;

    }

    @Override
    public Distance getPosition() {

      return Meters.of(extensionSim.getPositionMeters());

    } 

    @Override
    public void updateInputs(intakeInputs inputs) {

        double volts = 0.0;

        if (running) {
            double pidOut = pid.calculate(extensionSim.getPositionMeters());
            // use RoboRioSim voltage if available, otherwise fallback to 12V
            double maxV = Math.max(12.0, RoboRioSim.getVInVoltage());
            volts = MathUtil.clamp(pidOut * 12.0, -maxV, maxV);
        }

      extensionSim.setInputVoltage(volts);
      extensionSim.update(0.02);

      inputs.extensionPosition = extensionSim.getPositionMeters();
      inputs.isExtended = extensionSim.getPositionMeters() >= kExtension.EXTENSION_MAX_DISTANCE.in(Meters) - 0.01;
      inputs.isRetracted = extensionSim.getPositionMeters() <= kExtension.EXTENSION_MIN_DISTANCE.in(Meters) + 0.01;
      inputs.extensionVelocity = extensionSim.getVelocityMetersPerSecond();
      // inputs.extensionCurrent = extensionSim.getCurrentDrawAmps().in(Amps);
      inputs.extensionRunning = running;
      inputs.extensionVolts = volts;
      inputs.extensionTemp = 25.0; // Constant temp for sim

      inputs.rollerCurrent = rollerVoltage / 12.0 * 20.0; // Simulated current draw
      inputs.rollerVolts = rollerVoltage;
      inputs.rollerTemp = 25.0;
      inputs.rollerVelocity = rollerVoltage / 12.0 * 5000.0; // Simulated velocity


      inputs.rollerConnection = true;
      inputs.extensionConnection = true;
    }
}