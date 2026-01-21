package frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final IntakeIO intakeIO;
    private final intakeInputsAutoLogged inputs;

    private static final double MECH_WIDTH_M = 0.5;
    private static final double MECH_HEIGHT_M = 2.0;
    private final Mechanism2d mech = new Mechanism2d(MECH_WIDTH_M, MECH_HEIGHT_M);
    private final MechanismLigament2d carriage;
    private final MechanismLigament2d platform;

    public Intake(IntakeIO io) {
        this.intakeIO = io;
        inputs = new intakeInputsAutoLogged();

        MechanismRoot2d root = mech.getRoot("IntakeRoot", 0.0, 0.0);
        carriage = root.append(new MechanismLigament2d("1", 0.05, 90.0));
        platform = root.append(new MechanismLigament2d("2", 0.2, 0.0));

        // Publish to SmartDashboard / Shuffleboard so AdvantageScope / dashboard can show it.
        SmartDashboard.putData("Intake Mechanism", mech);
    }

    public Command intakeCommand(double voltage) {
                // System.out.println("Intaking with voltage: " + voltage);
                return Commands.runOnce(() -> intakeIO.setVoltage(voltage), this);
    }

    public Command brakemodeCommand() {
        return Commands.runOnce(() -> intakeIO.brakeMode(), this);
    }

    public Command extendCommand() {
        //  System.out.println("Extending Intake");
         return Commands.runOnce(() -> intakeIO.setSetpoint(Meters.of(100)), this);
    }

    public Command retractCommand() {
        // System.out.println("Retracting Intake");
        return Commands.runOnce(() -> intakeIO.setSetpoint(Meters.of(0.0)), this);
    }

    @Override
    public void periodic() {
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        double posMeters = inputs.extensionPosition;
        double displayLen = Math.max(0.0, Math.min(MECH_HEIGHT_M, posMeters));

        // Update visualization (length in meters, angle in degrees)
        carriage.setLength(displayLen);
        platform.setAngle(0.0);
    }
}