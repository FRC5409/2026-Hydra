package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.*;
import frc.robot.utils.Checkmate;
import frc.robot.utils.Checkmate.TestResult;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final IntakeIO intakeIO;
    private final IntakeInputsAutoLogged inputs;
    private Distance position;

    private static Pose3d extenderPose;

    public Intake(IntakeIO intakeIO) {

        this.intakeIO = intakeIO;
        this.inputs = new IntakeInputsAutoLogged();
        extenderPose = new Pose3d();

        Checkmate.register("Should fully extend Intake", () -> {

            double extendTarget = Extension.EXTENSION_DISTANCE.in(Meters);

            intakeIO.setSetpoint(Extension.EXTENSION_DISTANCE);

            Timer.delay(2.0);

            if (Math.abs(inputs.extensionPosition - extendTarget) > 0.05) {
                return TestResult.fail("Intake failed to extend, position: " + inputs.extensionPosition);
            }
            return TestResult.success("Intake extension ok");
        });

        Checkmate.register("Should fully retract Intake", () -> {

            double retractTarget = Extension.EXTENSION_MIN_DISTANCE.in(Meters);

            intakeIO.setSetpoint(Extension.EXTENSION_MIN_DISTANCE);

            Timer.delay(2.0);

            if (Math.abs(inputs.extensionPosition - retractTarget) > 0.05) {
                return TestResult.fail("Intake failed to retract, position: " + inputs.extensionPosition);
            }
            return TestResult.success("Intake retraction ok");
        });

        Checkmate.register("Should spin roller", () -> {

            intakeIO.setRollerVoltage(6.0);

            Timer.delay(2.0);

            double current = inputs.rollerCurrent.in(Amps);
            intakeIO.setRollerVoltage(0.0);
            if (Math.abs(current) < 1.0) {
                return TestResult.fail("Intake roller failed to spin up, current: " + current);
            }
            return TestResult.success("Intake roller ok, current: " + current);
        });
        ;

    }

/**
 * Spins the roller at the given voltage. Positive voltage should intake, negative voltage should outtake.
 * @param voltage The voltage to set the roller to, in volts. Should be between -12 and 12.
 * @return A command that sets the roller voltage when executed.
 */
    public Command setRollerVoltage(double voltage) {
        return Commands.runOnce(() -> intakeIO.setRollerVoltage(voltage), this);
    }
/**
 * Stops the roller by setting the voltage to 0.0 volts.
 * @return A command that stops the roller when executed.
 */
    public Command stopRoller() {
        return Commands.runOnce(() -> intakeIO.setRollerVoltage(0.0), this);
    }
/**
 * Sets the brake mode of the extension motor. In brake mode, the motor will resist being moved when no voltage is applied, which can help hold the intake in place when extended.
 * @return A command that sets the extension motor to brake mode when executed.
 */
    public Command brakemode() {
        return Commands.runOnce(() -> intakeIO.brakeMode(), this);
    }
/**
 * Extends the intake to the predefined extension distance. This command will set the extension motor's setpoint to the extension distance defined in IntakeConstants, which should cause the intake to extend when executed.
 * @return A command that extends the intake when executed.
 */
    public Command extend() {
        return Commands.runOnce(() -> intakeIO.setSetpoint(Extension.EXTENSION_DISTANCE), this);
    }
/**
 * Retracts the intake to the predefined minimum distance. This command will set the extension motor's setpoint to the minimum distance defined in IntakeConstants, which should cause the intake to retract when executed.
 * @return A command that retracts the intake when executed.
 */
    public Command retract() {
        return Commands.runOnce(() -> intakeIO.setSetpoint(Extension.EXTENSION_MIN_DISTANCE), this);
    }
/**
 * Moves the intake to a specific position. This command will set the extension motor's setpoint to the given position, which should cause the intake to move to that position when executed.
 * @param position The position to move the intake to.
 * @return A command that moves the intake to the specified position when executed.
 */
    public Command move(Distance position) {
        return Commands.runOnce(() -> intakeIO.setSetpoint(position), this);
    }
/**
 * Stops the extension motor by setting its voltage to 0.0 volts. This command will cause the intake to stop moving when executed, but it will not change the current setpoint of the extension motor, so if the intake is extended or retracted and then this command is executed, the intake will hold its position rather than moving back to a default position.
 * @return A command that stops the extension motor when executed.
 */
    public Command stopMotor() {
        return Commands.runOnce(() -> intakeIO.stopMotor(), this);
    }
/** * Sets the coast mode of the extension motor. In coast mode, the motor will not resist being moved when no voltage is applied, which can allow the intake to be moved more freely when extended.
 * @return A command that sets the extension motor to coast mode when executed.
 */
    public Command coastMode() {
        return Commands.runOnce(() -> intakeIO.coastMode(), this);
    }
/**
 * Sets the voltage of the extension motor directly. This command will set the voltage of the extension motor to the given voltage, which can be used for manual control of the intake extension when executed. Positive voltage should extend the intake, while negative voltage should retract it.
 * @param voltage The voltage to set the extension motor to.
 * @return A command that sets the extension motor voltage when executed.
 */
    public Command setExtensionVoltage(double voltage) {
        return Commands.runOnce(() -> intakeIO.setExtensionVoltage(voltage), this);
    }
/**
 * Gets the current position of the intake extension. This method will return the current position of the intake extension as a Distance object, which can be used for feedback control or monitoring the intake's position during operation.
 * @return The current position of the intake extension.
 */
    public Distance getPosition() {
        return intakeIO.getPosition();
    }
/**
 * The periodic method is called once per scheduler run and is responsible for updating the intake's state and logging relevant information. In this method, we first log the current inputs from the intake subsystem using the Logger utility. We then update the extenderPose variable based on the current extension position of the intake, which can be used for visualization or debugging purposes. Next, we check if the current drawn by the extension motor exceeds a predefined crash threshold, and if so, we set the position setpoint to the current position to hold it there and switch to coast mode to prevent damage. If the current is no longer above the threshold and we previously detected a crash, we reset the setpoint to the last known position and switch back to brake mode. Finally, we update the inputs from the intakeIO and log the extenderPose and PID controller values to the SmartDashboard for monitoring during operation.
 */
    @Override
    public void periodic() {
        Logger.processInputs("Intake", inputs);
        extenderPose = new Pose3d(

            inputs.extensionPosition, 0.0, 0.0,
            new Rotation3d(0.0, 0.0, Math.toRadians(0.0))

        );

        boolean overCurrent = inputs.extensionTorqueCurrent.gt(IntakeConstants.Extension.CRASH_CURRENT_THRESHOLD);
            
        if (DriverStation.isEnabled()){
            if (overCurrent && !inputs.isCrashDetected) { 
                position = getPosition();
                inputs.isCrashDetected = true;
                intakeIO.coastMode();
            } else if (!overCurrent && inputs.isCrashDetected) { 
                intakeIO.setSetpoint(position);
                inputs.isCrashDetected = false;
                intakeIO.brakeMode();
            }

        intakeIO.updateInputs(inputs);
        Logger.recordOutput("Components/Intake", extenderPose);
        SmartDashboard.putData("Intake/PID", Extension.PID);

        }
    
    }
}