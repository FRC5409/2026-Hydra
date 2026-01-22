package frc.robot.subsystems.Intake;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Distance;


public class IntakeIOSim implements IntakeIO {

  private static final double MAX_VOLTAGE = 12.0;
  private double appliedVoltage = 0.0;
  private Distance position = Meters.of(0);

  // private final IntakeSimulation intakeSimulation;

  // public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
  //   this.intakeSimulation = new IntakeSimulation(
  //       "Fuel",
  //       driveTrain,
  //       // Width of the intake
  //       Meters.of(0.7),
  //       // The extension length of the intake beyond the robot's frame (when activated)
  //       Meters.of(0.2),
  //       // The intake is mounted on the back side of the chassis
  //       IntakeSimulation.IntakeSide.BACK,
  //       // The intake can hold up to 1 fuel
  //       1);
  // }

  @Override
  public void setVoltage(double voltage) {
    appliedVoltage = voltage;
    System.out.println("IntakeIOSim.setVoltage: voltage=" + voltage);
  }

  @Override
  public void setSetpoint(Distance setpoint) {
    position = setpoint;
    System.out.println("IntakeIOSim.setSetpoint: setpoint=" + setpoint.in(Meters));
  //   if (setpoint.in(Meters) > 0) {
  //     intakeSimulation.startIntake();
  //   } else {
  //     intakeSimulation.stopIntake();
  //   }
  // }

  // @Override
  //   public boolean isNoteInsideIntake() {
  //       return intakeSimulation.getGamePiecesAmount() != 0;
  //   }

  // @Override
  //   public void launchNote() {
  //       if (intakeSimulation.obtainGamePieceFromIntake())
  //           LauncherIOSim.launchNote();
  //   }
}

  @Override
  public Distance getPosition() {
    return position;
  }

  @Override
  public void stopMotor() {
    appliedVoltage = 0.0;
  }

  @Override
  public void updateInputs(IntakeIO.intakeInputs inputs) {
    inputs.extensionPosition = position.in(Meters);
    inputs.extensionVolts = appliedVoltage;

    inputs.extensionCurrent = Math.abs(inputs.extensionVolts) / MAX_VOLTAGE * 40.0; // crude estimate
    inputs.extensionTemp = 20.0 + inputs.extensionCurrent * 0.05; // crude estimate
    inputs.extensionConnection = true;

    inputs.rollerVolts = appliedVoltage;
    inputs.rollerCurrent = Math.abs(inputs.rollerVolts) / MAX_VOLTAGE * 20.0; // crude estimate
    inputs.rollerTemp = 20.0 + inputs.rollerCurrent * 0.05; // crude estimate
    inputs.rollerConnection = true;
    // System.out.println("IntakeIOSim.updateInputs: volts=" + inputs.extensionVolts + " pos=" + inputs.extensionPosition);
  }
}
