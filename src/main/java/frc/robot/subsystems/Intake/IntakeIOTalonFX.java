package frc.robot.subsystems.Intake;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public final class IntakeIOTalonFX implements IntakeIO {
  
    private final TalonFX rollerMotor;
    private final TalonFX extensionMotor;

    private final PositionVoltage positionControl;

    private final StatusSignal<Angle>       rollerPositionSignal;
    private final StatusSignal<Temperature> rollerTemperatureSignal;
    private final StatusSignal<Voltage>     rollerVoltageSignal;
    private final StatusSignal<Current>     rollerCurrentSignal;

    private final StatusSignal<Angle>       extensionPositionSignal;
    private final StatusSignal<Temperature> extensionTemperatureSignal;
    private final StatusSignal<Voltage>     extensionVoltageSignal;
    private final StatusSignal<Current>     extensionCurrentSignal;

    public IntakeIOTalonFX(int rollerMotorId, int extensionMotorId) {
        rollerMotor = new TalonFX(rollerMotorId);
        extensionMotor = new TalonFX(extensionMotorId);
        rollerMotor.set(0.0);
        extensionMotor.set(0.0);

        positionControl = new PositionVoltage(0.0);
        positionControl.withSlot(0);

        TalonFXConfiguration rollerConfigurator = new TalonFXConfiguration();
        TalonFXConfiguration extensionConfigurator = new TalonFXConfiguration();

        extensionConfigurator.Slot0 = new Slot0Configs()
            .withKP(1.0)
            .withKI(0.0)
            .withKD(0.0);

        rollerConfigurator.Slot0 = new Slot0Configs()
            .withKP(rollerMotorId)
            .withKI(rollerMotorId)
            .withKD(rollerMotorId);

        extensionPositionSignal    = extensionMotor.getPosition();
        extensionTemperatureSignal = extensionMotor.getDeviceTemp();
        extensionVoltageSignal     = extensionMotor.getMotorVoltage();
        extensionCurrentSignal     = extensionMotor.getSupplyCurrent();

        rollerPositionSignal    = rollerMotor.getPosition();
        rollerTemperatureSignal = rollerMotor.getDeviceTemp();
        rollerVoltageSignal     = rollerMotor.getMotorVoltage();
        rollerCurrentSignal     = rollerMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50, 
            extensionPositionSignal,
            extensionTemperatureSignal,
            extensionVoltageSignal,
            extensionCurrentSignal,

            rollerPositionSignal,
            rollerTemperatureSignal,
            rollerVoltageSignal,
            rollerCurrentSignal
        );

        rollerMotor.optimizeBusUtilization();
        extensionMotor.optimizeBusUtilization();
    }

    public void setVoltage(double voltage) {
        rollerMotor.setVoltage(voltage);
    }

    public void setSetpoint(Distance position) {
        extensionMotor.setControl(
          positionControl.withPosition(
            position.in(Meters)
          )
        );
    }

    public void coastMode() {
        rollerMotor.setNeutralMode(NeutralModeValue.Coast);
        extensionMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void brakeMode() {
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);
        extensionMotor.setNeutralMode(NeutralModeValue.Brake);
    }   

    @Override
    public void updateInputs(IntakeIO.intakeInputs inputs) {
        inputs.extensionConnection = true;
        inputs.extensionVolts = extensionVoltageSignal.getValueAsDouble();
        inputs.extensionCurrent = extensionCurrentSignal.getValueAsDouble();
        inputs.extensionTemp = extensionTemperatureSignal.getValueAsDouble();
        inputs.extensionPosition = extensionPositionSignal.getValueAsDouble();

        inputs.rollerConnection = true;
        inputs.rollerVolts = rollerVoltageSignal.getValueAsDouble();
        inputs.rollerCurrent = rollerCurrentSignal.getValueAsDouble();
        inputs.rollerTemp = rollerTemperatureSignal.getValueAsDouble();
    }
}