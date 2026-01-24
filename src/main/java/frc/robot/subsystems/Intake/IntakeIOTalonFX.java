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
import frc.robot.subsystems.Intake.IntakeConstants.Extension;

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

        TalonFXConfiguration extensionConfigurator = new TalonFXConfiguration();

        extensionConfigurator.Slot0 = new Slot0Configs()
            .withKP(Extension.TALONFX_PID.kP)
            .withKI(Extension.TALONFX_PID.kI)
            .withKD(Extension.TALONFX_PID.kD);

        extensionMotor.getConfigurator().apply(extensionConfigurator);

        extensionPositionSignal    = extensionMotor.getPosition();
        extensionTemperatureSignal = extensionMotor.getDeviceTemp();
        extensionVoltageSignal     = extensionMotor.getMotorVoltage();
        extensionCurrentSignal     = extensionMotor.getSupplyCurrent();

        rollerPositionSignal    = rollerMotor.getPosition();
        rollerTemperatureSignal = rollerMotor.getDeviceTemp();
        rollerVoltageSignal     = rollerMotor.getMotorVoltage();
        rollerCurrentSignal     = rollerMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(

            Extension.UPDATE_FREQUENCY,   
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

    public void setRollerVoltage(double voltage) {

        rollerMotor.setVoltage(voltage);

    }

    public void setExtensionVoltage(double voltage) {

        extensionMotor.setVoltage(voltage);

    }

    public void setSetpoint(Distance position) {

        extensionMotor.setControl(positionControl.withPosition(position.in(Meters)));

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
    public void updateInputs(IntakeIO.IntakeInputs inputs) {

        inputs.isExtensionConnected = true;
        inputs.extensionVolts = Volts.of(extensionVoltageSignal.getValueAsDouble());
        inputs.extensionCurrent = Amps.of(extensionCurrentSignal.getValueAsDouble());
        inputs.extensionTemp = 0.0;
        inputs.extensionPosition = extensionPositionSignal.getValueAsDouble();

        inputs.isRollerConnected = true;
        inputs.rollerVolts = Volts.of(rollerVoltageSignal.getValueAsDouble());
        inputs.rollerCurrent = Amps.of(rollerCurrentSignal.getValueAsDouble());
        inputs.rollerTemp = 0.0;

    }
}