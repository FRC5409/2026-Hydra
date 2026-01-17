package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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

    private final DoubleSupplier positionSignal;

    public IntakeIOTalonFX(int rollerMotorId, int extensionMotorId) {
        rollerMotor = new TalonFX(rollerMotorId);
        extensionMotor = new TalonFX(extensionMotorId);
        rollerMotor.set(0.0);
        extensionMotor.set(0.0);
        positionControl = new PositionVoltage(0.0);
        positionControl.withSlot(0);

        TalonFXConfigurator rollerConfigurator = new TalonFXConfigurator();
        TalonFXConfiguration extensionConfigurator = new TalonFXConfiguration();

        extensionConfigurator.Slot0 = 
            new Slot0Configs()
                .withKP(0.0001)
                .withKI(0.0)
                .withKD(0.0);

        extensionConfigurator.MotorOutput = 
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        extensionConfigurator.CurrentLimits = 
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true);

        rollerConfigurator.Slot0 = 
            new Slot0Configs()
                .withKP(0.001)
                .withKI(0.0)
                .withKD(0.0);
        
        rollerConfigurator.MotorOutput = 
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        
        rollerConfigurator.CurrentLimits = 
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLimitEnable(true);

        // config.Feedback = 
        //     new FeedbackConfigs()
        //         .withSensorToMechanismRatio(ElevatorConstants.kSensorRatio);

        rollerConfigurator.apply(rollerMotor);
        extensionConfigurator.apply(extensionMotor);

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

        motor.optimizeBusUtilization();
    }

    @Override
    public void setVoltageCommand(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setSetpointCommand(Distance position) {
        motor.setControl(
          positionControl.withPosition(
            position.in(Meters)
          )
        );
    }

    @Override
    public Distance getPosition() {
        return Meters.of(positionSignal.getValueAsDouble());
    }

    @Override
    public void updateInputs(intakeInputsAutoLogged inputs) {
        
    }
}