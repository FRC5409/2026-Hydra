package frc.robot.subsystems.serializer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class SerializerIOTalonFX implements SerializerIO {
    
    private final TalonFX m_indexerMotor;
    private final TalonFX m_BottomFeederMotor;

    private final TalonFXConfigurator indexerMotorConfig;
    private final TalonFXConfigurator bottomFeederMotorConfig;

    private CurrentLimitsConfigs currentConfigs;

    private StatusSignal<AngularVelocity> indexerDeviceVelocity;
    private StatusSignal<Angle> indexerDevicePosition;
    private StatusSignal<Voltage> indexerDeviceVoltage;
    private StatusSignal<Current> indexerDeviceCurrent;
    private StatusSignal<Temperature> indexerDeviceTemp;

    private StatusSignal<AngularVelocity> bottomFeederDeviceVelocity;
    private StatusSignal<Angle> bottomFeederDevicePosition;
    private StatusSignal<Voltage> bottomFeederDeviceVoltage;
    private StatusSignal<Current> bottomFeederDeviceCurrent;
    private StatusSignal<Temperature> bottomFeederDeviceTemp;


    public SerializerIOTalonFX(int indexerId, int bottomFeederId) {
        m_indexerMotor = new TalonFX(indexerId);
        m_BottomFeederMotor = new TalonFX(bottomFeederId);

        indexerMotorConfig = m_indexerMotor.getConfigurator();
        bottomFeederMotorConfig = m_BottomFeederMotor.getConfigurator();

        currentConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(SerializerConstants.TALON_FX_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);
        indexerMotorConfig.apply(currentConfigs);
        bottomFeederMotorConfig.apply(currentConfigs);
        
        indexerMotorConfig.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        m_indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        m_BottomFeederMotor.setNeutralMode(NeutralModeValue.Brake);

        m_BottomFeederMotor.setControl(new Follower(indexerId, MotorAlignmentValue.Opposed));

        indexerDeviceVelocity = m_indexerMotor.getVelocity();
        indexerDevicePosition = m_indexerMotor.getPosition();
        indexerDeviceVoltage = m_indexerMotor.getMotorVoltage();
        indexerDeviceCurrent = m_indexerMotor.getSupplyCurrent();
        indexerDeviceTemp = m_indexerMotor.getDeviceTemp();

        bottomFeederDeviceVelocity = m_BottomFeederMotor.getVelocity();
        bottomFeederDevicePosition = m_BottomFeederMotor.getPosition();
        bottomFeederDeviceVoltage = m_BottomFeederMotor.getMotorVoltage();
        bottomFeederDeviceCurrent = m_BottomFeederMotor.getSupplyCurrent();
        bottomFeederDeviceTemp = m_BottomFeederMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            indexerDevicePosition,
            indexerDeviceVelocity,
            indexerDeviceVoltage,
            indexerDeviceCurrent,
            indexerDeviceTemp,

            bottomFeederDevicePosition,
            bottomFeederDeviceVelocity,
            bottomFeederDeviceVoltage,
            bottomFeederDeviceCurrent,
            bottomFeederDeviceTemp
        );

        m_indexerMotor.optimizeBusUtilization();
        m_BottomFeederMotor.optimizeBusUtilization();
    }

    @Override
    public void setVoltage(double voltage) {
        m_indexerMotor.setVoltage(voltage);
    }

    @Override
    public void stopMotors() {
        m_indexerMotor.stopMotor();
    }

    @Override
    public void zeroEncoders() {
        m_indexerMotor.setPosition(0);
        m_BottomFeederMotor.setPosition(0);
    }

    @Override
    public AngularVelocity getVelocity() {
        return indexerDeviceVelocity.getValue();
    }

    @Override
    public void updateInputs(SerializerInputs inputs) {
        inputs.isIndexerMotorConnected = BaseStatusSignal.refreshAll(
            indexerDevicePosition,
            indexerDeviceVelocity,
            indexerDeviceVoltage,
            indexerDeviceCurrent,
            indexerDeviceTemp
        ).isOK();
        inputs.indexerMotorPosition = indexerDevicePosition.getValue();
        inputs.indexerMotorVelocity = indexerDeviceVelocity.getValue();
        inputs.indexerAppliedVoltage = indexerDeviceVoltage.getValue();
        inputs.indexerAppliedCurrent = indexerDeviceCurrent.getValue();
        inputs.indexerMotorTemperature = indexerDeviceTemp.getValueAsDouble();

        inputs.isBottomFeederMotorConnected = BaseStatusSignal.refreshAll(
            bottomFeederDevicePosition,
            bottomFeederDeviceVelocity,
            bottomFeederDeviceVoltage,
            bottomFeederDeviceCurrent,
            bottomFeederDeviceTemp
        ).isOK();
        inputs.bottomFeederMotorPosition = bottomFeederDevicePosition.getValue();
        inputs.bottomFeederMotorVelocity = bottomFeederDeviceVelocity.getValue();
        inputs.bottomFeederAppliedVoltage = bottomFeederDeviceVoltage.getValue();
        inputs.bottomFeederAppliedCurrent = bottomFeederDeviceCurrent.getValue();
        inputs.bottomFeederMotorTemperature = bottomFeederDeviceTemp.getValueAsDouble();
    }

}