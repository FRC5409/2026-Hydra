package frc.robot.subsystems.serializer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class SerializerIOTalonFX implements SerializerIO {
    
    private TalonFX indexerMotor;

    private TalonFXConfigurator indexerMotorConfig;
    private CurrentLimitsConfigs currentConfigs;

    private StatusSignal<AngularVelocity> indexerDeviceVelocity;
    private StatusSignal<Angle> indexerDevicePosition;
    private StatusSignal<Voltage> indexerDeviceVoltage;
    private StatusSignal<Current> indexerDeviceCurrent;
    private StatusSignal<Temperature> indexerDeviceTemp;

    


    public SerializerIOTalonFX(int indexerID) {
        indexerMotor = new TalonFX(indexerID);

        indexerMotorConfig = indexerMotor.getConfigurator();

        currentConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(SerializerConstants.TALON_FX_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);
        indexerMotorConfig.apply(currentConfigs);

        indexerMotorConfig.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        indexerMotor.setNeutralMode(NeutralModeValue.Brake);

        indexerDeviceVelocity = indexerMotor.getVelocity();
        indexerDevicePosition = indexerMotor.getPosition();
        indexerDeviceVoltage = indexerMotor.getMotorVoltage();
        indexerDeviceCurrent = indexerMotor.getSupplyCurrent();
        indexerDeviceTemp = indexerMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            indexerDevicePosition,
            indexerDeviceVelocity,
            indexerDeviceVoltage,
            indexerDeviceCurrent,
            indexerDeviceTemp
        );

        indexerMotor.optimizeBusUtilization();
    }

    @Override
    public void setIndexerMotorVoltage(double voltage) {
        indexerMotor.setVoltage(voltage);
    }

    @Override
    public void stopIndexerMotor() {
        indexerMotor.stopMotor();
    }

    @Override
    public void zeroIndexerEncoder() {
        indexerMotor.setPosition(0);
    }

    @Override
    public AngularVelocity getIndexerVelocity() {
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
    }

}