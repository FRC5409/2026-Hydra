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
    
    private TalonFX floorMotor;
    private TalonFX feederMotor;

    private TalonFXConfigurator floorMotorConfig;
    private TalonFXConfigurator feederMotorConfig;
    private CurrentLimitsConfigs currentConfigs;

    private StatusSignal<AngularVelocity> deviceVelocity;
    private StatusSignal<Angle> devicePosition;
    private StatusSignal<Voltage> floorDeviceVoltage;
    private StatusSignal<Current> floorDeviceCurrent;
    private StatusSignal<Temperature> floorDeviceTemp;

    private StatusSignal<Voltage> feederDeviceVoltage;
    private StatusSignal<Current> feederDeviceCurrent;
    private StatusSignal<Temperature> feederDeviceTemp;


    public SerializerIOTalonFX(int floorId, int feederId) {
        floorMotor = new TalonFX(floorId);
        feederMotor = new TalonFX(feederId);

        floorMotorConfig = floorMotor.getConfigurator();
        feederMotorConfig = feederMotor.getConfigurator();
        currentConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(30)
            .withSupplyCurrentLimitEnable(true);
        floorMotorConfig.apply(currentConfigs);
        feederMotorConfig.apply(currentConfigs);

        floorMotorConfig.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        floorMotor.setNeutralMode(NeutralModeValue.Brake);
        feederMotor.setNeutralMode(NeutralModeValue.Brake);

        feederMotor.setControl(new Follower(floorId, MotorAlignmentValue.Opposed));

        deviceVelocity = floorMotor.getVelocity();
        devicePosition = floorMotor.getPosition();
        floorDeviceVoltage = floorMotor.getMotorVoltage();
        floorDeviceCurrent = floorMotor.getSupplyCurrent();
        floorDeviceTemp = floorMotor.getDeviceTemp();

        feederDeviceVoltage = feederMotor.getMotorVoltage();
        feederDeviceCurrent = feederMotor.getSupplyCurrent();
        feederDeviceTemp = feederMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            devicePosition,
            deviceVelocity,
            floorDeviceVoltage,
            floorDeviceCurrent,
            floorDeviceTemp,
            feederDeviceVoltage,
            feederDeviceCurrent,
            feederDeviceTemp
        );

        floorMotor.optimizeBusUtilization();
        feederMotor.optimizeBusUtilization();
    }


    @Override
    public void setMotorVoltage(double voltage) {
        floorMotor.setVoltage(voltage);
    }

    @Override
    public void setIndexerMotorVoltage(double voltage) {
        floorMotor.setVoltage(voltage);
    }

    @Override
    public void setFeederMotorVoltage(double voltage) {
        feederMotor.setVoltage(voltage);
    }

    @Override
    public void stopMotor() {
        floorMotor.stopMotor();
    }

    @Override
    public void updateInputs(SerializerInputs inputs) {
        inputs.isFloorMotorConnected = BaseStatusSignal.refreshAll(
            devicePosition,
            deviceVelocity,
            floorDeviceVoltage,
            floorDeviceCurrent,
            floorDeviceTemp
        ).isOK();
        
        inputs.isFeederMotorConnected = BaseStatusSignal.refreshAll(
            feederDeviceVoltage,
            feederDeviceCurrent,
            feederDeviceTemp
        ).isOK();   
    }

}