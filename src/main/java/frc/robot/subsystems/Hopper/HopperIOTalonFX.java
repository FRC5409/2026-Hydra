package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class HopperIOTalonFX implements HopperIO {
    private final TalonFX m_mainMotor;
    private final TalonFX m_followerMotor;

    private final TalonFXConfigurator m_mainMotorConfig;
    private final TalonFXConfigurator m_followerMotorConfig;

    private CurrentLimitsConfigs m_currentConfig;
    private FeedbackConfigs m_encoderConfigs;
    private Slot0Configs m_pidConfig;

    private PositionVoltage m_request;
    
    private StatusSignal<Angle> motorPosition;
    private StatusSignal<Voltage> mainDeviceVoltage;
    private StatusSignal<Current> mainDeviceCurrent;
    private StatusSignal<Temperature> mainDeviceTemp;
    private StatusSignal<Voltage> followerDeviceVoltage;
    private StatusSignal<Current> followerDeviceCurrent;
    private StatusSignal<Temperature> followerDeviceTemp;

    public HopperIOTalonFX(int mainMotorID, int followerMotorID) {
        m_mainMotor = new TalonFX(mainMotorID);
        m_followerMotor = new TalonFX(followerMotorID);

        m_mainMotorConfig = m_mainMotor.getConfigurator();
        m_followerMotorConfig = m_followerMotor.getConfigurator();

        m_currentConfig = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(30) //Test value, needs to be tuned (current limit)
            .withSupplyCurrentLimitEnable(true);
        m_mainMotorConfig.apply(m_currentConfig);
        m_followerMotorConfig.apply(m_currentConfig);

        m_encoderConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(1.0/10.0); //Test (random placeholder) value, needs to be tuned
        m_mainMotorConfig.apply(m_encoderConfigs);
        m_followerMotorConfig.apply(m_encoderConfigs);

        m_pidConfig = new Slot0Configs()
            .withKP(0.001) //test value
            .withKI(0.0)
            .withKD(0.0);

        m_mainMotorConfig.apply(m_pidConfig);
        m_followerMotorConfig.apply(m_pidConfig);

        m_mainMotorConfig.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        m_mainMotor.setNeutralMode(NeutralModeValue.Brake);
        m_followerMotor.setNeutralMode(NeutralModeValue.Brake);

        m_followerMotor.setControl(new Follower(mainMotorID, MotorAlignmentValue.Opposed));

        m_request = new PositionVoltage(0).withSlot(0);

        m_mainMotor.setPosition(0);
 
        motorPosition = m_mainMotor.getPosition();
        mainDeviceVoltage = m_mainMotor.getMotorVoltage();
        mainDeviceCurrent = m_mainMotor.getSupplyCurrent();
        mainDeviceTemp  = m_mainMotor.getDeviceTemp();
        followerDeviceVoltage = m_followerMotor.getMotorVoltage();
        followerDeviceCurrent = m_followerMotor.getSupplyCurrent();
        followerDeviceTemp = m_followerMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            motorPosition,
            mainDeviceVoltage,
            mainDeviceCurrent,
            mainDeviceTemp,
            followerDeviceVoltage,
            followerDeviceCurrent, 
            followerDeviceTemp
        );

        m_mainMotor.optimizeBusUtilization();
        m_followerMotor.optimizeBusUtilization();

    }


    @Override
    public void setMotorVoltage(double voltage) {
        m_mainMotor.setVoltage(voltage);
    }

    @Override
    public void stopMotor() {
        m_mainMotor.stopMotor();
    }

    @Override
    public void zeroEncoder() {
        m_mainMotor.setPosition(0);
    }

    @Override
    public Distance getPosition() {
        return Meters.of(m_mainMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void setSetpoint(Distance setpoint) {
        m_mainMotor.setControl(m_request.withPosition(setpoint.in(Meters)));
    }

    @Override
    public void updateInputs(HopperInputs inputs) {

        inputs.mainMotorConnection = BaseStatusSignal.refreshAll(
            motorPosition,
            mainDeviceVoltage, 
            mainDeviceCurrent, 
            mainDeviceTemp
        ).isOK();
        inputs.mainMotorVoltage = mainDeviceVoltage.getValueAsDouble();
        inputs.mainMotorCurrent = Math.abs(mainDeviceCurrent.getValueAsDouble());
        inputs.mainMotorTemp = mainDeviceTemp.getValueAsDouble();
        inputs.mainMotorPosition = motorPosition.getValueAsDouble();
        
        inputs.followerMotorConnection = BaseStatusSignal.refreshAll(
            followerDeviceVoltage, 
            followerDeviceCurrent, 
            followerDeviceTemp
        ).isOK();
        inputs.followerMotorVoltage = followerDeviceVoltage.getValueAsDouble();
        inputs.followerMotorCurrent = Math.abs(followerDeviceCurrent.getValueAsDouble());
        inputs.followerMotorTemp = followerDeviceTemp.getValueAsDouble();        
        inputs.followerMotorPosition = motorPosition.getValueAsDouble();
    }

}