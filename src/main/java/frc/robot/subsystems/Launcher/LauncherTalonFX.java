package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class LauncherTalonFX implements LauncherIO {
    private int launcherCanID;
    private int launcherSensorID;

    private int hoodCanID;
    private int hoodSensorID;

    private TalonFX hoodMotor;
    private CANcoder hoodSensor;

    private TalonFX launcherMotor;
    private CANcoder launcherSensor;

    private final VelocityVoltage velocityVoltage;

    private StatusSignal<Temperature> temperatureLauncher;
    private StatusSignal<Temperature> temperatureHood;
    private StatusSignal<Voltage> voltageLauncher;
    private StatusSignal<Voltage> voltageHood;
    private StatusSignal<Current> currentLauncher;
    private StatusSignal<Current> currentHood;
    private StatusSignal<AngularVelocity> speedLauncher;
    private StatusSignal<AngularVelocity> speedHood;
    private StatusSignal<Angle> hoodPosition;

    public LauncherTalonFX (int launcherCanID, int launcherSensorID,
                            int hoodCanID, int hoodSensorID) {

        this.launcherCanID = launcherCanID;
        this.launcherSensorID = launcherSensorID;

        this.hoodCanID = hoodCanID;
        this.hoodSensorID = hoodSensorID;

        hoodMotor = new TalonFX(hoodCanID);
        hoodSensor = new CANcoder(hoodSensorID);

        launcherMotor = new TalonFX(launcherCanID);
        launcherSensor = new CANcoder(launcherSensorID);

        temperatureLauncher = launcherMotor.getDeviceTemp();
        temperatureHood = hoodMotor.getDeviceTemp();
        voltageLauncher = launcherMotor.getMotorVoltage();
        voltageHood = hoodMotor.getMotorVoltage();
        currentLauncher = launcherMotor.getSupplyCurrent();
        currentHood = hoodMotor.getSupplyCurrent();
        speedLauncher = launcherMotor.getVelocity();
        speedHood = hoodMotor.getVelocity();
        hoodPosition = hoodMotor.getPosition();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
            temperatureLauncher,
            temperatureHood,
            voltageLauncher,
            voltageHood,
            currentLauncher,
            currentHood,
            speedLauncher,
            speedHood,
            hoodPosition
        );

        velocityVoltage = new VelocityVoltage(0).withSlot(0);

        TalonFXConfigurator launcherConfigurator = launcherMotor.getConfigurator();
        TalonFXConfigurator hoodConfigurator = hoodMotor.getConfigurator();

        Slot0Configs launcherSlotConfigs = new Slot0Configs();
        launcherSlotConfigs.kG = 0.0;
        launcherSlotConfigs.kS = 0.1;
        launcherSlotConfigs.kV = 0.12;
        launcherSlotConfigs.kP = 0.11;
        launcherSlotConfigs.kI = 0.0;
        launcherSlotConfigs.kD = 0.0;

        launcherConfigurator.apply(launcherSlotConfigs);

        final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

        launcherMotor.setControl(velocityVoltage.withVelocity(8).withFeedForward(0.5));
 
        CurrentLimitsConfigs launcherCurrentLimitsConfigs = new CurrentLimitsConfigs(); // check
        launcherCurrentLimitsConfigs.SupplyCurrentLimit = 30;
        launcherCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;

        // MotorOutputConfigs
        MotorOutputConfigs launcheOutputConfigs = new MotorOutputConfigs(); // check
        launcherConfigurator.apply(launcheOutputConfigs);

        // FeedBackConfigs
        FeedbackConfigs launcherFeedbackConfigs = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder).withRemoteCANcoder(launcherSensor); // check
        launcherConfigurator.apply(launcherFeedbackConfigs);
    }
    
    @Override
    public void setVoltage(double volts) {
        launcherMotor.setVoltage(volts);
    }

    @Override
    public void launchFuel() {
        launcherMotor.set(0.5);
        // launcherMotor.setControl(velocityVoltage.withVelocity(10));
    }

    @Override
    public void setHoodPos(Angle pos) {
        hoodMotor.setPosition(pos);
    }

    @Override
    public Angle getHoodPos() {
        return hoodMotor.getPosition().getValue();
    }

    @Override
    public void stop() {
        launcherMotor.setVoltage(0);
    }

    @Override
    public void updateInputs(LauncherInputs inputs) {
        inputs.launcherConnected = BaseStatusSignal.refreshAll(
            voltageLauncher,
            currentLauncher,
            temperatureLauncher,
            speedLauncher
        ).isOK();

        inputs.hoodConnected = BaseStatusSignal.refreshAll(
            voltageHood,
            currentHood,
            temperatureHood,
            speedHood
        ).isOK();

        inputs.temperatureLauncher = temperatureLauncher.getValueAsDouble();
        inputs.temperatureHood = temperatureHood.getValueAsDouble();
        inputs.volatgeLauncher = voltageLauncher.getValueAsDouble();
        inputs.voltageHood = voltageHood.getValueAsDouble();
        inputs.currentLauncher = currentLauncher.getValueAsDouble();
        inputs.currentHood = currentHood.getValueAsDouble();
        inputs.speedLauncher = speedLauncher.getValueAsDouble();
        inputs.speedHood = speedHood.getValueAsDouble();
        inputs.hoodPosition = hoodPosition.getValueAsDouble();
    }
}
