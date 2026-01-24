package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.launcher.LauncherConstants.kLauncher;

public class LauncherTalonFX implements LauncherIO {
    private final TalonFX  hoodMotor;
    private final CANcoder hoodSensor;

    private final TalonFX  launcherMotor;
    private final CANcoder launcherSensor;

    private final StatusSignal<Temperature>     temperatureLauncher;
    private final StatusSignal<Temperature>     temperatureHood;
    private final StatusSignal<Voltage>         voltageLauncher;
    private final StatusSignal<Voltage>         voltageHood;
    private final StatusSignal<Current>         currentLauncher;
    private final StatusSignal<Current>         currentHood;
    private final StatusSignal<AngularVelocity> speedLauncher;
    private final StatusSignal<AngularVelocity> speedHood;
    private final StatusSignal<Angle>           hoodPosition;

    public LauncherTalonFX(int launcherCanID, int launcherSensorID, int hoodCanID, int hoodSensorID) {
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

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
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

        TalonFXConfigurator launcherConfigurator = launcherMotor.getConfigurator();
        TalonFXConfigurator hoodConfigurator = hoodMotor.getConfigurator();

        Slot0Configs launcherSlotConfigs = new Slot0Configs();
        launcherSlotConfigs.kG = kLauncher.kG;
        launcherSlotConfigs.kS = kLauncher.kS;
        launcherSlotConfigs.kV = kLauncher.kV;
        launcherSlotConfigs.kP = kLauncher.kP;
        launcherSlotConfigs.kI = kLauncher.kI;
        launcherSlotConfigs.kD = kLauncher.kD;
        launcherConfigurator.apply(launcherSlotConfigs);

        CurrentLimitsConfigs launcherCurrentLimitsConfigs = new CurrentLimitsConfigs();
        launcherCurrentLimitsConfigs.SupplyCurrentLimit = 30;
        launcherCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
        launcherConfigurator.apply(launcherCurrentLimitsConfigs);

        MotorOutputConfigs launcherOutputConfigs = new MotorOutputConfigs();
        launcherConfigurator.apply(launcherOutputConfigs);

        FeedbackConfigs launcherFeedbackConfigs = new FeedbackConfigs();
        launcherFeedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);
        launcherFeedbackConfigs.withRemoteCANcoder(launcherSensor);
        launcherConfigurator.apply(launcherFeedbackConfigs);
    }

    @Override
    public void setVoltage(double volts) {
        launcherMotor.setVoltage(volts);
    }

    @Override
    public void runVelocity(double velocity) {
        launcherMotor.setControl(new VelocityVoltage(velocity)
                                         .withSlot(0)
                                         .withFeedForward(0.5)
        );
    }

    @Override
    public void launchFuel() {
        launcherMotor.set(0.1);
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
        inputs.isLauncherConnected = BaseStatusSignal.refreshAll(
                voltageLauncher,
                currentLauncher,
                temperatureLauncher,
                speedLauncher
        ).isOK();

        inputs.isHoodConnected = BaseStatusSignal.refreshAll(
                voltageHood,
                currentHood,
                temperatureHood,
                speedHood
        ).isOK();

        inputs.temperatureLauncher = temperatureLauncher.getValueAsDouble();
        inputs.temperatureHood = temperatureHood.getValueAsDouble();
        inputs.launcherVoltage = voltageLauncher.getValue();
        inputs.hoodVoltage = voltageHood.getValue();
        inputs.launcherCurrent = currentLauncher.getValue();
        inputs.hoodCurrent = currentHood.getValue();
        inputs.launcherSpeed = speedLauncher.getValue();
        inputs.hoodSpeed = speedHood.getValue();
        inputs.hoodPosition = hoodPosition.getValue();
    }
}
