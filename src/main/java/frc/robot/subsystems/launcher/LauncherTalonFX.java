package frc.robot.subsystems.launcher;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Ultrasonic;

public class LauncherTalonFX implements LauncherIO {
    // Motors and sensors
    private final TalonFX  hoodMotor;
    private final CANcoder hoodSensor;

    private final TalonFX  launcherMotor;
    private final CANcoder launcherSensor;

    private final TalonFX launcherFollowerMotor;

    private final Ultrasonic ultrasonic;
    private final MedianFilter medianFilter;

    // IOs
    private final StatusSignal<Temperature>     temperatureLauncher;
    private final StatusSignal<Voltage>         voltageLauncher;
    private final StatusSignal<Current>         currentLauncher;
    private final StatusSignal<AngularVelocity> speedLauncher;

    private final StatusSignal<Temperature> temperatureLauncherFollower;
    private final StatusSignal<Voltage> voltageLauncherFollower;
    private final StatusSignal<Current> currentLauncherFollower;
    private final StatusSignal<AngularVelocity> speedLauncherFollower;

    private final StatusSignal<Temperature>     temperatureHood;
    private final StatusSignal<Voltage>         voltageHood;
    private final StatusSignal<Current>         currentHood;
    private final StatusSignal<AngularVelocity> speedHood;
    private final StatusSignal<Angle>           hoodPosition;

    // This thing
    private double velocitySetpoint;

    public LauncherTalonFX(int launcherCanID, int launcherSensorID, int launcherFollowerCanID, int hoodCanID, int hoodSensorID, DigitalOutput pingChannel, DigitalInput echoChannel) {
        // Motors and sensors
        hoodMotor = new TalonFX(hoodCanID);
        hoodSensor = new CANcoder(hoodSensorID);

        launcherMotor = new TalonFX(launcherCanID);
        launcherSensor = new CANcoder(launcherSensorID);

        launcherFollowerMotor = new TalonFX(launcherFollowerCanID);

        ultrasonic = new Ultrasonic(pingChannel, echoChannel);
        medianFilter = new MedianFilter(0);

        // IOs
        temperatureLauncher = launcherMotor.getDeviceTemp();
        voltageLauncher = launcherMotor.getMotorVoltage();
        currentLauncher = launcherMotor.getSupplyCurrent();
        speedLauncher = launcherMotor.getVelocity();

        temperatureLauncherFollower = launcherFollowerMotor.getDeviceTemp();
        voltageLauncherFollower = launcherFollowerMotor.getMotorVoltage();
        currentLauncherFollower = launcherFollowerMotor.getSupplyCurrent();
        speedLauncherFollower = launcherFollowerMotor.getVelocity();
        
        temperatureHood = hoodMotor.getDeviceTemp();
        voltageHood = hoodMotor.getMotorVoltage();
        currentHood = hoodMotor.getSupplyCurrent();
        speedHood = hoodMotor.getVelocity();
        hoodPosition = hoodMotor.getPosition();
        
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                temperatureLauncher,
                voltageLauncher,
                currentLauncher,
                speedLauncher,

                temperatureLauncherFollower,
                voltageLauncherFollower,
                currentLauncherFollower,
                speedLauncherFollower,

                temperatureHood,
                voltageHood,
                currentHood,
                speedHood,
                hoodPosition
        );

        // Configurators
        TalonFXConfigurator launcherConfigurator = launcherMotor.getConfigurator();
        TalonFXConfigurator launcherFollowerConfigurator = launcherMotor.getConfigurator();

        TalonFXConfigurator hoodConfigurator = hoodMotor.getConfigurator();

        // Slot configs
        Slot0Configs launcherSlotConfigs = new Slot0Configs()
            .withKP(LauncherConstants.Launcher.PID.getP())
            .withKI(LauncherConstants.Launcher.PID.getI())
            .withKD(LauncherConstants.Launcher.PID.getD())
            .withKV(LauncherConstants.Launcher.kV)
            .withKS(LauncherConstants.Launcher.kS)
            .withKG(LauncherConstants.Launcher.kG);
        
        launcherConfigurator.apply(launcherSlotConfigs);
        launcherFollowerConfigurator.apply(launcherSlotConfigs);

        Slot0Configs hoodSlotConfigs = new Slot0Configs()
            .withKP(LauncherConstants.Hood.PID.getP())
            .withKI(LauncherConstants.Hood.PID.getI())
            .withKD(LauncherConstants.Hood.PID.getD());

        hoodConfigurator.apply(hoodSlotConfigs);

        // Current limit configs
        CurrentLimitsConfigs launcherCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(LauncherConstants.SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);

        launcherConfigurator.apply(launcherCurrentLimitsConfigs);
        launcherFollowerConfigurator.apply(launcherCurrentLimitsConfigs);

        CurrentLimitsConfigs hoodCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(LauncherConstants.SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);

        hoodConfigurator.apply(hoodCurrentLimitConfigs);

        // Motor output configs
        MotorOutputConfigs launcherOutputConfigs = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.CounterClockwise_Positive);
        MotorOutputConfigs launcherFollowerOutputConfigs = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast);

        launcherConfigurator.apply(launcherOutputConfigs);
        launcherFollowerConfigurator.apply(launcherFollowerOutputConfigs);

        MotorOutputConfigs hoodOutputConfigs = new MotorOutputConfigs();

        hoodConfigurator.apply(hoodOutputConfigs);

        // Feedback configs
        FeedbackConfigs launcherFeedbackConfigs = new FeedbackConfigs()
            .withRemoteCANcoder(launcherSensor);

        launcherConfigurator.apply(launcherFeedbackConfigs);
        launcherFollowerConfigurator.apply(launcherFeedbackConfigs);

        FeedbackConfigs hoodFeedbackConfigs = new FeedbackConfigs()
            .withRemoteCANcoder(hoodSensor);

        hoodConfigurator.apply(hoodFeedbackConfigs);

        launcherFollowerMotor.setControl(new Follower(launcherCanID, MotorAlignmentValue.Opposed));
    }

    // Voltage
    @Override
    public void launcherSetVoltage(double volts) {
        launcherMotor.setVoltage(volts);
    }

    @Override
    public void hoodSetVoltage(double volts) {
        hoodMotor.setVoltage(volts);
    }

    // Run systems
    @Override
    public void runRPS(Supplier<AngularVelocity> velocity) {
        launcherMotor.setControl(new VelocityVoltage(velocity.get())
                                         .withSlot(0)
                                         .withFeedForward(0));
    }

    @Override
    public void setHoodPos(Angle pos) {
        if (pos.gte(LauncherConstants.Hood.MIN_ANGLE) && pos.lte(LauncherConstants.Hood.MAX_ANGLE)) {
            hoodMotor.setPosition(pos);
        }
    }

    // Gettes
    @Override
    public Angle getHoodPos() {
        return hoodMotor.getPosition().getValue();
    }

    @Override
    public double getDistance() {
        return medianFilter.calculate(ultrasonic.getRangeInches());
    }

    // Stops
    @Override
    public void stopLauncher() {
        launcherMotor.setVoltage(0);
    }

    @Override
    public void stopHood() {
        hoodMotor.setVoltage(0);
    }

    @Override
    public void updateInputs(LauncherInputs inputs) {
        inputs.isLauncherConnected = BaseStatusSignal.refreshAll(
                voltageLauncher,
                currentLauncher,
                temperatureLauncher,
                speedLauncher,

                voltageLauncherFollower,
                currentLauncherFollower,
                temperatureLauncherFollower,
                speedLauncherFollower
        ).isOK();

        // inputs.isHoodConnected = BaseStatusSignal.refreshAll(
        //         voltageHood,
        //         currentHood,
        //         temperatureHood,
        //         speedHood
        // ).isOK();
        inputs.isHoodConnected = false;

        // Launcher
        inputs.temperatureLauncher = temperatureLauncher.getValueAsDouble();
        inputs.launcherVoltage = voltageLauncher.getValue();
        inputs.launcherCurrent = currentLauncher.getValue();
        inputs.launcherSpeedRadians = speedLauncher.getValue();

        inputs.launcherFollowerTemperature = temperatureLauncher.getValueAsDouble();
        inputs.launcherFollowerVoltage = voltageLauncher.getValue();
        inputs.launcherFollowerCurrent = currentLauncher.getValue();
        inputs.launcherFollowerSpeedRadians = speedLauncher.getValue();

        // Hood
        inputs.temperatureHood = temperatureHood.getValueAsDouble();
        inputs.hoodVoltage = voltageHood.getValue();
        inputs.hoodCurrent = currentHood.getValue();
        inputs.hoodSpeedRadians = speedHood.getValue();
        inputs.hoodPosition = hoodPosition.getValue();
    }
}
