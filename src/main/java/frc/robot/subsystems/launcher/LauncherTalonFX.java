package frc.robot.subsystems.launcher;

import java.lang.System.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

public class LauncherTalonFX implements LauncherIO {
    private final TalonFX  hoodMotor;
    private final CANcoder hoodSensor;

    private final TalonFX  launcherMotor;
    private final CANcoder launcherSensor;

    private final TalonFX launcherFollowerMotor;

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

    private double velocitySetpoint;

    public LauncherTalonFX(int launcherCanID, int launcherSensorID, int launcherFollowerCanID, int hoodCanID, int hoodSensorID) {
        hoodMotor = new TalonFX(hoodCanID);
        hoodSensor = new CANcoder(hoodSensorID);

        launcherMotor = new TalonFX(launcherCanID);
        launcherSensor = new CANcoder(launcherSensorID);

        launcherFollowerMotor = new TalonFX(launcherFollowerCanID);

        // Launcher
        temperatureLauncher = launcherMotor.getDeviceTemp();
        voltageLauncher = launcherMotor.getMotorVoltage();
        currentLauncher = launcherMotor.getSupplyCurrent();
        speedLauncher = launcherMotor.getVelocity();

        temperatureLauncherFollower = launcherFollowerMotor.getDeviceTemp();
        voltageLauncherFollower = launcherFollowerMotor.getMotorVoltage();
        currentLauncherFollower = launcherFollowerMotor.getSupplyCurrent();
        speedLauncherFollower = launcherFollowerMotor.getVelocity();
        
        // Hood
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

        TalonFXConfigurator launcherConfigurator = launcherMotor.getConfigurator();
        TalonFXConfigurator launcherFollowerConfigurator = launcherMotor.getConfigurator();

        TalonFXConfigurator hoodConfigurator = hoodMotor.getConfigurator();

        Slot0Configs launcherSlotConfigs = new Slot0Configs()
        .withKP(LauncherConstants.PID.getP())
        .withKI(LauncherConstants.PID.getI())
        .withKD(LauncherConstants.PID.getD())
        .withKV(LauncherConstants.kV)
        .withKS(LauncherConstants.kS)
        .withKG(LauncherConstants.kG);
        
        launcherConfigurator.apply(launcherSlotConfigs);
        launcherFollowerConfigurator.apply(launcherSlotConfigs);

        CurrentLimitsConfigs launcherCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(LauncherConstants.SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);

        launcherConfigurator.apply(launcherCurrentLimitsConfigs);
        launcherFollowerConfigurator.apply(launcherCurrentLimitsConfigs);

        MotorOutputConfigs launcherOutputConfigs = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.CounterClockwise_Positive);
        MotorOutputConfigs launcherFollowerOutputConfigs = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast);

        launcherConfigurator.apply(launcherOutputConfigs);
        launcherFollowerConfigurator.apply(launcherFollowerOutputConfigs);

        FeedbackConfigs launcherFeedbackConfigs = new FeedbackConfigs()
            .withRemoteCANcoder(launcherSensor);

        launcherConfigurator.apply(launcherFeedbackConfigs);
        launcherFollowerConfigurator.apply(launcherFeedbackConfigs);

        launcherFollowerMotor.setControl(new Follower(launcherCanID, MotorAlignmentValue.Opposed));
    }

    @Override
    public void setVoltage(double volts) {
        launcherMotor.setVoltage(volts);
    }

    @Override
    public void runRPS(double velocity) {
        this.velocitySetpoint = velocity;
        VelocityVoltage velocityVoltage = new VelocityVoltage(velocity)
                                         .withSlot(0)
                                         .withFeedForward(0);
                            
        launcherMotor.setControl(velocityVoltage
        );

        // Logger.recordOutput("Launcher/velocityVoltage", velocityVoltage.Velocity);
    }

    @Override
    public void launchFuel(Distance distance) {
        // SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(LauncherConstants.kS, LauncherConstants.kV, LauncherConstants.kA);

        // double tagrgetVelocityRadPerSec = Math.sqrt((distance * 9.81) / Math.sin(2 * 45));

        // double voltage = feedForward.calculate(tagrgetVelocityRadPerSec);

        // launcherMotor.setVoltage(voltage);
        
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
                speedLauncher,

                voltageLauncherFollower,
                currentLauncherFollower,
                temperatureLauncherFollower,
                speedLauncherFollower
        ).isOK();

        inputs.isHoodConnected = BaseStatusSignal.refreshAll(
                voltageHood,
                currentHood,
                temperatureHood,
                speedHood
        ).isOK();

        inputs.temperatureLauncher = temperatureLauncher.getValueAsDouble();
        inputs.launcherVoltage = voltageLauncher.getValue();
        inputs.launcherCurrent = currentLauncher.getValue();
        inputs.launcherSpeedRadians = speedLauncher.getValue();

        inputs.launcherFollowerTemperature = temperatureLauncher.getValueAsDouble();
        inputs.launcherFollowerVoltage = voltageLauncher.getValue();
        inputs.launcherFollowerCurrent = currentLauncher.getValue();
        inputs.launcherFollowerSpeedRadians = speedLauncher.getValue();

        inputs.temperatureHood = temperatureHood.getValueAsDouble();
        inputs.hoodVoltage = voltageHood.getValue();
        inputs.hoodCurrent = currentHood.getValue();
        inputs.hoodSpeedRadians = speedHood.getValue();
        inputs.hoodPosition = hoodPosition.getValue();
    }
}
