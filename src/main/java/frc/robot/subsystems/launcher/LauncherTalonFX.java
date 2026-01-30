package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

public class LauncherTalonFX implements LauncherIO {

    private final TalonFX  launcherMotor;
    private final TalonFX  launcherFollowerMotor;
    private final CANcoder launcherSensor;
    // private final CANcoder launcherFollowerSensor;

    // Launcher
    private final StatusSignal<Temperature>     temperatureLauncher;
    private final StatusSignal<Voltage>         voltageLauncher;
    private final StatusSignal<Current>         currentLauncher;
    private final StatusSignal<AngularVelocity> speedLauncher;

    private final StatusSignal<Temperature>     temperatureLauncherFollower;
    private final StatusSignal<Voltage>         voltageLauncherFollower;
    private final StatusSignal<Current>         currentLauncherFollower;
    private final StatusSignal<AngularVelocity> speedLauncherFollower;
    
    // Hood

    public LauncherTalonFX(int launcherCanID,int LauncherSensorCanID ,int launcherFollowerCanID) {
        launcherMotor = new TalonFX(launcherCanID);
        launcherFollowerMotor = new TalonFX(launcherFollowerCanID);
        launcherSensor = new CANcoder(LauncherSensorCanID);
        // Launcher
        temperatureLauncher = launcherFollowerMotor.getDeviceTemp();
        voltageLauncherFollower = launcherFollowerMotor.getMotorVoltage();
        currentLauncherFollower = launcherFollowerMotor.getSupplyCurrent();
        speedLauncherFollower = launcherFollowerMotor.getVelocity();

        temperatureLauncherFollower = launcherMotor.getDeviceTemp();
        voltageLauncher = launcherMotor.getMotorVoltage();
        currentLauncher = launcherMotor.getSupplyCurrent();
        speedLauncher = launcherMotor.getVelocity();
        
        // Hood
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                temperatureLauncher,
                voltageLauncher,
                currentLauncher,
                speedLauncher,
                temperatureLauncherFollower,
                voltageLauncherFollower,
                currentLauncherFollower,
                speedLauncherFollower
        );

        TalonFXConfigurator launcherConfigurator = launcherMotor.getConfigurator();
        TalonFXConfigurator launcherFollowerConfigurator = launcherFollowerMotor.getConfigurator();

        Slot0Configs launcherSlotConfigs = new Slot0Configs()
        .withKP(LauncherConstants.kP)
        .withKI(LauncherConstants.kI)
        .withKD(LauncherConstants.kD)
        .withKV(LauncherConstants.kV)
        .withKS(LauncherConstants.kS)
        .withKG(LauncherConstants.kG);
        
        launcherConfigurator.apply(launcherSlotConfigs);
        launcherFollowerConfigurator.apply(launcherSlotConfigs);

        CurrentLimitsConfigs launcherCurrentLimitsConfigs = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(30)
        .withSupplyCurrentLimitEnable(true);

        launcherConfigurator.apply(launcherCurrentLimitsConfigs);
        launcherFollowerConfigurator.apply(launcherCurrentLimitsConfigs);

        MotorOutputConfigs launcherOutputConfigs = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast);
        MotorOutputConfigs launcherFollowerOutputConfigs = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast);

        launcherConfigurator.apply(launcherOutputConfigs);
        launcherFollowerConfigurator.apply(launcherFollowerOutputConfigs);

        FeedbackConfigs launcherFeedbackConfigs = new FeedbackConfigs()
        // .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);
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
    public void runVelocity(double velocity) {
        launcherMotor.setControl(new VelocityVoltage(velocity)
                                         .withSlot(0)
                                         .withFeedForward(0.5)
        );
    }

    @Override
    public void launchFuel(AngularVelocity robotVelocity, double flyWheelRadius, double distance) {
        launcherMotor.set(0.1);
        // launcherMotor.setControl(velocityVoltage.withVelocity(10));
        SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(LauncherConstants.kS, LauncherConstants.kV, LauncherConstants.kA);

        double tagrgetVelocityRadPerSec = Math.sqrt((distance * 9.81) / Math.sin(2 * 45));

        double voltage = feedForward.calculate(tagrgetVelocityRadPerSec);

        launcherMotor.setVoltage(voltage);
        
    }

    // @Override
    // public void setHoodPos(Angle pos) {
    //     hoodMotor.setPosition(pos);
    // }

    // @Override
    // public Angle getHoodPos() {
    //     return hoodMotor.getPosition().getValue();
    // }

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

        // inputs.isHoodConnected = BaseStatusSignal.refreshAll(
        //         voltageHood,
        //         currentHood,
        //         temperatureHood,
        //         speedHood
        // ).isOK();

        // Launcher
        inputs.temperatureLauncher = temperatureLauncher.getValueAsDouble();
        inputs.launcherVoltage = voltageLauncher.getValue();
        inputs.launcherCurrent = currentLauncher.getValue();
        inputs.launcherSpeedRadians = speedLauncher.getValue();
        inputs.launcherRPM = Units.radiansPerSecondToRotationsPerMinute(speedLauncher.getValueAsDouble());

        inputs.temperatureFollowerLauncher = temperatureLauncherFollower.getValueAsDouble();
        inputs.launcherFollowerVoltage = voltageLauncherFollower.getValue();
        inputs.launcherFollowerCurrent = currentLauncherFollower.getValue();
        inputs.launcherFollowerSpeedRadians = speedLauncherFollower.getValue();
        inputs.launcherFollowerRPM = Units.radiansPerSecondToRotationsPerMinute(speedLauncherFollower.getValueAsDouble());
        
        // Hood
        // inputs.temperatureHood = temperatureHood.getValueAsDouble();
        // inputs.hoodVoltage = voltageHood.getValue();
        // inputs.hoodCurrent = currentHood.getValue();
        // inputs.hoodSpeedRadians = speedHood.getValue();
        // inputs.launcherRPM = Units.radiansPerSecondToRotationsPerMinute(speedLauncher.getValueAsDouble());
        // inputs.hoodPosition = hoodPosition.getValue();
    }
}
