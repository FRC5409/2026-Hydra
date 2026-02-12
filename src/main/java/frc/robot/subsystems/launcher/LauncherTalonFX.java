package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Inches;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Ultrasonic;

public class LauncherTalonFX implements LauncherIO {
    // private final TalonFX  hoodMotor;
    // private final CANcoder hoodSensor;

    // private final TalonFX  launcherMotor;
    // private final CANcoder launcherSensor;

    // private final TalonFX launcherFollowerMotor;

    private final Ultrasonic ultrasonic;
    private final MedianFilter medianFilter;

    // private final Servo servo;

    // private final StatusSignal<Temperature>     temperatureLauncher;
    // private final StatusSignal<Voltage>         voltageLauncher;
    // private final StatusSignal<Current>         currentLauncher;
    // private final StatusSignal<AngularVelocity> speedLauncher;

    // private final StatusSignal<Temperature> temperatureLauncherFollower;
    // private final StatusSignal<Voltage> voltageLauncherFollower;
    // private final StatusSignal<Current> currentLauncherFollower;
    // private final StatusSignal<AngularVelocity> speedLauncherFollower;

    // private final StatusSignal<Temperature>     temperatureHood;
    // private final StatusSignal<Voltage>         voltageHood;
    // private final StatusSignal<Current>         currentHood;
    // private final StatusSignal<AngularVelocity> speedHood;
    // private final StatusSignal<Angle>           hoodPosition;

    private double velocitySetpoint;

    public LauncherTalonFX(
        //int launcherCanID,
        // int launcherSensorID,
        // int launcherFollowerCanID,
        DigitalOutput pingChannel,
        DigitalInput echoChannel
        // int channel
        // int hoodCanID,
        // int hoodSensorI0D
    ) {
        // hoodMotor = new TalonFX(hoodCanID);
        // hoodSensor = new CANcoder(hoodSensorID);

        // launcherMotor = new TalonFX(launcherCanID);
        // launcherSensor = new CANcoder(launcherSensorID);

        ultrasonic = new Ultrasonic(pingChannel, echoChannel);
        medianFilter = new MedianFilter(0);

        // servo = new Servo(channel);
        // servo.setBoundsMicroseconds(LauncherConstants.Servo.MAX_PMW_PULSE_WIDTH, LauncherConstants.Servo.DEAD_BAND_MAX, LauncherConstants.Servo.PULSE_WIDTH_CENTER, LauncherConstants.Servo.DEAD_BAND_MIN, LauncherConstants.Servo.MIN_PMW_PULSE_WIDTH);

        // launcherFollowerMotor = new TalonFX(launcherFollowerCanID);

        // Launcher
        // temperatureLauncher = launcherMotor.getDeviceTemp();
        // voltageLauncher = launcherMotor.getMotorVoltage();
        // currentLauncher = launcherMotor.getSupplyCurrent();
        // speedLauncher = launcherMotor.getVelocity();

        // temperatureLauncherFollower = launcherFollowerMotor.getDeviceTemp();
        // voltageLauncherFollower = launcherFollowerMotor.getMotorVoltage();
        // currentLauncherFollower = launcherFollowerMotor.getSupplyCurrent();
        // speedLauncherFollower = launcherFollowerMotor.getVelocity();
        
        // Hood
        // temperatureHood = hoodMotor.getDeviceTemp();
        // voltageHood = hoodMotor.getMotorVoltage();
        // currentHood = hoodMotor.getSupplyCurrent();
        // speedHood = hoodMotor.getVelocity();
        // hoodPosition = hoodMotor.getPosition();
        
        // BaseStatusSignal.setUpdateFrequencyForAll(
        //         50,
        //         temperatureLauncher,
        //         voltageLauncher,
        //         currentLauncher,
        //         speedLauncher,

        //         temperatureLauncherFollower,
        //         voltageLauncherFollower,
        //         currentLauncherFollower,
        //         speedLauncherFollower

        //         // temperatureHood,
        //         // voltageHood,
        //         // currentHood,
        //         // speedHood,
        //         // hoodPosition
        // );

        // TalonFXConfigurator launcherConfigurator = launcherMotor.getConfigurator();
        // TalonFXConfigurator launcherFollowerConfigurator = launcherMotor.getConfigurator();

        // TalonFXConfigurator hoodConfigurator = hoodMotor.getConfigurator();

        // Slot0Configs launcherSlotConfigs = new Slot0Configs()
        // .withKP(LauncherConstants.PID.getP())
        // .withKI(LauncherConstants.PID.getI())
        // .withKD(LauncherConstants.PID.getD())
        // .withKV(LauncherConstants.kV)
        // .withKS(LauncherConstants.kS)
        // .withKG(LauncherConstants.kG);
        
        // launcherConfigurator.apply(launcherSlotConfigs);
        // launcherFollowerConfigurator.apply(launcherSlotConfigs);

        // CurrentLimitsConfigs launcherCurrentLimitsConfigs = new CurrentLimitsConfigs()
        //     .withSupplyCurrentLimit(LauncherConstants.SUPPLY_CURRENT_LIMIT)
        //     .withSupplyCurrentLimitEnable(true);

        // launcherConfigurator.apply(launcherCurrentLimitsConfigs);
        // launcherFollowerConfigurator.apply(launcherCurrentLimitsConfigs);

        // MotorOutputConfigs launcherOutputConfigs = new MotorOutputConfigs()
        //     .withNeutralMode(NeutralModeValue.Coast)
        //     .withInverted(InvertedValue.CounterClockwise_Positive);
        // MotorOutputConfigs launcherFollowerOutputConfigs = new MotorOutputConfigs()
        //     .withNeutralMode(NeutralModeValue.Coast);

        // launcherConfigurator.apply(launcherOutputConfigs);
        // launcherFollowerConfigurator.apply(launcherFollowerOutputConfigs);

        // FeedbackConfigs launcherFeedbackConfigs = new FeedbackConfigs()
        //     .withRemoteCANcoder(launcherSensor);

        // launcherConfigurator.apply(launcherFeedbackConfigs);
        // launcherFollowerConfigurator.apply(launcherFeedbackConfigs);

        // launcherFollowerMotor.setControl(new Follower(launcherCanID, MotorAlignmentValue.Opposed));
    }

    // @Override
    // public void setServoPos(double pos) {
    //     if (pos <= 1 && pos >= 0) {
    //         servo.setPosition(pos);
    //     }
    // }

    // @Override
    //     public double getServoPos() {
    //         return servo.getPosition();
    //     }

    // @Override
    // public void setVoltage(double volts) {
    //     launcherMotor.setVoltage(volts);
    // }

    // @Override
    // public void runVelocity(double velocity) {
    //     this.velocitySetpoint = velocity;
    //     VelocityVoltage velocityVoltage = new VelocityVoltage(velocity)
    //                                      .withSlot(0)
    //                                      .withFeedForward(0);
                            
    //     launcherMotor.setControl(velocityVoltage
    //     );

    //     // Logger.recordOutput("Launcher/velocityVoltage", velocityVoltage.Velocity);
    // }

    // @Override
    // public void setHoodPos(Angle pos) {
    //     hoodMotor.setPosition(pos);
    // }

    // @Override
    // public Angle getHoodPos() {
    //     return hoodMotor.getPosition().getValue();
    // }

    @Override
    public double getDistance() {
        // return medianFilter.calculate(ultrasonic.getRangeInches());
        return ultrasonic.getRangeInches();
        }

    // @Override
    // public void stop() {
    //     launcherMotor.setVoltage(0);
    // }

    @Override
    public void updateInputs(LauncherInputs inputs) {
        // inputs.isLauncherConnected = BaseStatusSignal.refreshAll(
        //         voltageLauncher,
        //         currentLauncher,
        //         temperatureLauncher,
        //         speedLauncher,

        //         voltageLauncherFollower,
        //         currentLauncherFollower,
        //         temperatureLauncherFollower,
        //         speedLauncherFollower
        // ).isOK();

        // // inputs.isHoodConnected = BaseStatusSignal.refreshAll(
        // //         voltageHood,
        // //         currentHood,
        // //         temperatureHood,
        // //         speedHood
        // // ).isOK();
        // inputs.isHoodConnected = false;

        // inputs.temperatureLauncher = temperatureLauncher.getValueAsDouble();
        // inputs.launcherVoltage = voltageLauncher.getValue();
        // inputs.launcherCurrent = currentLauncher.getValue();
        // inputs.launcherSpeedRadians = speedLauncher.getValue();

        // inputs.launcherFollowerTemperature = temperatureLauncher.getValueAsDouble();
        // inputs.launcherFollowerVoltage = voltageLauncher.getValue();
        // inputs.launcherFollowerCurrent = currentLauncher.getValue();
        // inputs.launcherFollowerSpeedRadians = speedLauncher.getValue();

        inputs.ultrasonicDistance = getDistance();
        ultrasonic.setAutomaticMode(true);

        // inputs.temperatureHood = temperatureHood.getValueAsDouble();
        // inputs.hoodVoltage = voltageHood.getValue();
        // inputs.hoodCurrent = currentHood.getValue();
        // inputs.hoodSpeedRadians = speedHood.getValue();
        // inputs.hoodPosition = hoodPosition.getValue();
        //ii
    }
}
