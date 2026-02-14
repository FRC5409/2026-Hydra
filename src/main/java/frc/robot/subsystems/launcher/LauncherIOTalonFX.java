package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Servo;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeters;

import java.util.function.Supplier;

public class LauncherIOTalonFX implements LauncherIO {
    // Motors and sensors
    // private final TalonFX hoodMotor;
    private final Servo   hoodServo;
    private final Servo   hoodServo2;
    // private final TalonFX launcherMotor;
    // private final TalonFX launcherFollowerMotor;

    private final boolean hasSecondServo = false;

    // private final Ultrasonic   ultrasonic;
    // private final MedianFilter medianFilter;

    // // IOs
    // private final StatusSignal<Temperature>     temperatureLauncher;
    // private final StatusSignal<Voltage>         voltageLauncher;
    // private final StatusSignal<Current>         currentLauncher;
    // private final StatusSignal<AngularVelocity> speedLauncher;

    // private final StatusSignal<Temperature>     temperatureLauncherFollower;
    // private final StatusSignal<Voltage>         voltageLauncherFollower;
    // private final StatusSignal<Current>         currentLauncherFollower;
    // private final StatusSignal<AngularVelocity> speedLauncherFollower;

    // private final StatusSignal<Temperature>     temperatureHood;
    // private final StatusSignal<Voltage>         voltageHood;
    // private final StatusSignal<Current>         currentHood;
    // private final StatusSignal<AngularVelocity> speedHood;
    // private final StatusSignal<Angle>           hoodPosition;

    public LauncherIOTalonFX(
    //                          int launcherCanID, 
    //                          int launcherSensorID, 
    //                          int launcherFollowerCanID, 
                            //  int hoodCanID,
                             int hoodServoChannel,
                             int hoodServoChannel2
                            //  int hoodSensorID
                            //  DigitalOutput pingChannel, DigitalInput echoChannel
                             ) {
        // Motors and sensors
        // hoodMotor   = new TalonFX(hoodCanID);
        hoodServo   = new Servo(hoodServoChannel);
        if (hasSecondServo)
            hoodServo2  = new Servo(hoodServoChannel2);
        else
            hoodServo2 = null;

        // hoodServo.setBoundsMicroseconds(
        //     LauncherConstants.Hood.MAX_PULSE_WIDTH, 
        //     LauncherConstants.Hood.SERVO_DEADBAND_MAX, 
        //     LauncherConstants.Hood.SERVO_DEADBAND_CENTER, 
        //     LauncherConstants.Hood.SERVO_DEADBAND_MIN, 
        //     LauncherConstants.Hood.MIN_PULSE_WIDTH
        // );

        hoodServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    

        // hoodServo2.setBoundsMicroseconds(
        //     LauncherConstants.Hood.MAX_PULSE_WIDTH, 
        //     LauncherConstants.Hood.SERVO_DEADBAND_MAX, 
        //     LauncherConstants.Hood.SERVO_DEADBAND_CENTER, 
        //     LauncherConstants.Hood.SERVO_DEADBAND_MIN, 
        //     LauncherConstants.Hood.MIN_PULSE_WIDTH
        // );
        // CANcoder hoodSensor = new CANcoder(hoodSensorID);

        // launcherMotor = new TalonFX(launcherCanID);
        // CANcoder launcherSensor = new CANcoder(launcherSensorID);

        // launcherFollowerMotor = new TalonFX(launcherFollowerCanID);

        // ultrasonic = new Ultrasonic(pingChannel, echoChannel);
        // medianFilter = new MedianFilter(0);

        // IOs
        // temperatureLauncher = launcherMotor.getDeviceTemp();
        // voltageLauncher = launcherMotor.getMotorVoltage();
        // currentLauncher = launcherMotor.getSupplyCurrent();
        // speedLauncher = launcherMotor.getVelocity();

        // temperatureLauncherFollower = launcherFollowerMotor.getDeviceTemp();
        // voltageLauncherFollower = launcherFollowerMotor.getMotorVoltage();
        // currentLauncherFollower = launcherFollowerMotor.getSupplyCurrent();
        // speedLauncherFollower = launcherFollowerMotor.getVelocity();

        // temperatureHood = hoodMotor.getDeviceTemp();
        // voltageHood = hoodMotor.getMotorVoltage();
        // currentHood = hoodMotor.getSupplyCurrent();
        // speedHood = hoodMotor.getVelocity();
        // hoodPosition = hoodMotor.getPosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50
                // temperatureLauncher,
                // voltageLauncher,
                // currentLauncher,
                // speedLauncher,

                // temperatureLauncherFollower,
                // voltageLauncherFollower,
                // currentLauncherFollower,
                // speedLauncherFollower

                // temperatureHood,
                // voltageHood,
                // currentHood,
                // speedHood,
                // hoodPosition
        );

        // Configurators
        // TalonFXConfigurator launcherConfigurator = launcherMotor.getConfigurator();
        // TalonFXConfigurator launcherFollowerConfigurator = launcherFollowerMotor.getConfigurator();

        // TalonFXConfigurator hoodConfigurator = hoodMotor.getConfigurator();

        // Slot configs
        Slot0Configs launcherSlotConfigs = new Slot0Configs()
                .withKP(LauncherConstants.Launcher.LAUNCER_PID.kP)
                .withKI(LauncherConstants.Launcher.LAUNCER_PID.kI)
                .withKD(LauncherConstants.Launcher.LAUNCER_PID.kD)
                .withKV(LauncherConstants.Launcher.kV)
                .withKS(LauncherConstants.Launcher.kS)
                .withKG(LauncherConstants.Launcher.kG);

        // launcherConfigurator.apply(launcherSlotConfigs);
        // launcherFollowerConfigurator.apply(launcherSlotConfigs);

        // Slot0Configs hoodSlotConfigs = new Slot0Configs()
        //         .withKP(LauncherConstants.Hood.HOOD_PID.kP)
        //         .withKI(LauncherConstants.Hood.HOOD_PID.kI)
        //         .withKD(LauncherConstants.Hood.HOOD_PID.kD);

        // hoodConfigurator.apply(hoodSlotConfigs);

        // Current limit configs
        CurrentLimitsConfigs launcherCurrentLimitsConfigs = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(LauncherConstants.SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);

        // launcherConfigurator.apply(launcherCurrentLimitsConfigs);
        // launcherFollowerConfigurator.apply(launcherCurrentLimitsConfigs);

        // CurrentLimitsConfigs hoodCurrentLimitConfigs = new CurrentLimitsConfigs()
        //         .withSupplyCurrentLimit(LauncherConstants.SUPPLY_CURRENT_LIMIT)
        //         .withSupplyCurrentLimitEnable(true);

        // hoodConfigurator.apply(hoodCurrentLimitConfigs);

        // Motor output configs
        MotorOutputConfigs launcherOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive);
        MotorOutputConfigs launcherFollowerOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast);

        // launcherConfigurator.apply(launcherOutputConfigs);
        // launcherFollowerConfigurator.apply(launcherFollowerOutputConfigs);

        // MotorOutputConfigs hoodOutputConfigs = new MotorOutputConfigs();

        // hoodConfigurator.apply(hoodOutputConfigs);

        // Feedback configs
        // FeedbackConfigs launcherFeedbackConfigs = new FeedbackConfigs()
        //         .withRemoteCANcoder(launcherSensor);

        // launcherConfigurator.apply(launcherFeedbackConfigs);
        // launcherFollowerConfigurator.apply(launcherFeedbackConfigs);

        // FeedbackConfigs hoodFeedbackConfigs = new FeedbackConfigs()
        //         .withSensorToMechanismRatio(LauncherConstants.Hood.HOOD_GEAR_RATIO);

        // hoodConfigurator.apply(hoodFeedbackConfigs);
        

        // launcherFollowerMotor.setControl(new Follower(launcherCanID, MotorAlignmentValue.Opposed));
    }

    // Voltage
    // @Override
    // public void launcherSetVoltage(double volts) {
    //     launcherMotor.setVoltage(volts);
    // }

    // @Override
    // public void hoodSetVoltage(double volts) {
    //     hoodMotor.setVoltage(volts);
    // }

    // Run systems
    // @Override
    // public void runRPS(Supplier<AngularVelocity> velocity) {
    //     launcherMotor.setControl(new VelocityVoltage(velocity.get())
    //                                      .withSlot(0)
    //                                      .withFeedForward(0));
    // }

    @Override
    public void setHoodAngle(Angle pos) {
        // if (pos.gte(LauncherConstants.Hood.MIN_ANGLE) && pos.lte(LauncherConstants.Hood.MAX_ANGLE)) {
        //     hoodMotor.setPosition(pos);
        // }

        double setpoint = MathUtil.clamp(
                pos.in(Degrees), 
                LauncherConstants.Hood.MIN_ANGLE.in(Degrees), 
                LauncherConstants.Hood.MAX_ANGLE.in(Degrees)
            );
        hoodServo.setAngle(setpoint);
        if (hasSecondServo)
            hoodServo2.setAngle(setpoint);
    }

    @Override
    public void setHoodPos(Distance pos){
        double setpoint = 
            MathUtil.clamp(
                pos.in(Millimeters), 
                LauncherConstants.Hood.MIN_EXTENSION.in(Millimeters), 
                LauncherConstants.Hood.MAX_EXTENSION.in(Millimeters)
            ) / LauncherConstants.Hood.MAX_EXTENSION.in(Millimeters);
        hoodServo.setPosition(setpoint);
        if (hasSecondServo)
            hoodServo2.setPosition(setpoint);
    }

    /**
     * Set hood speed from -1 to 1
     */
    @Override
    public void setHoodSpeed(double setpoint){
        setpoint = MathUtil.clamp(setpoint, -1, 1);
        hoodServo.setSpeed(setpoint);
        if (hasSecondServo)
            hoodServo2.setSpeed(setpoint);
    }

    @Override
    public void setHoodPWM(int pwm){
        hoodServo.setPulseTimeMicroseconds(pwm);
    }



    @Override
    public Distance getHoodPos() {
        // return hoodMotor.getPosition().getValue();
        return Millimeters.of(hoodServo.getPosition() * LauncherConstants.Hood.MAX_EXTENSION.in(Millimeters));
    }

    // @Override
    // public double getDistance() {
    //     return medianFilter.calculate(ultrasonic.getRangeInches());
    // }

    // Stops
    // @Override
    // public void stopLauncher() {
    //     launcherMotor.setVoltage(0);
    // }

    // @Override
    // public void stopHood() {
    //     hoodMotor.setVoltage(0);
    // }

    @Override
    public void updateInputs(LauncherInputs inputs) {
        // Launcher
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

        // inputs.temperatureLauncher = temperatureLauncher.getValueAsDouble();
        // inputs.launcherVoltage = voltageLauncher.getValue();
        // inputs.launcherCurrent = currentLauncher.getValue();
        // inputs.launcherSpeedRadians = speedLauncher.getValue();

        // inputs.launcherFollowerTemperature = temperatureLauncher.getValueAsDouble();
        // inputs.launcherFollowerVoltage = voltageLauncher.getValue();
        // inputs.launcherFollowerCurrent = currentLauncher.getValue();
        // inputs.launcherFollowerSpeedRadians = speedLauncher.getValue();

        // Hood
        // inputs.isHoodConnected = BaseStatusSignal.refreshAll(
        //         voltageHood,
        //         currentHood,
        //         temperatureHood,
        //         speedHood
        // ).isOK();

        // inputs.temperatureHood = temperatureHood.getValueAsDouble();

        // inputs.hoodVoltage = voltageHood.getValue();
        // inputs.hoodCurrent = currentHood.getValue();
        // inputs.hoodSpeedRadians = speedHood.getValue();
        // inputs.hoodPosition = hoodPosition.getValue();
        inputs.hood1Position = hoodServo.getPosition() * LauncherConstants.Hood.MAX_EXTENSION.in(Millimeters);
        inputs.targetHood1PositionMM = hoodServo.get() * LauncherConstants.Hood.MAX_EXTENSION.in(Millimeters);
        inputs.hood1TargetAngle = hoodServo.getAngle();
        inputs.hood1Speed = hoodServo.getSpeed();
        inputs.hood1PWM = hoodServo.getPulseTimeMicroseconds();

        if (hasSecondServo){
            inputs.hood2Position = hoodServo2.getPosition() * LauncherConstants.Hood.MAX_EXTENSION.in(Millimeters);
            inputs.targetHood2PositionMM = hoodServo2.get() * LauncherConstants.Hood.MAX_EXTENSION.in(Millimeters);
            inputs.hood2TargetAngle = hoodServo2.getAngle();
            inputs.hood2Speed = hoodServo2.getSpeed();
            inputs.hood2PWM = hoodServo2.getPulseTimeMicroseconds();
        }

    }
}
