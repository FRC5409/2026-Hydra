package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Volts;

public class LauncherIOTalonFX implements LauncherIO {
    // Motors and sensors
    private final TalonFX launcherMotor;

    private final Servo   hoodServo;
    private final Servo   hoodServo2;

    private final AnalogInput ultrasonic;

    private double servo1CurPos;
    private double servo2CurPos;
    private double servo1Setpoint;
    private double servo2Setpoint;

    // IOs
    private final StatusSignal<MagnetHealthValue>   magnetHealth;
    
    private final StatusSignal<Temperature>     temperatureLauncher;
    private final StatusSignal<Voltage>         voltageLauncher;
    private final StatusSignal<Current>         currentLauncher;
    private final StatusSignal<AngularVelocity> speedLauncher;

    private final StatusSignal<Temperature>     temperatureLauncherFollower;
    private final StatusSignal<Voltage>         voltageLauncherFollower;
    private final StatusSignal<Current>         currentLauncherFollower;
    private final StatusSignal<AngularVelocity> speedLauncherFollower;

    public LauncherIOTalonFX(
            int launcherCANCoderID,
            int launcherCanID,
            int launcherFollowerCanID,
            int ultrasonicChannel,
            int servoChannel,
            int servoChannel2
    ) {
        // Motors and sensors
        launcherMotor = new TalonFX(launcherCanID);

        CANcoder launcherCANCoder = new CANcoder(launcherCANCoderID);

        TalonFX launcherFollowerMotor = new TalonFX(launcherFollowerCanID);

        hoodServo = new Servo(servoChannel);
        hoodServo2 = new Servo(servoChannel2);

        ultrasonic = new AnalogInput(ultrasonicChannel);

        hoodServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        hoodServo2.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

        // IOs
        magnetHealth = launcherCANCoder.getMagnetHealth();

        temperatureLauncher = launcherMotor.getDeviceTemp();
        voltageLauncher = launcherMotor.getMotorVoltage();
        currentLauncher = launcherMotor.getSupplyCurrent();
        speedLauncher = launcherMotor.getVelocity();

        temperatureLauncherFollower = launcherFollowerMotor.getDeviceTemp();
        voltageLauncherFollower = launcherFollowerMotor.getMotorVoltage();
        currentLauncherFollower = launcherFollowerMotor.getSupplyCurrent();
        speedLauncherFollower = launcherFollowerMotor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,

                magnetHealth,

                temperatureLauncher,
                voltageLauncher,
                currentLauncher,
                speedLauncher,

                temperatureLauncherFollower,
                voltageLauncherFollower,
                currentLauncherFollower,
                speedLauncherFollower
        );

        // Configurators
        TalonFXConfigurator launcherConfigurator = launcherMotor.getConfigurator();
        TalonFXConfigurator launcherFollowerConfigurator = launcherMotor.getConfigurator();

        launcherCANCoder.getConfigurator()      // check sensor configs
                        .apply(new CANcoderConfiguration()
                                .MagnetSensor
                                .withSensorDirection(SensorDirectionValue.Clockwise_Positive));

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

        // Current limit configs
        CurrentLimitsConfigs launcherCurrentLimitsConfigs = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(LauncherConstants.SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);

        launcherConfigurator.apply(launcherCurrentLimitsConfigs);
        launcherFollowerConfigurator.apply(launcherCurrentLimitsConfigs);

        // Motor output configs
        launcherConfigurator.apply(
                new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(InvertedValue.CounterClockwise_Positive));
        launcherFollowerConfigurator.apply(
                new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast));

        // Feedback configs
        FeedbackConfigs launcherFeedbackConfigs = new FeedbackConfigs()
                .withRemoteCANcoder(launcherCANCoder);

        launcherConfigurator.apply(launcherFeedbackConfigs);
        launcherFollowerConfigurator.apply(launcherFeedbackConfigs);

        launcherFollowerMotor.setControl(new Follower(launcherCanID, MotorAlignmentValue.Opposed));
    }

    // Run systems
    @Override
    public void runVelocity(Supplier<AngularVelocity> velocity) {
        launcherMotor.setControl(
                new VelocityVoltage(velocity.get())
                        .withSlot(0)
                        .withFeedForward(0));
    }

    @Override
    public void updateHood(Distance extension) {
        double targetSetpoint = extension.in(Millimeters);
        double t = Timer.getFPGATimestamp();

        // update servo continuous positions
        if (servo1CurPos > servo1Setpoint + 30 * t) servo1CurPos -= 30 * t;
        else if (servo1CurPos < servo1Setpoint - 30 * t) servo1CurPos += 30 * t;
        else servo1CurPos = servo1Setpoint;

        if (servo2CurPos > servo2Setpoint + 30 * t) servo2CurPos -= 30 * t;
        else if (servo2CurPos < servo2Setpoint - 30 * t) servo2CurPos += 30 * t;
        else servo2CurPos = servo2Setpoint;

        // update applied setpoints for servos
        double setpoint1 = targetSetpoint + 20;
        double appliedSetpoint = MathUtil.clamp(setpoint1, 0, LauncherConstants.Hood.MAX_EXTENSION.in(Millimeters));
        servo1Setpoint = appliedSetpoint;
        appliedSetpoint = (setpoint1 / LauncherConstants.Hood.MAX_EXTENSION.in(Millimeters) * 2) - 1;
        hoodServo.setSpeed(appliedSetpoint);

        double setpoint2 = targetSetpoint + 24;
        appliedSetpoint = MathUtil.clamp(setpoint2, 0, LauncherConstants.Hood.MAX_EXTENSION.in(Millimeters));
        servo2Setpoint = appliedSetpoint;
        appliedSetpoint = (setpoint2 / LauncherConstants.Hood.MAX_EXTENSION.in(Millimeters) * 2) - 1;
        hoodServo2.setSpeed(appliedSetpoint);
    }

    @Override
    public Distance getHoodExtension() {
        return Millimeters.of(hoodServo.getPosition());
    }

    @Override
    public Voltage getUltrasonicVolts() {
        return Volts.of(ultrasonic.getVoltage());
    }

    // Stops
    @Override
    public void stopLauncher() {
        launcherMotor.setVoltage(0);
    }

    @Override
    public void updateInputs(LauncherInputs inputs) {
        // Launcher
        inputs.isCANCoderConnected = BaseStatusSignal.refreshAll(magnetHealth).isOK();

        inputs.isLauncherConnected = BaseStatusSignal.refreshAll(
                voltageLauncher,
                currentLauncher,
                temperatureLauncher,
                speedLauncher
        ).isOK();

        inputs.magnetHealth = magnetHealth.getValue();

        inputs.launcherTemperature = temperatureLauncher.getValueAsDouble();
        inputs.launcherVoltage = voltageLauncher.getValue();
        inputs.launcherCurrent = currentLauncher.getValue();
        inputs.launcherVelocity = speedLauncher.getValue();

        inputs.isLauncherFollowerConnected = BaseStatusSignal.refreshAll(
                voltageLauncherFollower,
                currentLauncherFollower,
                temperatureLauncherFollower,
                speedLauncherFollower
        ).isOK();
        inputs.launcherFollowerTemperature = temperatureLauncher.getValueAsDouble();
        inputs.launcherFollowerVoltage = voltageLauncher.getValue();
        inputs.launcherFollowerCurrent = currentLauncher.getValue();
        inputs.launcherFollowerVelocity = speedLauncher.getValue();

        // Hood
        inputs.hoodServo1Pos = Millimeters.of(servo1CurPos);
        inputs.hoodServo2Pos = Millimeters.of(servo2CurPos);
        inputs.hoodServo1Target = Millimeters.of(servo1Setpoint);
        inputs.hoodServo2Target = Millimeters.of(servo2Setpoint);

        inputs.ultrasonicVoltage = getUltrasonicVolts();
    }
}
