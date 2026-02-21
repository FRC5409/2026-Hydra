package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Volts;

public class LauncherIOTalonFX implements LauncherIO {
    // Motors and sensors
    private final TalonFX launcherMotor;
    private final Servo   hoodServo;
    private final Servo   hoodServo2;

    private final AnalogInput ultrasonic;

    private double servo1Pos;
    private double servo2Pos;
    private double servo1Setpoint;
    private double servo2Setpoint;

    // IOs
    private final StatusSignal<Temperature>     temperatureLauncher;
    private final StatusSignal<Voltage>         voltageLauncher;
    private final StatusSignal<Current>         currentLauncher;
    private final StatusSignal<AngularVelocity> speedLauncher;

    private final StatusSignal<Temperature>     temperatureLauncherFollower;
    private final StatusSignal<Voltage>         voltageLauncherFollower;
    private final StatusSignal<Current>         currentLauncherFollower;
    private final StatusSignal<AngularVelocity> speedLauncherFollower;

    public LauncherIOTalonFX(
            int launcherCanID,
            int launcherFollowerCanID,
            int ultrasonicChannel,
            int servoChannel,
            int servoChannel2
    ) {
        // Motors and sensors
        launcherMotor = new TalonFX(launcherCanID);

        TalonFX launcherFollowerMotor = new TalonFX(launcherFollowerCanID);

        hoodServo = new Servo(servoChannel);
        hoodServo2 = new Servo(servoChannel2);

        ultrasonic = new AnalogInput(ultrasonicChannel);

        hoodServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        hoodServo2.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

        // IOs
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
        FeedbackConfigs launcherFeedbackConfigs = new FeedbackConfigs();

        launcherConfigurator.apply(launcherFeedbackConfigs);
        launcherFollowerConfigurator.apply(launcherFeedbackConfigs);

        launcherFollowerMotor.setControl(new Follower(launcherCanID, MotorAlignmentValue.Opposed));
    }

    // Voltage
    @Override
    public void launcherSetVoltage(double volts) {
        launcherMotor.setVoltage(volts);
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
    public void setHoodExtension(Distance ext) {
        double setpoint = MathUtil.clamp(ext.in(Millimeters), 0, LauncherConstants.Hood.MAX_EXTENSION.in(Millimeters));
        servo1Setpoint = setpoint;
        servo2Setpoint = setpoint;
        setpoint = (ext.in(Millimeters) / LauncherConstants.Hood.MAX_EXTENSION.in(Millimeters) * 2) - 1;
        hoodServo.setSpeed(setpoint + 18);
        hoodServo2.setSpeed(setpoint + 24);
    }

    @Override
    public Distance getHoodExtension() {
         return Millimeters.of(hoodServo.getPosition());
    }

    /**
     * Run this method in any periodic function to update the position estimation of your servo
     */
    private void updateServos() {
        double epsilon = 30 * Timer.getFPGATimestamp();
        // SERVO 1
        if (servo1Pos > servo1Setpoint + epsilon) servo1Pos -= epsilon;
        else if (servo1Pos < servo1Setpoint - epsilon) servo1Pos += epsilon;
        else servo1Pos = servo1Setpoint;
        // SERVO 2
        if (servo2Pos > servo2Setpoint + epsilon) servo2Pos -= epsilon;
        else if (servo2Pos < servo2Setpoint - epsilon) servo2Pos += epsilon;
        else servo2Pos = servo2Setpoint;
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
        // doesn't log anything but is required for servos to work
        updateServos();

        // Launcher
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

        inputs.launcherTemperature = temperatureLauncher.getValueAsDouble();
        inputs.launcherVoltage = voltageLauncher.getValue();
        inputs.launcherCurrent = currentLauncher.getValue();
        inputs.launcherVelocity = speedLauncher.getValue();

        inputs.launcherFollowerTemperature = temperatureLauncher.getValueAsDouble();
        inputs.launcherFollowerVoltage = voltageLauncher.getValue();
        inputs.launcherFollowerCurrent = currentLauncher.getValue();
        inputs.launcherFollowerVelocity = speedLauncher.getValue();

        inputs.hoodServo1Pos = Millimeters.of(servo1Pos);
        inputs.hoodServo2Pos = Millimeters.of(servo2Pos);
        inputs.hoodServo1Target = Millimeters.of(servo1Setpoint);
        inputs.hoodServo2Target = Millimeters.of(servo2Setpoint);

        inputs.ultrasonicVoltage = getUltrasonicVolts();

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
    }
}
