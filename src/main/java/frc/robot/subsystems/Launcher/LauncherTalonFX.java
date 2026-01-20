package frc.robot.subsystems.Launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class LauncherTalonFX implements LauncherIO {
    // check how many motors launcher uses
    private int launcherCanID;
    private int launcherSensorID;

    private int hoodCanID;
    private int hoodSensorID;

    private TalonFX hoodMotor;
    private CANcoder hoodSensor;

    private CANcoder launcherSensor;

    private final VelocityVoltage velocityVoltage;

    TalonFX launcherMotor = new TalonFX(launcherCanID);
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

        // CirrentLimitConfigs
        // MotorOutputConfigs
        // FeedBackConfigs
        // SlotConfigs
    }
    

    @Override
    public void setVoltage(double volts) {
        launcherMotor.setVoltage(volts);
    }

    @Override
    public void prepareFuel() {} // ?

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
