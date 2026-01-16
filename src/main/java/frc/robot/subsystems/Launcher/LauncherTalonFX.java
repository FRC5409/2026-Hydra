package frc.robot.subsystems.Launcher;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;

public class LauncherTalonFX implements LauncherIO{
    // check how many motors launcher uses
    int hopperCanID;
    int hopperSensorID;

    int launcherCanID;
    int launcherSensorID;

    int hoodCanID;
    int hoodSensorID;


    public LauncherTalonFX (int hopperCanID, int hopperSensorID,
                            int launcherCanID, int launcherSensorID,
                            int hoodCanID, int hoodSensorID) {
        this.hopperCanID = hopperCanID;
        this.hopperSensorID = hopperSensorID;

        this.launcherCanID = launcherCanID;
        this.launcherSensorID = launcherSensorID;

        this.hoodCanID = hoodCanID;
        this.hoodSensorID = hoodSensorID;
    }

    TalonFX hopperMotor = new TalonFX(hopperCanID);
    CANcoder hopperSensor = new CANcoder(hopperSensorID);

    TalonFX launcherMotor = new TalonFX(launcherCanID);
    CANcoder launcherSensor = new CANcoder(launcherSensorID);

    TalonFX hoodMotor = new TalonFX(hoodCanID);
    CANcoder hoodSensor = new CANcoder(hoodSensorID);

    @Override
    public void setVoltage(double volts) {}

    @Override
    public void prepareFuel() {}

    @Override
    public void shootFuel() {}

    @Override
    public void setHoodPos(Angle pos) {}

    @Override
    public Angle getHoodPos() {return null;}

    @Override
    public void stop() {}

    @Override
    public void updateInputs(LauncherIO inputs) {}
}
