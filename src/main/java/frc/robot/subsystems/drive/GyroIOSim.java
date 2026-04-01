package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {

    private final GyroSimulation gyroSim;

    public GyroIOSim(GyroSimulation gyroSim) {
        this.gyroSim = gyroSim;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.isConnected = true;

        inputs.yawPosition = gyroSim.getGyroReading();
        inputs.yawVelocityRadPerSec = RadiansPerSecond.of(gyroSim.getMeasuredAngularVelocity().in(RadiansPerSecond));
        inputs.odometryYawTimestamps = new double[] {};
        inputs.odometryYawPositions = gyroSim.getCachedGyroReadings();
    }

    @Override
    public  void zeroPigeon(){
        setPigeonYaw(Rotation2d.k180deg);
    }

    @Override
    public  void setPigeonYaw(Rotation2d rotation2d){
        gyroSim.setRotation(rotation2d);
    }
}
