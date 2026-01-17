package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

/**
 * @author Logan Dhillon, FRC 5409 Chargers
 */
public class VisionIOLimelight implements VisionIO {
    private double lastPrxLatency = 0;
    private double disconnectedFrames = 0;

    public VisionIOLimelight() {
        new Trigger(DriverStation::isDisabled)
                .onTrue(
                        Commands.parallel(
                                setIMUMode(IMUMode.FUSED),
                                setThrottle(VisionConstants.THROTTLE_DISABLED)
                        )
                ).onFalse(
                        Commands.parallel(
                                setIMUMode(IMUMode.INTERNAL),
                                setThrottle(0)
                        )
                );

        // TODO: add throttle debug commands, i'll do this when my FRC Checkmate library is done
//        DebugCommand.register("No Throttle LL", setThrottle(0));
//        DebugCommand.register("Throttle LL", setThrottle(VisionConstants.THROTTLE_DISABLED));
//        DebugCommand.register("Fused LL", setIMUMode(IMUMode.FUSED));
//        DebugCommand.register("Internal LL", setIMUMode(IMUMode.INTERNAL));

        LimelightHelpers.SetThrottle(VisionConstants.PRIMARY_CAM_NAME, VisionConstants.THROTTLE_DISABLED);
        LimelightHelpers.SetIMUMode(VisionConstants.PRIMARY_CAM_NAME, IMUMode.FUSED.ID);
    }

    private Command setThrottle(int throttle) {
        return Commands.runOnce(
                () -> LimelightHelpers.SetThrottle(VisionConstants.PRIMARY_CAM_NAME, throttle)
        ).ignoringDisable(true);
    }

    private Command setIMUMode(IMUMode mode) {
        return Commands.runOnce(
                () -> LimelightHelpers.SetIMUMode(VisionConstants.PRIMARY_CAM_NAME, mode.ID)
        ).ignoringDisable(true);
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.tx = LimelightHelpers.getTX(VisionConstants.PRIMARY_CAM_NAME);
        inputs.ty = LimelightHelpers.getTY(VisionConstants.PRIMARY_CAM_NAME);
        inputs.ta = LimelightHelpers.getTA(VisionConstants.PRIMARY_CAM_NAME);
        inputs.hasTarget = LimelightHelpers.getTV(VisionConstants.PRIMARY_CAM_NAME);
        inputs.targetId = LimelightHelpers.getFiducialID(VisionConstants.PRIMARY_CAM_NAME);
        inputs.imgLatency = LimelightHelpers.getLatency_Capture(VisionConstants.PRIMARY_CAM_NAME);
        inputs.prxLatency = LimelightHelpers.getLatency_Pipeline(VisionConstants.PRIMARY_CAM_NAME);

        double[] hw = LimelightHelpers.getLimelightNTTableEntry(VisionConstants.PRIMARY_CAM_NAME, "hw")
                                      .getDoubleArray(new double[]{ 0.0, 0.0, 0.0, 0.0 });

        try {
            inputs.fps = hw[3];
            inputs.cpuTemp = hw[2];
            inputs.ramUsage = hw[1];
            inputs.sysTemp = hw[0];
        } catch (Exception e) {
            inputs.fps = -1.0;
            inputs.cpuTemp = -1.0;
            inputs.ramUsage = -1.0;
            inputs.sysTemp = -1.0;
        }

        // check if disconnected by comparing prx latency
        if (lastPrxLatency != inputs.prxLatency) {
            disconnectedFrames = 0;
            inputs.isConnected = true;
        } else {
            inputs.isConnected = ++disconnectedFrames <= VisionConstants.DISCONNECTION_TIMEOUT;
        }

        lastPrxLatency = inputs.prxLatency;
    }

    @Override
    public void setCameraOffset() {
        LimelightHelpers.setCameraPose_RobotSpace(
                VisionConstants.PRIMARY_CAM_NAME,
                VisionConstants.OFFSET_FROM_ROBOT_ORIGIN.getTranslation().getX(),
                VisionConstants.OFFSET_FROM_ROBOT_ORIGIN.getTranslation().getY(),
                VisionConstants.OFFSET_FROM_ROBOT_ORIGIN.getTranslation().getZ(),
                VisionConstants.OFFSET_FROM_ROBOT_ORIGIN.getRotation().getMeasureX().in(Units.Degrees),
                VisionConstants.OFFSET_FROM_ROBOT_ORIGIN.getRotation().getMeasureY().in(Units.Degrees),
                VisionConstants.OFFSET_FROM_ROBOT_ORIGIN.getRotation().getMeasureZ().in(Units.Degrees));
    }

    private void logGryoMode(String mode) {
        Logger.recordOutput("Vision/Gyro-Mode", mode);
    }

    /**
     * Updates the current robot orientation in {@link LimelightHelpers}, then gets the
     * {@link frc.robot.util.LimelightHelpers.PoseEstimate} using WPI Blue MegaTag2.
     *
     * @param drive Drive subsystem to get rotation from
     */
    @Override
    public LimelightHelpers.PoseEstimate estimatePose(Drive drive) {
        LimelightHelpers.SetRobotOrientation(VisionConstants.PRIMARY_CAM_NAME,
                                             drive.getRotation().getDegrees(),
                                             0, 0, 0, 0, 0);
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.PRIMARY_CAM_NAME);
    }

    @Override
    public void setRotation(Rotation2d rotation) {
        // use fused IMU when setting rotation
        LimelightHelpers.SetIMUMode(VisionConstants.PRIMARY_CAM_NAME, IMUMode.FUSED.ID);
        LimelightHelpers.SetRobotOrientation(VisionConstants.PRIMARY_CAM_NAME, rotation.getDegrees(),
                                             0, 0, 0, 0, 0);
    }

    /**
     * Forward limelight ports (5800-5809) so it can be used over USB
     */
    public static void forwardLimelightPorts() {
        for (int i = 5800; i <= 5809; i++)
            PortForwarder.add(i, VisionConstants.PRIMARY_CAM_NAME + ".local", i);
    }

    public enum IMUMode {
        EXTERNAL(0),
        FUSED(1),
        INTERNAL(2);

        public final int ID;

        IMUMode(int num) {
            ID = num;
        }
    }
}

