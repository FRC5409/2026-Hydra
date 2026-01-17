package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
    public static final String CAM_NAME = "limelight";

    public static final int FIDUCIAL_TRUST_THRESHOLD = 1;

    /**
     * Frames allowed without latency update before flagged as disconnected
     */
    public static final int DISCONNECTION_TIMEOUT = 5;

    public static final int THROTTLE_DISABLED = 200;

    // TODO: update these to camera offset
    public static final Transform3d OFFSET_FROM_ROBOT_ORIGIN = new Transform3d(
            new Translation3d(0,0,0),
            new Rotation3d(0, 0, 0));
}
