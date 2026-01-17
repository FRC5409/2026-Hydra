package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * @author Logan Dhillon, FRC 5409 Chargers
 */
public class VisionConstants {
    public static final String PRIMARY_CAM_NAME = "limelight";

    public static final int FIDUCIAL_TRUST_THRESHOLD = 1;
    public static final int DISCONNECTION_TIMEOUT    = 5;
    public static final int THROTTLE_DISABLED        = 200;

    // 1σ translation error at zero distance (meters)
    public static final double VISION_XY_STDDEV_BASE_METERS  = 0.10;
    // Additional translation error per meter of tag distance
    public static final double VISION_XY_STDDEV_PER_METER    = 0.05;
    // 1σ rotation error at zero distance (deg)
    public static final double VISION_THETA_STDDEV_BASE_DEG  = 2.0;
    // Additional rotation error per meter of tag distance (deg / meter)
    public static final double VISION_THETA_STDDEV_PER_METER = 1.5;

    // TODO: update these to camera offset
    public static final Transform3d OFFSET_FROM_ROBOT_ORIGIN = new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0));
}
