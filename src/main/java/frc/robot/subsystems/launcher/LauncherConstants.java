package frc.robot.subsystems.launcher;

import edu.wpi.first.math.controller.PIDController;

public class LauncherConstants {
    public static final int LAUNCHER_SENSOR_ID = 15;
    public static final int LAUNCHER_CAN_ID    = 11;

    public static final int FOLLOWER_LAUNCHER_CAN_ID       = 12;
    public static final int FOLLOWER_LAUNCHER_SENSOR_ID    = 0;

    public static final int HOOD_CAN_ID     = 1;
    public static final int HOOD_SENSOR_ID  = 1;

    public static final int SUPPLY_CURRENT_LIMIT = 30;

    public static final double kG = 0.0;
    public static final double kS = 0.1;
    public static final double kV = 0.12;
    public static final double kA = 0.0;

    public static final PIDController PID = new PIDController(0,0,0);
}
