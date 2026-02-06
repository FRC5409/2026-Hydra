package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;

public class LauncherConstants {
    public static class Launcher {
        public static final int LAUNCHER_SENSOR_ID = 1;
        public static final int LAUNCHER_CAN_ID    = 11;

        public static final int FOLLOWER_LAUNCHER_CAN_ID       = 12;
        public static final int FOLLOWER_LAUNCHER_SENSOR_ID    = 0;

        public static final double kG = 0.0;
        public static final double kS = 0.1;
        public static final double kV = 0.12;
        public static final double kA = 0.0;

        public static final PIDController PID = new PIDController(0.05,0,0);
    }

    // public static final int LAUNCHER_SENSOR_ID = 1;
    // public static final int LAUNCHER_CAN_ID    = 11;

    // public static final int FOLLOWER_LAUNCHER_CAN_ID       = 12;
    // public static final int FOLLOWER_LAUNCHER_SENSOR_ID    = 0;

    // public static final int HOOD_CAN_ID     = 1;
    // public static final int HOOD_SENSOR_ID  = 1;

    public static final int SUPPLY_CURRENT_LIMIT = 30;

    // public static final double kG = 0.0;
    // public static final double kS = 0.1;
    // public static final double kV = 0.12;
    // public static final double kA = 0.0;

    // public static final PIDController PID = new PIDController(0,0,0);

    public static class Hood {
        public static final int HOOD_CAN_ID     = 1;
        public static final int HOOD_SENSOR_ID  = 1;

        public static final Angle MIN_ANGLE = Degrees.of(0);
        public static final Angle MAX_ANGLE = Degrees.of(30);

        public static final PIDController PID = new PIDController(0.0, 0.0, 0.0);
    }
}
