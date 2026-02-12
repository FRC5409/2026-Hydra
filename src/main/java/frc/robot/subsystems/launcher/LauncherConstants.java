package frc.robot.subsystems.launcher;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

public class LauncherConstants {
    public static final int LAUNCHER_SENSOR_ID = 15;
    public static final int LAUNCHER_CAN_ID    = 11;

    public static final int FOLLOWER_LAUNCHER_CAN_ID       = 12;
    public static final int FOLLOWER_LAUNCHER_SENSOR_ID    = 0;

    public static final int HOOD_CAN_ID     = 1;
    public static final int HOOD_SENSOR_ID  = 1;

    public static final int SUPPLY_CURRENT_LIMIT = 60;

    public static final double kG = 0.0;
    public static final double kS = 0.1;
    public static final double kV = 0.12;
    public static final double kA = 0.0;

    public static final PIDController PID = new PIDController(0,0,0);

    public static class Ultrasonic {
        public static DigitalOutput DIGITAL_OUPTPUT = new DigitalOutput(0);
        public static DigitalInput DIGITAL_INPUT = new DigitalInput(1);
    }

    public static class Servo {
        public static int CHANNEL = 1;
        
        public static int MAX_PMW_PULSE_WIDTH = 2;
        public static int DEAD_BAND_MAX = 2;
        public static int PULSE_WIDTH_CENTER = 1;
        public static int DEAD_BAND_MIN = 0;
        public static int MIN_PMW_PULSE_WIDTH = 0;
    }
}
