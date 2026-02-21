package frc.robot.subsystems.launcher;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.subsystems.launcher.interpolator.BilinearStrategy;
import frc.robot.subsystems.launcher.interpolator.LaunchStrategy;

import static edu.wpi.first.units.Units.*;

public class LauncherConstants {
    public static class Launcher {
        public static final int LAUNCHER_SENSOR_ID = 1;
        public static final int LAUNCHER_CAN_ID    = 11;

        public static final int FOLLOWER_LAUNCHER_CAN_ID    = 12;
        public static final int FOLLOWER_LAUNCHER_SENSOR_ID = 0;

        public static final double kG = 0.0;
        public static final double kS = 0.1;
        public static final double kV = 0.12;
        public static final double kA = 0.0;

        public static final PIDController PID = new PIDController(0.05, 0, 0);

        public static final LaunchStrategy DEFAULT_LAUNCH_STRATEGY = new BilinearStrategy();
    }

    public static final int SUPPLY_CURRENT_LIMIT = 60;

    public static class Hood {
        // values derived from linear map of deg to mm
        public static final Per<DistanceUnit, AngleUnit> MM_PER_DEG = Millimeters.per(Degrees).ofNative(3.37591);
        public static final Distance                     OFFSET_MM  = Millimeters.of(51.27737);

        public static final int HOOD_PWM_CHANNEL_1 = 4;
        public static final int HOOD_PWM_CHANNEL_2 = 5;

        // stored as raw number (degrees) to save computation frames
        public static final double MIN_ANGLE_DEG = 15.18919;
        public static final double MAX_ANGLE_DEG = MIN_ANGLE_DEG + 30;

        public static final Distance       MIN_EXTENSION = Millimeters.of(0.0);
        // TODO: GET REAL VALUE
        public static final Distance       MAX_EXTENSION = Millimeters.of(140);
        // TODO: GET REAL VALUE
        public static final LinearVelocity MAX_SPEED     = Millimeters.per(Second).of(20);

        // public static final Distance IMPOSED_MAX_EXTENSION = Millimeters.of(100);

        // TODO: SEE IF VALUES ARE NECESSARY, IF SO GET WHAT THE VALUES SHOULD BE
        public static final int MAX_PULSE_WIDTH       = 4;
        public static final int SERVO_DEADBAND_MAX    = 3;
        public static final int SERVO_DEADBAND_CENTER = 2;
        public static final int SERVO_DEADBAND_MIN    = 1;
        public static final int MIN_PULSE_WIDTH       = 0;
    }

    public static class Ultrasonic {
        public static DigitalOutput DIGITAL_OUTPUT = new DigitalOutput(0);
        public static DigitalInput  DIGITAL_INPUT  = new DigitalInput(0);

        public static final Per<DistanceUnit, VoltageUnit> MM_PER_VOLT = Millimeters.of(1024.0).div(Volts.of(5.0));
    }
}
