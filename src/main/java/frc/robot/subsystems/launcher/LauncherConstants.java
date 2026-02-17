package frc.robot.subsystems.launcher;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Seconds;

import javax.sound.sampled.Line;

import com.pathplanner.lib.config.PIDConstants;

public class LauncherConstants {
    public static class Launcher {
        public static final int LAUNCHER_SENSOR_ID = 1;
        public static final int LAUNCHER_CAN_ID    = 24;

        public static final int FOLLOWER_LAUNCHER_CAN_ID    = 25;
        public static final int FOLLOWER_LAUNCHER_SENSOR_ID = 0;

        public static final double kG = 0.0;
        public static final double kS = 0.1;
        public static final double kV = 0.12;
        public static final double kA = 0.0;

        // KEEP I 0
        public static final PIDController PID         = new PIDController(0.00, 0, 0);
        public static final PIDConstants  LAUNCER_PID = new PIDConstants(PID.getP(), PID.getI(), PID.getD());
        
    }

    public static final int SUPPLY_CURRENT_LIMIT = 60;

    public static class Hood {

        public static final int HOOD_CAN_ID    = 1;
        public static final int HOOD_SENSOR_ID = 1;
        public static final int HOOD_PWM_CHANNEL_1 = 4;
        public static final int HOOD_PWM_CHANNEL_2 = 5;


        public static final Angle MIN_ANGLE = Degrees.of(0);
        public static final Angle MAX_ANGLE = Degrees.of(30);

        public static final Distance MIN_EXTENSION      = Millimeters.of(0.0);
        // TODO: GET REAL VALUE
        public static final Distance MAX_EXTENSION      = Millimeters.of(140);
        // TODO: GET REAL VALUE
        public static final LinearVelocity MAX_SPEED    = Millimeters.per(Seconds).of(20);

        // public static final Distance IMPOSED_MAX_EXTENSION = Millimeters.of(100);

        // TODO: SEE IF VALUES ARE NECESSARY, IF SO GET WHAT THE VALUES SHOULD BE
        public static final int MAX_PULSE_WIDTH = 4;
        public static final int SERVO_DEADBAND_MAX  = 3;
        public static final int SERVO_DEADBAND_CENTER  = 2;
        public static final int SERVO_DEADBAND_MIN  = 1;
        public static final int MIN_PULSE_WIDTH = 0;



        // TODO: Update to real values
        public static final double HOOD_GEAR_RATIO = 1/1;

        // Start with 0 and work to tune, do not start with set values
        public static final PIDController PID       = new PIDController(0.0, 0.0, 0.0);
        public static final PIDConstants  HOOD_PID  = new PIDConstants(PID.getP(), PID.getI(), PID.getD());

    }

    public static class Ultrasonic {
        public static DigitalOutput DIGITAL_OUTPUT = new DigitalOutput(0);
        public static DigitalInput  DIGITAL_INPUT  = new DigitalInput(0);
    }
}
