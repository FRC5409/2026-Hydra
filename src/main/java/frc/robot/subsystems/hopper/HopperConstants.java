package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pound;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

/* ALL TEST VALUES */
public final class HopperConstants {

    public static final int MAIN_MOTOR_ID = 20;
    public static final int FOLLOWER_MOTOR_ID = 21;

    public static final Current CURRENT_LIMIT = Amps.of(30.0);
    public static final double kGearing = 9.0 / 1.0;
    public static final Distance HOPPER_DRUMRADIUS = Inches.of(1.751 / 2.0);
    public static final double kCircumfrence = 2 * Math.PI * HOPPER_DRUMRADIUS.in(Meters);
    public static final double kRotationConverter = kCircumfrence / kGearing;
    public static final Mass HOPPER_MASS = Pound.of(52.95);

    public static final PIDConstants TALONFX_PID = new PIDConstants(0.001, 0, 0);
    public static final PIDConstants SIM_PID = new PIDConstants(0.001, 0, 0);

    public static final Distance HOPPER_MIN_EXTENSION = Meters.of(0.0);
    public static final Distance HOPPER_MAX_EXTENSION = Meters.of(0.3);

    public static final Distance PUMP_EXTENSION_POINT = Meters.of(0.25);
    public static final Distance PUMP_RETRACTION_POINT = Meters.of(0.1);
    public static final Distance TOLERANCE = Meters.of(0.02);
}
