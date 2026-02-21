package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

/* ALL TEST VALUES */
public final class HopperConstants {

    public static final int MAIN_MOTOR_ID = 20;
    public static final int FOLLOWER_MOTOR_ID = 21;

    public static final Current CURRENT_LIMIT = Amps.of(30.0);
    public static final double kGearing = 10.0 / 1.0;
    public static final Distance HOPPER_DRUMRADIUS = Meters.of(1.0);
    public static final double kCircumfrence = 2 * Math.PI * HOPPER_DRUMRADIUS.in(Inches);
    public static final double kRotationConverter = kCircumfrence / kGearing; //Need values for rack & pinion
    public static final Mass HOPPER_MASS = Pounds.of(10.561);

    public static final PIDConstants TALONFX_PID = new PIDConstants(0.001, 0, 0);
    public static final PIDConstants SIM_PID = new PIDConstants(10, 0, 0);

    public static final Distance HOPPER_MIN_EXTENSION = Inches.of(0.0);
    public static final Distance HOPPER_MAX_EXTENSION = Inches.of(12.0);
    public static final Distance STARTING_GAP_TO_INTAKE = Inches.of(0.4);

}
