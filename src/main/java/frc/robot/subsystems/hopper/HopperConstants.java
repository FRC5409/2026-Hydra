package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/* ALL TEST VALUES */
public final class HopperConstants {

    public static final Current CURRENT_LIMIT = Amps.of(30.0);
    public static final Current DAMAGE_DETECTION_CURRENT = Amps.of(50.0); //TODO: test on real robot
    public static final boolean CRASH_DETECTION_ENABLED = true;

    public static final double GEARING = 12.0 / 1.0;

    public static final Distance HOPPER_DRUMRADIUS = Meters.of(1.0);
    public static final Mass HOPPER_MASS = Pounds.of(10.561);    

    public static final PIDConstants TALONFX_PID = new PIDConstants(4.5,0.55,0);


    public static final PIDConstants SIM_PID = new PIDConstants(4, 0, 3);

    public static final Distance HOPPER_MIN_EXTENSION = Meters.of(0.0);
    public static final Distance HOPPER_MAX_EXTENSION = Meters.of(2.7);
    public static final Distance STARTING_GAP_TO_INTAKE = Meters.of(0.0);
    public static final Distance EXTEND_INCREMENT = Inches.of(0.3);
    public static final Distance AGITATE_TOLERANCE = Meters.of(0.001);

}
