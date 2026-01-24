package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;
import com.pathplanner.lib.config.PIDConstants;

public final class ElevatorConstants {

    public static final int MAIN_MOTOR_ID = 20;
    public static final double CURRENT_LIMIT = 0.0;
    public static final double kGearing = 0.0;
    public static final Distance ELEVATOR_DRUMRADIUS = Inches.of(0.0);
    public static final double kCircumfrence = 2 * Math.PI * ELEVATOR_DRUMRADIUS.in(Meters);
    public static final double kRotationConverter = kCircumfrence / kGearing;
    public static final PIDConstants TALONFX_PID = new PIDConstants(0, 0, 0);
    public static final PIDConstants SIM_PID = new PIDConstants(0, 0, 0);
    public static final Mass ELEVATOR_MASS = Pound.of(0);
    public static final double ELEVATOR_MIN_HEIGHT = 0.0;
    public static final double ELEVATOR_MAX_HEIGHT = 0.0;

    public static final Distance ELEVATOR_PREP_HEIGHT = Meters.of(0);

    public static final Distance IDLING_HEIGHT = Meters.of(0);

}