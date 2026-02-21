package frc.robot.subsystems.elevator;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

import static edu.wpi.first.units.Units.*;

public final class ElevatorConstants {
    // TODO: Replace with real climber current limit after testing
    public static final Current      CURRENT_LIMIT        = Amps.of(30.0);
    public static final double       GEARING              = 9.0; // TODO: Replace with actual gearbox ratio
    public static final Distance     ELEVATOR_DRUM_RADIUS = Inches.of(20.0);
    public static final PIDConstants TALONFX_PID          = new PIDConstants(0, 0, 0);
    public static final PIDConstants SIM_PID              = new PIDConstants(10, 0, 0);
    public static final Mass         ELEVATOR_MASS        = Pound.of(1);
    public static final Distance     ELEVATOR_MIN_HEIGHT  = Inches.of(10.0);
    public static final Distance     ELEVATOR_MAX_HEIGHT  = Inches.of(20.0);

    public static final Distance ELEVATOR_PREP_HEIGHT = Meters.of(15.0);
    public static final Distance IDLING_HEIGHT        = Meters.of(11.0);
}