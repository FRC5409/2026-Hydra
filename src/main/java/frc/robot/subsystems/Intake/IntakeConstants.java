package frc.robot.subsystems.Intake;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;
import com.pathplanner.lib.config.PIDConstants;

public final class IntakeConstants {

	public static final class kExtension {

		public static final Distance EXTENSION_MIN_DISTANCE = Meters.of(0.0);
		public static final Distance EXTENSION_MAX_DISTANCE = Meters.of(0.5);

		public static PIDConstants TALONFX_PIDCONSTANTS = new PIDConstants(1.0, 0.01, 0.0);
		public static PIDConstants SIM_PIDCONSTANTS = new PIDConstants(1.0, 0.0, 0.0);
		public static final Voltage MAXVOLTAGE = Volts.of(12.0);
		public static final Current MAXCURRENT = Amps.of(40.0);
		public static final double GEARING = 10.0;
		public static final Mass INTAKE_MASS = Kilograms.of(1.0);
		public static final Distance INTAKE_DRUMRADIUS = Meters.of(0.0254);
		public static final Distance INTAKE_MIN_DISTANCE = Meters.of(0.0);
		public static final Distance INTAKE_MAX_DISTANCE = Meters.of(0.5); 

	}

    public static final class kRoller {

        public static final Voltage kMaxVoltage = Volts.of(12.0);
        public static final Current kMaxCurrent = Amps.of(40.0);
        public static final double kGearing = 10.0;
        public static final Mass ROLLER_MASS = Kilograms.of(1.0);
        public static final Distance ROLLER_DRUMRADIUS = Meters.of(0.0254);

    }
}