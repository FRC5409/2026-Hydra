package frc.robot.subsystems.Intake;
import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface IntakeIO {

    @AutoLog
    public class intakeInputs{

        public boolean rollerConnection = false;
        public Voltage rollerVolts = Volts.of(0.0);
        public Current rollerCurrent = 0.0;
        public double rollerTemp = 0.0;
        public Voltage rollerVelocity = Volts.of(0.0);

        public boolean extensionConnection = false;
        public boolean extensionRunning = false;
        public Voltage extensionVolts = Volts.of(0.0);
        public Current extensionCurrent = Amps.of(0.0);
        public double extensionTemp = 0.0;
        public double extensionVelocity = 0.0;
        public double extensionPosition = 0.0;

        public boolean isExtended = false;
        public boolean isRetracted = true;

    }
    public default void setExtensionVoltage(double voltage) {}

    public default void setRollerVoltage(double voltage) {}

    public default void updateInputs(intakeInputs inputs) {}

    public default double getMotorCurrent() {

        return 0.0;

    }

    public default void setSetpoint(Distance position) {}

    public default void coastMode() {}

    public default void brakeMode() {}

    public default void extend() {}

    public default void retract() {}

    public default void stopMotor() {}

    public default Distance getPosition() {return Meters.of(0);}

    }

  
