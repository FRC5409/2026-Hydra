package frc.robot.subsystems.Intake;


import org.littletonrobotics.junction.AutoLog;
public interface IntakeIO {
    @AutoLog
    public class intakeInputs{
        public boolean rollerConnection = false;
        public double rollerVolts = 0.0;
        public double rollerCurrent = 0.0;
        public double rollerTemp = 0.0;
        public double rollerVelocity = 0.0;
        public double rollerPosition = 0.0;

        public boolean extensionConnection = false;
        public double extensionVolts = 0.0;
        public double extensionCurrent = 0.0;
        public double extensionTemp = 0.0;
        public double extensionVelocity = 0.0;
        public double extensionPosition = 0.0;

        public boolean isExtended = false;
        public boolean isRetracted = true;
    }
    public default void setVoltage(double volts) {}

    public default void updateInputs(intakeInputs inputs) {}

    public default double getMotorCurrent() {
        return 0.0;
    }

    public default void setSetpointCommand(edu.wpi.first.units.measure.Distance position) {}

    public default void coastMode() {}

    public default void brakeMode() {}

    public default void extend() {}

    public default void retract() {}

    }

  
