package frc.robot.subsystems.Intake;


import org.littletonrobotics.junction.AutoLog;
public interface IntakeIO {
    @AutoLog
    public class intakeInputs{
        public boolean intakeConnection = false;
        public double intakeVolts = 0.0;
        public double intakeCurrent = 0.0;
        public double intakeTemp = 0.0;
        public double intakeVelocity = 0.0;
        public double intakePosition = 0.0;
    }
    public default void setVoltage(double volts) {}

    public default void updateInputs(intakeInputs inputs) {}

    public default double getMotorCurrent() {
        return 0.0;
    }

    public default void coastMode() {}

    public default void brakeMode() {}

    }

  
