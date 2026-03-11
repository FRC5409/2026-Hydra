package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.IntakeConstants.Extension;
import frc.robot.util.PhoenixUtil;

public class HopperIOTalonFX implements HopperIO {
    private final TalonFX m_mainMotor;

    private final TalonFXConfigurator m_mainMotorConfig;

    private Distance motorSetpoint;
    private PositionVoltage m_request;
    
    // Data that the encoders will return
    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<Voltage> deviceVoltage;
    private final StatusSignal<Current> deviceCurrent;
    private final StatusSignal<Temperature> deviceTemp;
    private final StatusSignal<Current> torqueCurrent;

    public HopperIOTalonFX(int mainMotorID) {
        m_mainMotor = new TalonFX(mainMotorID);

        m_mainMotorConfig = m_mainMotor.getConfigurator(); // Gets the motor's conifgurator and assigns it to a variable
                                                            // to apply various configurations such as PID and gear ratios

        // SmartDashboard.putData("Publish PIDFF", Commands.runOnce(() -> 
        //     m_mainMotor.getConfigurator().apply(
        //                 new Slot0Configs()
        //                         .withKP(HopperConstants.PID.getP())
        //                         .withKI(HopperConstants.PID.getI())
        //                         .withKD(HopperConstants.PID.getD())
        //                         .withKS(HopperConstants.KS.get())
        //                         .withKV(HopperConstants.KV.get()))));

        // LoggedNetworkBoolean isTrue = new LoggedNetworkBoolean("Hopper/Publish", false);

        // new Trigger( () -> isTrue.get())
        // .onTrue(Commands.runOnce(() -> 
        //     m_mainMotor.getConfigurator().apply(
        //                 new Slot0Configs()
        //                         .withKP(HopperConstants.PID.getP())
        //                         .withKI(HopperConstants.PID.getI())
        //                         .withKD(HopperConstants.PID.getD())
        //                         .withKS(HopperConstants.KS.get())
        //                         .withKV(HopperConstants.KV.get()))));

        final CurrentLimitsConfigs m_currentConfig = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(HopperConstants.CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);
        m_mainMotorConfig.apply(m_currentConfig);

        // Applies the gear ratio to the motor
        final FeedbackConfigs m_encoderConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(HopperConstants.GEARING);
        m_mainMotorConfig.apply(m_encoderConfigs);

        final Slot0Configs m_pidConfig =
        //  Constants.IS_TUNING
        // ? new Slot0Configs()
        //     .withKP(HopperConstants.PID.getP())
        //     .withKI(HopperConstants.PID.getI())
        //     .withKD(HopperConstants.PID.getD()):
        new Slot0Configs()
            .withKP(HopperConstants.TALONFX_PID.kP)
            .withKI(HopperConstants.TALONFX_PID.kI)
            .withKD(HopperConstants.TALONFX_PID.kD);

        m_mainMotorConfig.apply(m_pidConfig); // Applies PID

        // Determines which way the motor should spin with positive voltage
        m_mainMotorConfig.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        m_mainMotor.setNeutralMode(NeutralModeValue.Brake);

        m_request = new PositionVoltage(0).withSlot(0);

        m_mainMotor.setPosition(0);
 
        motorPosition = m_mainMotor.getPosition();
        deviceVoltage = m_mainMotor.getMotorVoltage();
        deviceCurrent = m_mainMotor.getSupplyCurrent();
        deviceTemp  = m_mainMotor.getDeviceTemp();
        torqueCurrent = m_mainMotor.getTorqueCurrent();

        // Tells encoders how often to update values
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            motorPosition,
            deviceVoltage,
            deviceCurrent,
            deviceTemp
        );

        m_mainMotor.optimizeBusUtilization();

    }

    /**
     *  Applies voltage to the motor
     * @param voltage the voltage to apply
     */
    @Override
    public void setMotorVoltage(double voltage) {
        m_mainMotor.setVoltage(voltage);
    }

    /** Stops the motor by either braking or coasting, specified in brakeMode() or coastMode() */
    @Override
    public void stopMotor() {
        m_mainMotor.stopMotor();
    }

    /** Sets the encoder's position to 0.
     * The tryUntilOk() method sends the signal at most 3 times to make sure it works.
    */
    @Override
    public void zeroEncoder() {
        PhoenixUtil.tryUntilOk(3, () -> m_mainMotor.setPosition(0));
    }

    /**
     * Sets the motor so that when it's stopped, it will enter brake mode,
     * which is when the motor actively tries to maintain its position
     * and strongly resists movement.
     * The tryUntilOk() method sends the signal at most 3 times to make sure it works.
     */
    @Override
    public void brakeMode() {
        PhoenixUtil.tryUntilOk(3, () -> m_mainMotor.setNeutralMode(NeutralModeValue.Brake));
    }

    /**
     * Sets the motor so that when it's stopped, it will enter coast mode,
     * which is when the motor stops but does NOT actively try to maintain its position,
     * nor does it try to resist movement,
     * allowing it to be moved/turned even when stopped.
     * The tryUntilOk() method sends the signal at most 3 times to make sure it works.
     */
    @Override
    public void coastMode() {
        PhoenixUtil.tryUntilOk(3, () -> m_mainMotor.setNeutralMode(NeutralModeValue.Coast));
    }

    /**
     * Gets the motor's current position in Meters
     */
    @Override
    public Distance getPosition() {
        return Meters.of(motorPosition.getValueAsDouble() * HopperConstants.UNIT_CONVERSION_FACTOR);
    }

     /**
     * Gets the motor's current position in meters, 
     * then adds the gap to the intake 
     * to get the position relative to the intake's zero point
     */
    @Override
    public Distance getPositionIntakeZero() {
        return getPosition().plus(HopperConstants.STARTING_GAP_TO_INTAKE);
    }

    /**
     * Sets the setpoint for PID to go to.
     * The tryUntilOk() method sends the signal at most 3 times to make sure it works.
     */
    @Override
    public void setSetpoint(Supplier<Distance> setpoint) {
        Distance setpointNew = Meters.of(MathUtil.clamp(setpoint.get().in(Meters)/HopperConstants.UNIT_CONVERSION_FACTOR, 0, HopperConstants.HOPPER_MAX_EXTENSION.in(Meters)/HopperConstants.UNIT_CONVERSION_FACTOR));
        PhoenixUtil.tryUntilOk(3, 
        () -> m_mainMotor.setControl(m_request.withPosition(setpointNew.in(Meters))
        .withSlot(0)));
        motorSetpoint = setpoint.get();

    }

    /** Gets the currently assigned setpoint */
    @Override
    public Distance getSetpoint() {
        return motorSetpoint;
    }

    @Override
    public void updateInputs(HopperInputs inputs) {
        inputs.isMotorConnected = BaseStatusSignal.refreshAll(
            motorPosition,
            deviceVoltage, 
            deviceCurrent, 
            deviceTemp
        ).isOK();
        inputs.appliedVoltage = deviceVoltage.getValue();
        inputs.appliedCurrent = deviceCurrent.getValue();
        inputs.torqueCurrent = torqueCurrent.getValue();
        inputs.motorTemp = deviceTemp.getValueAsDouble();
        inputs.motorPosition = getPosition();
        inputs.motorPositionIntakeZero = inputs.motorPosition.plus(HopperConstants.STARTING_GAP_TO_INTAKE);
        inputs.setpoint = motorSetpoint;
    }

}