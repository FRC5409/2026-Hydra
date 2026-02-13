package frc.robot.subsystems.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

public class LauncherSim implements LauncherIO {
    private static final double JOULES_PER_KILO_METERS_SQUARED = 1.0;
    private static final double GEARINGLAUNCHER                        = 1.0;

    private static final double GEARINGHOOD                        = 1.0;

    private final FlywheelSim   flywheelSim;
    private final PIDController controllerLauncher;

    private final PIDController controllerHood;

    private final SingleJointedArmSim hoodSim;

    private final DCMotor motorLauncher = DCMotor.getFalcon500Foc(1);
    private final DCMotor motorHood = DCMotor.getFalcon500Foc(1);

    private Angle targetHoodAngle = Degrees.of(0.0);

    private final Mechanism2d mech;
    private final MechanismRoot2d root;
    private final MechanismLigament2d stand;
    private final MechanismLigament2d hood;


    private boolean isRunning;

    public LauncherSim() {
        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        motorLauncher,
                        JOULES_PER_KILO_METERS_SQUARED,
                        GEARINGLAUNCHER
                ),
                motorLauncher,
                0.02
        );

        hoodSim = new SingleJointedArmSim(
            motorHood,
            GEARINGHOOD,
            0.7,
            0.3,
            0.0,
            Units.degreesToRadians(30),
            true,
            15,
            Math.PI
        );

        flywheelSim.update(0.01);
        hoodSim.update(0.01);

        mech = new Mechanism2d(0.6, 10.0);
        root = mech.getRoot("Base", 0.3, 0.1);

        stand = root.append(
            new MechanismLigament2d(
                "Stand",
                 0.7,
                 90
            )
        );

        hood = stand.append(
            new MechanismLigament2d(
                "Hood",
                 0.5,
                 45
            )
        );

        controllerLauncher = new PIDController(LauncherConstants.Launcher.PID.getP(), LauncherConstants.Launcher.PID.getI(), LauncherConstants.Launcher.PID.getD());

        controllerHood = new PIDController(LauncherConstants.Hood.PID.getP(), LauncherConstants.Hood.PID.getI(), LauncherConstants.Hood.PID.getD());

        isRunning = true;
    }

    @Override
    public void runRPS(Supplier<AngularVelocity> velocity) {
        flywheelSim.setAngularVelocity(velocity.get().in(RadiansPerSecond));
    }

    @Override
    public void setHoodPos(Angle angle) {
        targetHoodAngle = angle;
        controllerHood.setSetpoint(angle.in(Radians));
    }

    @Override
    public void stopHood() {
        hoodSim.setInput(0);
        controllerHood.reset();
        isRunning = false;
    }

    @Override
    public void stopLauncher() {
        // flywheelSim.setInputVoltage(0.0);
        flywheelSim.setAngularVelocity(0);
        controllerLauncher.reset();
        isRunning = false;
    }

    @Override
    public void updateInputs(LauncherInputs inputs) {
        double voltageLauncher = 0;
        double currentLauncher = 0;

        double voltageHood = 0;
        double currentHood = 0;

        if (isRunning) {
            voltageLauncher = MathUtil.clamp(
                    controllerLauncher.calculate(flywheelSim.getAngularVelocityRPM()),
                    12,
                    -12
            );

            voltageHood = MathUtil.clamp(
                    controllerHood.calculate(hoodSim.getAngleRads()),
                    12,
                    -12
            );

            currentLauncher = flywheelSim.getCurrentDrawAmps();

            currentHood = hoodSim.getCurrentDrawAmps();
        }

        inputs.isLauncherConnected = true;
        inputs.isHoodConnected = true;

        inputs.temperatureLauncher = 0.0;
        inputs.temperatureHood = 0.0;

        inputs.launcherVoltage = Volts.of(voltageLauncher);
        inputs.hoodVoltage = Volts.of(voltageHood);
        inputs.launcherCurrent = Current.ofBaseUnits(currentLauncher, Amps);
        inputs.hoodCurrent = Current.ofBaseUnits(currentHood, Amps);

        inputs.launcherSpeedRadians = flywheelSim.getAngularVelocity();
        // inputs.hoodSpeedRadians = Radians.of(hoodSim.getVelocityRadPerSec());

        inputs.targetHoodPosition = targetHoodAngle;
        inputs.hoodPosition = Degrees.of(hoodSim.getAngleRads());

        SmartDashboard.putNumber("Input volatge", flywheelSim.getInputVoltage());
        SmartDashboard.putNumber("Angular velocity RPM", flywheelSim.getAngularVelocityRadPerSec());
    }
}