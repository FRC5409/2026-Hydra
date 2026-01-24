package frc.robot.subsystems.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public class LauncherSim implements LauncherIO {
    private static final double JOULES_PER_KILO_METERS_SQUARED = 1.0;
    private static final double GEARING                        = 1.0;

    private final FlywheelSim   flywheelSim;
    private final PIDController controller;

    private final Mechanism2d     mech2D;
    private final MechanismRoot2d root;

    private final MechanismLigament2d stand;
    private final MechanismLigament2d flywheel;

    private final DCMotor motor = DCMotor.getFalcon500Foc(1);

    private boolean isRunning;

    public LauncherSim() {
        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        motor,
                        JOULES_PER_KILO_METERS_SQUARED,
                        GEARING
                ),
                motor,
                0.001
        );

        flywheelSim.update(0.01);
        controller = new PIDController(LauncherConstants.kP, LauncherConstants.kI, LauncherConstants.kD);
        mech2D = new Mechanism2d(5.0, 5.0);

        root = mech2D.getRoot("Bot base", 2, 0);

        stand = root.append(new MechanismLigament2d(
                "Stand",
                0.7,
                90
        ));
        flywheel = stand.append(new MechanismLigament2d(
                "Fly wheel",
                0.2,
                90
        ));

        SmartDashboard.putData("Mech2d", mech2D);
        isRunning = true;
    }

    @Override
    public void launchFuel() {
        flywheelSim.setInputVoltage(6.0);
    }

    @Override
    public void stop() {
        flywheelSim.setInputVoltage(0.0);
        controller.reset();
        isRunning = false;
    }

    @Override
    public void updateInputs(LauncherInputs inputs) {
        double voltage = 0;
        double current = 0;

        if (isRunning) {
            voltage = MathUtil.clamp(
                    controller.calculate(flywheelSim.getAngularVelocityRPM()),
                    12,
                    -12
            );

            current = flywheelSim.getCurrentDrawAmps();
        }

        inputs.isLauncherConnected = true;
        inputs.isHoodConnected = false;

        inputs.temperatureLauncher = 0.0;
        inputs.temperatureHood = 0.0;

        inputs.launcherVoltage = Volts.of(voltage);
        inputs.hoodVoltage = Volts.of(0.0);
        inputs.launcherCurrent = Current.ofBaseUnits(current, Amps);
        inputs.hoodCurrent = Current.ofBaseUnits(0.0, Amps);

        inputs.launcherSpeed = flywheelSim.getAngularVelocity();
        inputs.hoodSpeed = RadiansPerSecond.of(0.0);

        inputs.targetHoodPosition = Degrees.of(0.0);
        inputs.hoodPosition = Degrees.of(0.0);

        SmartDashboard.putData("Mech2d", mech2D);
    }
}