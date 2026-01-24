package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Launcher.LauncherConstants.kLauncher;

public class LauncherSim implements LauncherIO {

    private FlywheelSim flywheelSim;
    private PIDController controller;

    private Mechanism2d mech2D;
    private MechanismRoot2d root;

    private MechanismLigament2d stand;
    private MechanismLigament2d flyWheel;

    private boolean isRunning;

    private DCMotor dcMotor = DCMotor.getFalcon500Foc(1);
    private double joulesPerKiloMetersSquared = 1.0;
    private double gearing = 1.0;

    public LauncherSim() {
        flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(
                                            dcMotor,
                                            joulesPerKiloMetersSquared,
                                            gearing
                                        ),
                                        dcMotor,
                                        0.001
                                    );

        flywheelSim.update(0.01);

        controller = new PIDController(kLauncher.kP, kLauncher.kI, kLauncher.kD);

        mech2D = new Mechanism2d(5.0,5.0);

        root = mech2D.getRoot("Bot base", 2, 0);

        stand = root.append(new MechanismLigament2d(
            "Stand",
            0.7,
            90
            )
        );

        flyWheel = stand.append(new MechanismLigament2d(
            "Fly wheel",
            0.2,
            90
            )
        );

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
        
        inputs.launcherConnected = true;
        inputs.hoodConnected = false;

        inputs.temperatureLauncher = 0.0;
        inputs.temperatureHood = 0.0;

        inputs.volatgeLauncher = Volts.of(voltage);
        inputs.voltageHood = Volts.of(0.0);
        inputs.currentLauncher = Current.ofBaseUnits(current, Amps);
        inputs.currentHood = Current.ofBaseUnits(0.0, Amps);

        inputs.speedLauncherRadians = flywheelSim.getAngularVelocity();
        inputs.speedHoodRadians = RadiansPerSecond.of(0.0);

        inputs.targetHoodPosition = Degrees.of(0.0);
        inputs.hoodPosition = Degrees.of(0.0);

        SmartDashboard.putData("Mech2d", mech2D);
    }
}