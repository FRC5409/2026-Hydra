package frc.robot.subsystems.Launcher;

import org.littletonrobotics.junction.ConsoleSource.Simulator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants.kLauncher;

public class LauncherSim implements LauncherIO{
    
    private FlywheelSim flywheelSim;
    private PIDController controller;

    private Mechanism2d mech2D;
    private MechanismRoot2d root;

    private MechanismLigament2d stand;
    private MechanismLigament2d flyWheel;

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

        // mech2D = new Mechanism2d(0.6,5.0);

        // root = mech2D.getRoot("Bot base", 0, 0);

        // stand = root.append(new MechanismLigament2d("Stand", 0.7, 0));

        // flyWheel = root.append(new MechanismLigament2d("Fly wheel", 0.2, 0));
        
    }

    @Override
    public void launchFuel() {
        flywheelSim.setInputVoltage(6.0);
    }

    @Override
    public void stop() {
        flywheelSim.setInputVoltage(0.0);
        controller.reset();
    }

    @Override
    public void updateInputs(LauncherInputs inputs) {
        inputs.launcherConnected = true;
        inputs.hoodConnected = false;

        inputs.temperatureLauncher = 0.0;
        inputs.temperatureHood = 0.0;

        inputs.volatgeLauncher = flywheelSim.getInputVoltage();
        inputs.voltageHood = 0.0;
        inputs.currentLauncher = flywheelSim.getCurrentDrawAmps();
        inputs.currentHood = 0.0;

        inputs.speedLauncher = flywheelSim.getAngularVelocityRadPerSec();
        inputs.speedHood = 0.0;

        inputs.targetHoodPosition = 0.0;
        inputs.hoodPosition = 0.0;
    }
}
