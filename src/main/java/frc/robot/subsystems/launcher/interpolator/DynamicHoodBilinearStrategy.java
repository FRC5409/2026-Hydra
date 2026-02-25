package frc.robot.subsystems.launcher.interpolator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

/**
 * Interpolates a {@link LaunchConfig} (angular velocity and shoot angle) given a displacement to shoot the fuel.
 * <p>
 * The angular velocity and launch angle are separated into 2 different functions, only correlated by the common
 * independent variable (distance).
 *
 * @author Logan Dhillon, FRC 5409 Chargers
 * @apiNote This strategy internally uses a {@link BilinearStrategy}; the test points in the {@link BilinearStrategy}
 * will be used here.
 */
public class DynamicHoodBilinearStrategy extends BilinearStrategy {
    private LaunchConfig lastConfig;

    /**
     * Tuned value that affects the correction rate of the hood as per the velocity error.
     */
    private static final float ALPHA = 0.005f;

    /**
     * Computes the new hood value to correct the error of theoretical velocity and real velocity
     *
     * @param targetVelocity target/theoretical velocity
     * @param realVelocity   actual velocity of the motor
     * @param hoodAngle      measured hood angle
     *
     * @return new hood angle
     */
    public Angle computeHoodAdjustment(
            AngularVelocity targetVelocity, AngularVelocity realVelocity, Angle hoodAngle) {
        return Radians.of(ALPHA
                          * targetVelocity.minus(realVelocity).in(RadiansPerSecond) // velocity error
                          * Math.cos(hoodAngle.times(2).in(Radians))); // hood adjustment
    }

    @Override
    public LaunchConfig interpolate(Distance displacement) {
        var params = super.interpolate(displacement);
        lastConfig = params;
        return params;
    }

    @Override
    public void periodicActive() {
        if (lastConfig == null) return;

        // update hood before returning interpolation
        Angle err = computeHoodAdjustment(lastConfig.speed(), this.launcher.getVelocity(), this.launcher.getHoodAngle());
        Logger.recordOutput("Launcher/Interpolator/DynamicHoodAdjustment", err);
        CommandScheduler.getInstance().schedule(this.launcher.setHoodAngle(() -> err));
    }

    @Override
    public String getName() {
        return "Bilinear w/ Dynamic Hood";
    }
}
