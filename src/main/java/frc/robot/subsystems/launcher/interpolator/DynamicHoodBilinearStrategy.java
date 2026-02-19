package frc.robot.subsystems.launcher.interpolator;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;

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
    private static final Per<AngleUnit, AngularVelocityUnit> ALPHA = Radians.per(RadiansPerSecond).ofNative(1);

    public Angle computeHoodAdjustment(AngularVelocity targetVelocity, AngularVelocity realVelocity, Angle hoodAngle) {
        return (Angle)ALPHA.timesDivisor(targetVelocity.minus(realVelocity)).times(Math.cos(hoodAngle.in(Radians)));
    }

    @Override
    public LaunchConfig interpolate(Distance displacement) {
        var params = super.interpolate(displacement);

        // update hood before returning interpolation
        this.launcher.setHoodPos(computeHoodAdjustment(
                params.speed(), this.launcher.getVelocity(), this.launcher.getHoodPos()));

        return params;
    }

    @Override
    public String getName() {
        return "Bilinear w/ Dynamic Hood";
    }
}
