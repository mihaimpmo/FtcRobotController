package org.firstinspires.ftc.teamcode.Pedro;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.FollowerConstants;

@Configurable
public class SwerveFollowerConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            // Robot physical properties
            .mass(10.0) // kg - adjust to actual robot mass

            // Zero-power deceleration (negative, in/s^2) - symmetric for swerve
            .forwardZeroPowerAcceleration(-30.0)
            .lateralZeroPowerAcceleration(-30.0)

            // Translational PIDF - corrects lateral drift from path
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.005, 0))

            // Heading PIDF - maintains robot heading
            .headingPIDFCoefficients(new PIDFCoefficients(1.0, 0, 0.05, 0))
            .headingPIDFSwitch(Math.toRadians(10))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.0, 0, 0.1, 0))

            // Drive PIDF - controls speed along path
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.0001, 0.6, 0.01))
            .drivePIDFSwitch(15)
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.015, 0, 0.00005, 0.6, 0.008))

            // Centripetal correction for curves
            .centripetalScaling(0.0005)

            // Hold point scaling
            .holdPointTranslationalScaling(0.45)
            .holdPointHeadingScaling(0.35);
}
