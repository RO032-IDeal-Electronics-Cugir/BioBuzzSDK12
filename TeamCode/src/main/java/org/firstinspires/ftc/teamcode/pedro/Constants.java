package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.frontRightName.set("fr");
        c.backLeftName.set("bl");
        c.backRightName.set("br");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });


    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(6.99261762964444);
        c.yPodOffset.set(-2.2805598401647855);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.22698672164171535);
                Controller secondaryTranslationalForward = Controller.proportional(0.08386552844970724);
                Controller primaryTranslationalLateral = Controller.proportional(0.401767073397227);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1484421982063573);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.015331354283587255));
                c.brake.set(Controller.proportionalFeedforward(0.013031651141049165));

                c.headingFeedback.set(Controller.proportional(3.560112358610397));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.039908871630194145, 0.008458317225422387));

                c.linearBrakeCoefficients.set(Matrix.diag(0.061386867516773164, 0.0444121008551278));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0021044643530402363, 0.00219021591384284));

                c.maxAchievableForwardVelocity.set(67.2502678541289);
                c.maxAchievableStrafeVelocity.set(50.84985523217834);
                c.naturalForwardDeceleration.set(38.44484423449467);
                c.naturalStrafeDeceleration.set(67.07973731400625);
            }
    );


    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }
}