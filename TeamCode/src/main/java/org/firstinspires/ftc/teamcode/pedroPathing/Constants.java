package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.247)
            .forwardZeroPowerAcceleration(-62.29810319738266) //forward zero power accel tuner -65.60243346038591, -60.21660154951493, -58.84116279449511, -70.47662315434071, -56.353695028176666
            .lateralZeroPowerAcceleration(-90.15549551388034) //lateral zero power accel tuner -88.07851685180407, -95.55018620025608, -82.66648718507551, -88.33451698329932, -96.14777034896672
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .translationalPIDFCoefficients(new PIDFCoefficients(.25,0,0.03,0))
            .headingPIDFCoefficients(new PIDFCoefficients(3,0,0,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0,0,1,0))
            .centripetalScaling(0.0001)
            ;



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.8, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)

                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useVoltageCompensation(true)
            .useBrakeModeInTeleOp(true)
            .xVelocity(60.90678653266486) // forward vel tuner //61.05235278512549, 60.941393274021905, 60.80657286155881 , 60.476244828832435, 61.25736891378568
            .yVelocity(50.16638568067175); //lateral vel tuner 51.29702350286048, 49.92107817882628, 50.47421096441314, 49.9237944835753, 49.215821273683574

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(7.45965)
            .strafePodX(0.19387)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


}
