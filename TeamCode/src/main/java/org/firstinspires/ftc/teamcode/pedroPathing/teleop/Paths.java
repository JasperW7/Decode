package org.firstinspires.ftc.teamcode.pedroPathing.teleop;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {

    public PathChain scorePreloadBlue;
    public PathChain toFirstChain;
    public PathChain grabFirst;
    public PathChain grabSecond;
    public PathChain grabThird;
    public PathChain scoreFirstChain;
    public PathChain toSecondChain;
    public PathChain grabFourth;
    public PathChain grabFifth;
    public PathChain grabSixth;
    public PathChain scoreSecondChain;
    public PathChain leave;

    public Paths(Follower follower) {
        scorePreloadBlue = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(33.400, 134.500), new Pose(48.000, 96.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(63.4))
                .build();

        toFirstChain = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, new Pose(42.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(63.4), Math.toRadians(180))
                .build();

        grabFirst = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, new Pose(36.500, 84.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        grabSecond = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, new Pose(31.500, 84.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        grabThird = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, new Pose(26.500, 84.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        scoreFirstChain = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, new Pose(48.000, 96.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        toSecondChain = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, new Pose(42.000, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        grabFourth = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, new Pose(36.500, 60.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        grabFifth = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, new Pose(31.500, 60.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        grabSixth = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, new Pose(26.500, 60.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        scoreSecondChain = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, new Pose(48.000, 96.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        leave = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose,new Pose(20,72))
                )
                .setTangentHeadingInterpolation()
                .build();
    }
}
