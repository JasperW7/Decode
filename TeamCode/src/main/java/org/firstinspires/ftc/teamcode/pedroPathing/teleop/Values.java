package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import java.util.Map;
import java.util.TreeMap;
@Config
public final class Values {


    public static final class flywheelConstants {
        public static final PIDFController flywheelPIDF = new PIDFController(0, 0, 0, 0);
        public static double fP = 0.01;
        public static double fI = 0;
        public static double fD = 0.00005;
        public static double fK = 0.000445;
        public static double fV  = 0;
        public static double fA = 0;

        public static double flywheelVelocity=1800;
    }

    public static final class intakeConstants {
        public static final PIDFController intakePIDF = new PIDFController(0, 0, 0, 0);
        public static double iP = 0.0005;
        public static double iI = 0;
        public static double iD = 0.000015;
        public static double iK = 0.0004;
        public static double iV = 0;
        public static double iA = 0;

        public static double intakeVelocity = 500;
    }


    public static final class spindexerConstants {
        public static final ProfiledPIDController spindexerPIDF = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0,0));
        public static double sP = -0.00026;
        public static double sI = -0.08;
        public static double sD = -0.00003;
        public static double sK = 0;
        public static double sV =999999999;
        public static double sA = 0;
        public static double spindexerPosition=0;
        public static double spindexerStart = 0, spindexerGreen=2100+relativeSpindexOverride, spindexerPurple1=5000+relativeSpindexOverride, spindexerPurple2=7700+relativeSpindexOverride, spindexerGreenTransfer = 6200+relativeSpindexOverride,spindexerPurpleTransfer1 = 800+relativeSpindexOverride, spindexerPurpleTransfer3 = 3600+relativeSpindexOverride; //forward -> 800, 3600, 6200; transfer -> 2100, 4950, 7600
        public static double[]indexer = new double[] {spindexerGreen,spindexerPurple1,spindexerPurple2,spindexerGreenTransfer,spindexerPurpleTransfer1,spindexerPurpleTransfer3};
        public static int index = 0;
    }


    public static final class turretConstants {
        public static final ProfiledPIDController turretPIDF = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0,0));
        public static double tP = 0.007;
        public static double tI = 0.12;
        public static double tD = 0.0003;
        public static double tK = 0;
        public static double tV = 0;
        public static double tA = 0;
        public static double turretPosition=0;

        public static double turretMin = -1060, turretStart = 0,turretMax = 1080;
    }


    public enum Mode {
        INTAKING,
        OUTTAKING,
        ENDGAME
    }




    public static Mode mode = Mode.INTAKING;


    //TODO:UPDATE
    public static final TreeMap<Double, Integer> lerpTable = new TreeMap<>(
            // distance, target velocity
            Map.ofEntries(
                    Map.entry(39.95,1520),
                    Map.entry(53.3,1550),
                    Map.entry(61.1,1640),
                    Map.entry(70.2, 1650),
                    Map.entry(90.2,1820),
                    Map.entry(112.1,1970)
            )
    );
    public enum Motif {
        PPG,
        PGP,
        GPP,
        NONE
    }
    static Motif motif = Motif.NONE;
    public static String team="blue";
    public static final double transferBeltStart = 1, transferBeltStop = 0.5, transferBeltMid = 0.8;
    public static double transferDisengage=0.45, transferKick=0.9;
    public static double transferEngage=0.6;


    public static int greenCount=0;
    public static int purpleCount =0;
    public static boolean purpleBallProcessed = false;
    public static boolean greenBallProcessed = false;
    public static boolean waitingOnSpindex = false;

    public static boolean reversingIntake = false;
    public static Methods.DetectedColor lastDetectedColor = Methods.DetectedColor.UNKNOWN;
    public static Methods.DetectedColor lastProcessedColor = Methods.DetectedColor.UNKNOWN;
    public static Methods.DetectedColor reversingColor = Methods.DetectedColor.UNKNOWN;

    public static int lastColorFrames=0;


    public static boolean drivers=false;
    public static double relativeSpindexOverride = 0;

    public static boolean init = true;
    public static boolean turretOverride = false;
    public static int engaged = 0;
    public static boolean endgame = false;
    public static void reset(){
        endgame=false;
        engaged=0;
        init=true;
        drivers=false;
        lastColorFrames=0;
        reversingColor=Methods.DetectedColor.UNKNOWN;
        lastDetectedColor=Methods.DetectedColor.UNKNOWN;
        reversingIntake=false;
        waitingOnSpindex=false;
        greenCount=0;
        purpleCount=0;
        greenBallProcessed=false;
        purpleBallProcessed=false;
        team="blue";
        motif=Motif.NONE;
        mode=Mode.INTAKING;
        turretConstants.turretPosition=0;
        spindexerConstants.spindexerPosition=0;
        flywheelConstants.flywheelVelocity=1800;
    }

}
