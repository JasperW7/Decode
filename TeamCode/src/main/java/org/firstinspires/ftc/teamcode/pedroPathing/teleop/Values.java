package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.arcrobotics.ftclib.controller.PIDFController;
import java.util.Map;
import java.util.TreeMap;

public final class Values {


    public static final class flywheelConstants {
        public static final PIDFController flywheelPIDF = new PIDFController(0, 0, 0, 0);
        public static double fP = 0.1;
        public static double fI = 0;
        public static double fD = 0;
        public static double fK = 0.0045;

        public static double flywheelVelocity;
    }


    public static final class intakeConstants {
        public static final PIDFController intakePIDF = new PIDFController(0, 0, 0, 0);
        public static double iP = 0;
        public static double iI = 0;
        public static double iD = 0;
        public static double iK = 0;

        public static double intakeVelocity;
    }


    public static final class spindexerConstants {
        public static final PIDFController spindexerPIDF = new PIDFController(0, 0, 0, 0);
        public static double sP = 0;
        public static double sI = 0;
        public static double sD = 0;
        public static double sK = 0;
        public static double spindexerPosition;
    }


    public static final class turretConstants {
        public static final PIDFController turretPIDF = new PIDFController(0, 0, 0, 0);
        public static double tP = 0;
        public static double tI = 0;
        public static double tD = 0;
        public static double tK = 0;
        public static double turretPosition;
    }


    public enum Mode {
        INTAKING,
        OUTTAKING,
        ENDGAME
    }




    public static Mode mode = Mode.INTAKING;

    // Example mechanism positions


    public static final TreeMap<Double, Integer> lerpTable = new TreeMap<>(
            Map.ofEntries(
                    Map.entry(2.13, 2100),
                    Map.entry(2.54, 2000),
                    Map.entry(3.81, 2000),
                    Map.entry(4.5, 2000),
                    Map.entry(4.7, 2000)
            )
    );
    public static String motif;
    public static double slidingDoorOpen = 0.8, slidingDoorClose = 0.2;

    public static boolean intaking = false;

}
