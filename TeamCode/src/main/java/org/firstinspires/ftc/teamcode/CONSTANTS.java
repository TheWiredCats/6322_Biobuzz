package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

public final class CONSTANTS {
    public static final class MOTOR_CONFIGS{
        final String NAME;
        final DcMotor.RunMode RUN_MODE;
        final DcMotor.ZeroPowerBehavior BRAKE_MODE;
        final DcMotor.Direction DIRECTION;

        public MOTOR_CONFIGS(String name, DcMotor.RunMode runMode, DcMotor.ZeroPowerBehavior brakeMode, DcMotor.Direction direction) {
            NAME = name;
            RUN_MODE = runMode;
            BRAKE_MODE = brakeMode;
            DIRECTION = direction;
        }
    }
    public static final String PINPOINT = "pinpoint";
    public static final String LIMELIGHT = "limelight";
    public static final String HUSKY_LENS = "huskylens";

    private CONSTANTS(){
        //so u don't accidentally make an instance of it and only call it as needed
    }

    //All motors should be in order FL, BL, FR, BR, Intake, Transfer
    private static final String[] MOTORS = {"FL", "BL", "FR", "BR", "intake", "transfer"};
    private static final List<DcMotor.RunMode> RUN_TYPES = List.of(
    /* FL */DcMotor.RunMode.RUN_WITHOUT_ENCODER,
    /* BL */DcMotor.RunMode.RUN_WITHOUT_ENCODER,
    /* FR */DcMotor.RunMode.RUN_WITHOUT_ENCODER,
    /* BR */DcMotor.RunMode.RUN_WITHOUT_ENCODER,
/* Intake */DcMotor.RunMode.RUN_WITHOUT_ENCODER,
/*Transfer*/DcMotor.RunMode.RUN_WITHOUT_ENCODER
    );
    private static final List<DcMotor.ZeroPowerBehavior> BRAKE_MODES = List.of(
    /* FL */DcMotor.ZeroPowerBehavior.BRAKE,
    /* BL */DcMotor.ZeroPowerBehavior.BRAKE,
    /* FR */DcMotor.ZeroPowerBehavior.BRAKE,
    /* BR */DcMotor.ZeroPowerBehavior.BRAKE,
/* Intake */DcMotor.ZeroPowerBehavior.FLOAT,
/*Transfer*/DcMotor.ZeroPowerBehavior.FLOAT
    );
    private static final List<DcMotor.Direction> DIRECTIONS = List.of(
    /* FL */DcMotor.Direction.FORWARD,
    /* BL */DcMotor.Direction.REVERSE,
    /* FR */DcMotor.Direction.FORWARD,
    /* BR */DcMotor.Direction.FORWARD,
/* Intake */DcMotor.Direction.FORWARD,
/*Transfer*/DcMotor.Direction.FORWARD
    );
    public static final List<MOTOR_CONFIGS> MOTOR_CONFIG = new ArrayList<>();
    static {
        for (int i = 0; i < 6; i++) {
            MOTOR_CONFIG.add(new MOTOR_CONFIGS(MOTORS[i],
                    RUN_TYPES.get(i),
                    BRAKE_MODES.get(i),
                    DIRECTIONS.get(i))
            );
        }
    }
    public static final DistanceUnit DISTANCE = DistanceUnit.INCH;
    public static final AngleUnit ANGLE = AngleUnit.DEGREES;

    //used in case we switch to Radians
    //private final double pi = Math.PI;

    //replace with the position of this year's apriltags in inches
    //except for the first one keep this one empty   V
    public static final double[][] APRIL_TAG_POSITIONS = {{0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {0, 0, -1},
            {12, 0, 180},
            {12 + ((5.5 * Math.sqrt(3)) / 2), 2.75, 45},
            {12 + ((5.5 * Math.sqrt(3)) / 2), -2.75, 135}
    };
    public static final double CAMERA_X_OFFSET = 5;
    public static final double CAMERA_Y_OFFSET = 6;
    public static final double CAMERA_HEIGHT = 10.375;
    //replace with the height of the center of this year's apriltags
    public static final double APRIL_TAG_HEIGHT = 18.3125 - CAMERA_HEIGHT;

}
