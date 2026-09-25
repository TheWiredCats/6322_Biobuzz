package RobotUtil;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

public final class CONSTANTS {
    public final static class tolerances{
        public final double DError;
        public final double DSpeed;
        public final double TError;
        public final double TSpeed;
        public tolerances(double DE, double DS, double TE, double TS){
            this.DError = DE;
            this.DSpeed = DS;
            this.TError = TE;
            this.TSpeed = TS;
        }
    }
    public static final tolerances tolerance = new tolerances(1, 1, 5, 5);
    public final static class tunerHolder{
        public final double KP;
        public final double KI;
        public final double KD;
        public tunerHolder(double kp, double ki, double kd){
            KP = kp;
            KI = ki;
            KD = kd;
        }
    }
    public static final class MOTOR_CONFIGS{
        final String NAME;
        final DcMotor.RunMode RUN_MODE;
        final DcMotor.ZeroPowerBehavior BRAKE_MODE;
        final DcMotor.Direction DIRECTION;

        public MOTOR_CONFIGS(String name, DcMotor.RunMode runMode,
                             DcMotor.ZeroPowerBehavior brakeMode, DcMotor.Direction direction) {
            NAME = name;
            RUN_MODE = runMode;
            BRAKE_MODE = brakeMode;
            DIRECTION = direction;
        }
    }

    public static final class LED_CONFIGS{
        final RevBlinkinLedDriver.BlinkinPattern NONE_SIGHTED;
        final RevBlinkinLedDriver.BlinkinPattern TARGET_SIGHTED;
        final RevBlinkinLedDriver.BlinkinPattern CLOSE;
        final RevBlinkinLedDriver.BlinkinPattern ON_POINT;

        public LED_CONFIGS(RevBlinkinLedDriver.BlinkinPattern none,
                           RevBlinkinLedDriver.BlinkinPattern targetFound,
                           RevBlinkinLedDriver.BlinkinPattern close,
                           RevBlinkinLedDriver.BlinkinPattern found) {
            NONE_SIGHTED = none;
            TARGET_SIGHTED = targetFound;
            CLOSE = close;
            ON_POINT = found;
        }
    }

    public static final double MAXIMUM = 1;
    public static final double MINIMUM = 0.25;

    //change the colors based on what colors u want for each setting
    public static final LED_CONFIGS ledConfig = new LED_CONFIGS(
            RevBlinkinLedDriver.BlinkinPattern.RED,
            RevBlinkinLedDriver.BlinkinPattern.YELLOW,
            RevBlinkinLedDriver.BlinkinPattern.GREEN,
            RevBlinkinLedDriver.BlinkinPattern.BLUE
    );
    public static final String LED = "led";
    public static final String PINPOINT = "pinpoint";
    public static final String LIMELIGHT = "limelight";
    public static final String HUSKY_LENS = "huskylens";

    private CONSTANTS(){
        //so u don't accidentally make an instance of it and only call it as needed
    }

    //All motors should be in order FL, BL, FR, BR, Intake, Transfer, Shoot
    private static final String[] MOTORS = {"FL", "BL", "FR", "BR", "intake", "transfer", "shoot"};
    private static final List<DcMotor.RunMode> RUN_TYPES = List.of(
    /* FL */DcMotor.RunMode.RUN_WITHOUT_ENCODER,
    /* BL */DcMotor.RunMode.RUN_WITHOUT_ENCODER,
    /* FR */DcMotor.RunMode.RUN_WITHOUT_ENCODER,
    /* BR */DcMotor.RunMode.RUN_WITHOUT_ENCODER,
/* Intake */DcMotor.RunMode.RUN_WITHOUT_ENCODER,
/*Transfer*/DcMotor.RunMode.RUN_WITHOUT_ENCODER,
/* Shoots */DcMotor.RunMode.RUN_WITHOUT_ENCODER
    );
    private static final List<DcMotor.ZeroPowerBehavior> BRAKE_MODES = List.of(
    /* FL */DcMotor.ZeroPowerBehavior.BRAKE,
    /* BL */DcMotor.ZeroPowerBehavior.BRAKE,
    /* FR */DcMotor.ZeroPowerBehavior.BRAKE,
    /* BR */DcMotor.ZeroPowerBehavior.BRAKE,
/* Intake */DcMotor.ZeroPowerBehavior.FLOAT,
/*Transfer*/DcMotor.ZeroPowerBehavior.FLOAT,
/* Shoots */DcMotor.ZeroPowerBehavior.FLOAT
    );
    private static final List<DcMotor.Direction> DIRECTIONS = List.of(
    /* FL */DcMotor.Direction.FORWARD,
    /* BL */DcMotor.Direction.REVERSE,
    /* FR */DcMotor.Direction.FORWARD,
    /* BR */DcMotor.Direction.FORWARD,
/* Intake */DcMotor.Direction.REVERSE,
/*Transfer*/DcMotor.Direction.FORWARD,
/* Shoots */DcMotor.Direction.FORWARD
    );
    public static final List<MOTOR_CONFIGS> MOTOR_CONFIG = new ArrayList<>();
    static {
        for (int i = 0; i < 7; i++) {
            MOTOR_CONFIG.add(i, new MOTOR_CONFIGS(MOTORS[i],
                    RUN_TYPES.get(i),
                    BRAKE_MODES.get(i),
                    DIRECTIONS.get(i))
            );
        }
    }
    public static final class Units{
        public final DistanceUnit DU;
        public final AngleUnit AU;
        public Units(DistanceUnit du, AngleUnit au){
            DU = du;
            AU = au;
        }
    }

    public static final tunerHolder driverConstants = new tunerHolder(0.25275,  -0.075, 0.155);
    public static final tunerHolder turningConstants = new tunerHolder(0.2, 0, 0);

    public static final Units unit = new Units(DistanceUnit.INCH, AngleUnit.DEGREES);

}
