package RobotUtil;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

/**<!DOCTYPE html>
 * <html>
 * <body>
 * <h1><b>Robot Super Class</b></h1>
 * <h4><strong><i>Save the trouble of having to repeat code in every OpMode</i></strong></h4>
 * <p>V3, Added Telemetry throughout building for testing, also fixed major problem</p>
 * </body>
 * </html>
 */
public class Robot {
    enum OPMode {
        AUTO,
        TELEOP
    }
    //Sub Classes Logic
    static final class Motors{
        private final OpMode op;
        private Motors(OpMode op) {
            this.op=op;
        }

        List<DcMotor> setupMotors(){
            List<DcMotor> motors = new ArrayList<>();
            for(int i = 4; i < 7; i++){
                Constants.MOTOR_CONFIGS motorData = Constants.MOTOR_CONFIG.get(i);
                if(motors.size()<=i-4)motors.add(i-4, this.op.hardwareMap.dcMotor.get(motorData.NAME));
                else motors.set(i-4, this.op.hardwareMap.dcMotor.get(motorData.NAME));
                motors.get(i-4).setMode(motorData.RUN_MODE);
                motors.get(i-4).setZeroPowerBehavior(motorData.BRAKE_MODE);
                motors.get(i-4).setDirection(motorData.DIRECTION);
            }
            return motors;
        }

        List<DcMotor> setupDrivingMotors() {
            List<DcMotor> motors = new ArrayList<>();
            for (int i = 0; i < 4; i++) {
                Constants.MOTOR_CONFIGS motorData = Constants.MOTOR_CONFIG.get(i);
                if (motors.size() <= i) motors.add(i, this.op.hardwareMap.dcMotor.get(motorData.NAME));
                else motors.set(i, this.op.hardwareMap.dcMotor.get(motorData.NAME));
                motors.get(i).setMode(motorData.RUN_MODE);
                motors.get(i).setZeroPowerBehavior(motorData.BRAKE_MODE);
                motors.get(i).setDirection(motorData.DIRECTION);
            }
            return motors;
        }
        Servo setupServo(){
            Servo servo = this.op.hardwareMap.servo.get(Constants.SERVO_NAME);
            servo.setDirection(Servo.Direction.FORWARD);
            return servo;
        }
    }
    static final class Cameras{
        private final OpMode op;
        private Limelight3A limelight;
        private GoBildaPinpointDriver pinpoint;
        private Cameras(OpMode op){
            this.op = op;
        }

        HuskyLens setupHuskyLens(){
            HuskyLens huskyLens = this.op.hardwareMap.get(HuskyLens.class, Constants.HUSKY_LENS);
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
            return huskyLens;
        }

        Limelight3A setupLimeLight(GoBildaPinpointDriver pinpoint){
            Limelight3A limelight = this.op.hardwareMap.get(Limelight3A.class, Constants.LIMELIGHT);
            limelight.pipelineSwitch(0);
            limelight.start();
            List<LLFieldMap.Fiducial> map = new ArrayList<>();
            for(int i = 30; i <= 45;i++){
                map.add(new LLFieldMap.Fiducial(i, 82.55, "apriltag3_36h11_classic", TagData.transforms.get(i-30), true));
            }
            limelight.uploadFieldmap(new LLFieldMap(map, "ftc"), null);
            limelight.updateRobotOrientation(pinpoint.getHeading(Constants.unit.AU));
            this.pinpoint=pinpoint;
            this.limelight=limelight;
            return limelight;
        }

        public synchronized LLResult updateLimelight() {
            if(this.limelight==null||this.pinpoint==null)throw new MonkeyBusiness("Please Setup Limelight Before Calling This Method");
            limelight.updateRobotOrientation(this.pinpoint.getHeading(AngleUnit.DEGREES));
            return this.limelight.getLatestResult();
        }
    }
    static final class Pinpoint{
        private final OpMode op;
        private Pinpoint(OpMode op){this.op=op;}

        /**
         * Sets up Pinpoint and gives it's default data
         * @return The Complete Pinpoint
         */
        GoBildaPinpointDriver setUpPinpoint(){
            GoBildaPinpointDriver pinpoint = this.op.hardwareMap.get(GoBildaPinpointDriver.class, Constants.PINPOINT);
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                          GoBildaPinpointDriver.EncoderDirection.REVERSED);
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);
            pinpoint.setBulkReadScope(GoBildaPinpointDriver.Register.X_POSITION, GoBildaPinpointDriver.Register.X_VELOCITY,
                                      GoBildaPinpointDriver.Register.Y_POSITION, GoBildaPinpointDriver.Register.Y_VELOCITY,
                                      GoBildaPinpointDriver.Register.H_ORIENTATION, GoBildaPinpointDriver.Register.H_VELOCITY);
            return pinpoint;
        }
    }
    private final Motors motorSub;
    private final Cameras cameraSub;
    private final Pinpoint pinpointSub;

    //List Of Driving Motors
    List<DcMotor> drivingMotors;

    //List Of Utility Motors
    List<DcMotor> utilMotors;

    //Servo
    Servo servo;

    //Pinpoint Controller
    GoBildaPinpointDriver pinpoint;

    //Limelight Camera
    Limelight3A limelight;

    //HuskyLens Camera
    HuskyLens huskyLens;

    //Team (Red or Blue)
    //Useful for most autos
    Team team;

    //Auto specific Op Mode
    //(useful for things like sleep())
    LinearOpMode ll;

    //Generic OpMode, for both Auto & TeleOp
    OpMode op;

    //Driving System were gonna use
    //(Dependent on team because the parking is slightly different)
    Driving_Systems ds;

    //Auto Or Tele Mode
    OPMode currentMode;

    final RobotUtil calc;

    //Helpful for singleton -ing
    private static Robot instance;

    //Initialize all the variables in the singleton instance
    protected Robot(LinearOpMode ll, Team team){
        this.op=ll;
        motorSub=new Motors(op);
        cameraSub=new Cameras(op);
        pinpointSub=new Pinpoint(op);
        this.team = team;
        this.currentMode = OPMode.AUTO;
        this.ll = ll;
        this.calc = RobotUtil.initialize(this);
        this.ds = Driving_Systems.getInstance(team, this);
        setUpHardware();
    }

    //catcher in case starting with teleop
    protected Robot(OpMode op){
        //Auto only things
        this.ds=null;
        this.ll=null;
        this.team=null;

        this.op = op;
        motorSub=new Motors(op);
        cameraSub=new Cameras(op);
        pinpointSub=new Pinpoint(op);
        this.currentMode = OPMode.TELEOP;
        this.calc = RobotUtil.initialize(this);
        setUpHardware();
    }

    public static Robot startAuto(Team team, LinearOpMode ll)throws MonkeyBusiness{
        if(instance!=null)throw new MonkeyBusiness("Already started an auto", ll);
        instance = new Robot(ll, team);
        return instance;
    }

    public static Robot startTele(OpMode op){
        if(instance==null){
            instance = new Robot(op);
            return instance;
        }
        instance.op = op;
        instance.ll = null;
        instance.team = null;
        instance.ds = null;
        instance.currentMode=OPMode.TELEOP;
        return instance;
    }
    private void setUpHardware(){
        this.drivingMotors = motorSub.setupDrivingMotors();
        this.utilMotors = motorSub.setupMotors();
        this.servo = motorSub.setupServo();
        this.pinpoint = pinpointSub.setUpPinpoint();
        this.limelight = cameraSub.setupLimeLight(this.pinpoint);
        //this.huskyLens = cameraSub.setupHuskyLens();
        op.telemetry.addLine("DONE!");
        op.telemetry.update();
    }

    //public Get Instance
    /*
    public static Robot getInstance()throws MonkeyBusiness{
        if(instance==null) {
            throw new MonkeyBusiness("No Robot to get. Fix this");
        }else{
            return instance;
        }
    }
     */


    //Actual Getters
    public List<DcMotor> getDrivingMotors(){return this.drivingMotors;}

    public List<DcMotor> getUtilMotors(){return this.utilMotors;}
    public GoBildaPinpointDriver getPinpoint(){return this.pinpoint;}
    public Limelight3A getLimelight(){return this.limelight;}
    public HuskyLens getHuskyLens(){return this.huskyLens;}
    public Team getTeam(){return this.team;}
    public Driving_Systems getDSInstance(){return this.ds;}
    //public LinearOpMode getLL(){return this.ll;}
    public OpMode getOP(){return this.op;}
    //public OPMode getMode(){return this.currentMode;}
    public RobotUtil getCalc(){return this.calc;}
    public synchronized LLResult updateLimelight(){return cameraSub.updateLimelight();}
    //public Servo getServo(){return this.servo;}
}