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

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.List;

/**<!DOCTYPE html>
 * <html>
 * <body>
 * <h1><b>Robot Super Class</b></h1>
 * <h4><strong><i>Save the trouble of having to repeat code in every OpMode</i></strong></h4>
 * <p>V4.0 Added ender to TeleOp/Auto, and added comments</p>
 * </body>
 * </html>
 */
@SuppressWarnings("unused")
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
                Constants.MOTOR_CONFIGS motorData = Constants.MOTOR_CONFIG[i];
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
                Constants.MOTOR_CONFIGS motorData = Constants.MOTOR_CONFIG[i];
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
            servo.scaleRange(0,Constants.SERVO_MAX);
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
            limelight.updateRobotOrientation(this.pinpoint.getHeading(Constants.unit.AU));
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
    volatile List<DcMotor> drivingMotors;

    //List Of Utility Motors
    volatile List<DcMotor> utilMotors;

    //Servo
    volatile Servo servo;

    //Pinpoint Controller
    volatile GoBildaPinpointDriver pinpoint;

    //Limelight Camera
    volatile Limelight3A limelight;

    //HuskyLens Camera
    volatile HuskyLens huskyLens;

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
    private Robot(LinearOpMode ll, Team team){
        //LinearOpMode is a form of OpMode so it can be used as such
        this.op=ll;

        //make a new instance of each of the subclasses
        this.motorSub=new Motors(op);
        this.cameraSub=new Cameras(op);
        this.pinpointSub=new Pinpoint(op);

        //Set the team, the mode to Auto, and make sure LinearOpMode is set
        this.team = team;
        this.currentMode = OPMode.AUTO;
        this.ll = ll;

        //Set up the calc, Auto Driving Station
        this.calc = RobotUtil.initialize(this);
        this.ds = Driving_Systems.getInstance(team, this);

        //Set up all the Hardware with a simple method
        setUpHardware();
    }

    //catcher in case starting with teleop
    private Robot(OpMode op){
        //Auto only things should be set to null
        this.ds=null;
        this.ll=null;
        this.team=null;

        //Set the OpMode
        this.op = op;

        //Make the SubClasses Instances
        this.motorSub=new Motors(op);
        this.cameraSub=new Cameras(op);
        this.pinpointSub=new Pinpoint(op);

        //Set the current Mode to TeleOP
        this.currentMode = OPMode.TELEOP;

        //Set Up the calc
        this.calc = RobotUtil.initialize(this);

        //Setup All the Hardware Components
        setUpHardware();
    }

    //Use At the Start of Auto as a sort of getter that also does some helpful logic
    public static Robot startAuto(Team team, LinearOpMode ll){
        //if an instance already exists no need to make a new one
        if(instance!=null)return instance;
        //otherwise you have to make a new one and then return it
        return instance = new Robot(ll, team);
    }
    //Use At the Start of TeleOp as a sort of getter that also does a bunch of helpful logic
    //using the OpMode blackBoard
    public static Robot startTeleOp(OpMode op){
        //if there's no instance and the blackboard is empty make a new Tele Op instance
        if(instance==null&&OpMode.blackboard.isEmpty()){
            instance = new Robot(op);
            return instance;
            //if it isn't empty then make the instance that and also get the position on the field
        }else if(!OpMode.blackboard.isEmpty()){
            instance = ((Robot) OpMode.blackboard.get("robot_instance"));
            //If the gotten instance is null somehow it will return a Major Error
            if(instance==null)throw new MonkeyBusiness("OpMode error in startTele");
            instance.pinpoint.setPosition((Pose2D) OpMode.blackboard.get("position"));
        }
        //If there is an instance then change it from auto mode to TeleOp mode
        instance.op = op;
        instance.ll = null;
        instance.team = null;
        instance.ds = null;
        instance.currentMode=OPMode.TELEOP;
        //After it's been updated return it
        return instance;
    }
    //Sets Up all the hardware using their respective subclasses
    private void setUpHardware(){
        this.drivingMotors = motorSub.setupDrivingMotors();
        this.utilMotors = motorSub.setupMotors();
        this.servo = motorSub.setupServo();
        this.pinpoint = pinpointSub.setUpPinpoint();
        this.limelight = cameraSub.setupLimeLight(this.pinpoint);
        this.huskyLens = cameraSub.setupHuskyLens();
    }
    //At the End of Auto add the current instance to the blackboard aswell as the current position
    public void endAuto(){
        OpMode.blackboard.put("robot_instance", this);
        OpMode.blackboard.put("position", this.pinpoint.getPosition());
    }
    //At the End of TeleOp clear Anything on the blackboard
    public void endTeleOp(){
        OpMode.blackboard.clear();
    }

    //public Get Instance

    public static Robot getInstance()throws MonkeyBusiness{
        //If there isn't an instance to return then it will throw a Major Error
        if(instance==null) {
            throw new MonkeyBusiness("No Robot to get in getInstance.");
        }else{
            //otherwise it wil return the Instance
            return instance;
        }
    }



    //Actual Getters
    public List<DcMotor> getDrivingMotors(){return this.drivingMotors;}

    public List<DcMotor> getUtilMotors(){return this.utilMotors;}
    public GoBildaPinpointDriver getPinpoint(){return this.pinpoint;}
    public Limelight3A getLimelight(){return this.limelight;}
    public HuskyLens getHuskyLens(){return this.huskyLens;}
    public Team getTeam(){return this.team;}
    public Driving_Systems getDSInstance(){return this.ds;}
    public LinearOpMode getLL(){return this.ll;}
    public OpMode getOP(){return this.op;}
    public RobotUtil getCalc(){return this.calc;}
    public Servo getServo(){return this.servo;}
    //Only really need this for inside the Package
    OPMode getMode(){return this.currentMode;}
    //updates stuff on the limelight as well as returning the latest result from the limelight
    public synchronized LLResult updateLimelight(){return cameraSub.updateLimelight();}
}