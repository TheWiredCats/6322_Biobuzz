package RobotUtil;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

/**<!DOCTYPE html>
 * <html>
 * <body>
 * <h1><b>Robot Super Class</b></h1>
 * <h4><strong><i>Save the trouble of having to repeat code in every OpMode</i></strong></h4>
 * <p>Comment out stuff you don't have, keep the stuff you do, or even add some stuff we forgot!</p>
 * </body>
 * </html>
 */
public class Robot {

    //List Of Driving Motors
    private List<DcMotor> drivingMotors;

    //List Of Utility Motors
    private List<DcMotor> utilMotors;

    //Led Controller
    private RevBlinkinLedDriver led;

    //Pinpoint Controller
    private GoBildaPinpointDriver pinpoint;

    //Limelight Camera
    private Limelight3A limelight;

    //HuskyLens Camera
    private HuskyLens huskyLens;

    //Team (Red or Blue)
    //Useful for most autos
    private Team team;

    //Auto specific Op Mode
    //(useful for things like sleep())
    private LinearOpMode ll;

    //Generic OpMode, for both Auto & TeleOp
    private OpMode op;

    //Driving System were gonna use
    //(Dependent on team because the parking is slightly different)
    private Driving_Systems ds;

    //Auto Or Tele Mode
    private OPMode currentMode;

    private final RobotUtil calc;

    //Helpful for singleton -ing
    private static Robot instance;

    //Initialize all the variables in the singleton instance
    protected Robot(LinearOpMode ll, Team team){
        this.op=ll;
        this.team = team;
        this.currentMode = OPMode.AUTO;
        this.ll = ll;
        this.ds = Driving_Systems.getInstance(team, this);
        this.calc = RobotUtil.initialize(this);
        setUpHardware(op);
    }

    //catcher in case starting with teleop
    protected Robot(OpMode op){
        //Auto only things
        this.ds=null;
        this.ll=null;
        this.team=null;
        this.op = op;
        this.currentMode = OPMode.TELEOP;
        this.calc = RobotUtil.initialize(this);
        setUpHardware(op);
    }

    public static Robot startAuto(Team team, LinearOpMode ll)throws MonkeyBusiness{
        if(instance!=null)throw new MonkeyBusiness("Already started an auto asshole", ll);
        return instance = new Robot(ll, team);
    }

    public static Robot startTele(OpMode op){
        if(instance==null) return instance = new Robot(op);
        instance.op = op;
        instance.ll = null;
        instance.team = null;
        instance.ds = null;
        instance.currentMode=OPMode.TELEOP;
        return instance;
    }
    private void setUpHardware(OpMode op){
        this.drivingMotors = setupDrivingMotors(op);
        this.utilMotors = setupMotors(op);
        this.led = LEDSetUP(op);
        this.pinpoint = setUpPinpoint(op);
        this.limelight = setupLimeLight(op, this.pinpoint);
        this.huskyLens = setupHuskyLens(op);
    }

    //public Get Instance
    public static Robot getInstance()throws MonkeyBusiness{
    if(instance==null) {
        throw new MonkeyBusiness("No OpMode to get.  Fix this");
    }else{
        return instance;
    }}

    //Have the subclasses worry about this stuff

    protected List<DcMotor> setupDrivingMotors(OpMode op){return Motors.getMotors(op).setupDrivingMotors(op);}
    protected List<DcMotor> setupMotors(OpMode op){return Motors.getMotors(op).setupMotors(op);}
    protected RevBlinkinLedDriver LEDSetUP(OpMode op){return Led.getLed(op).LEDSetUP(op);}
    protected GoBildaPinpointDriver setUpPinpoint(OpMode op){return Pinpoint.getPinpoint(op).setUpPinpoint(op);}
    protected Limelight3A setupLimeLight(OpMode op, GoBildaPinpointDriver pinpoint){return Cameras.getCameras(op).setupLimeLight(op, pinpoint);}
    protected HuskyLens setupHuskyLens(OpMode op){return Cameras.getCameras(op).setupHuskyLens(op);}

    //Actual Getters
    public List<DcMotor> getDrivingMotors(){return this.drivingMotors;}

    public List<DcMotor> getUtilMotors(){return this.utilMotors;}
    public RevBlinkinLedDriver getLed(){return this.led;}
    public GoBildaPinpointDriver getPinpoint(){return this.pinpoint;}
    public Limelight3A getLimelight(){return this.limelight;}
    public HuskyLens getHuskyLens(){return this.huskyLens;}
    public Team getTeam(){return this.team;}
    public Driving_Systems getDSInstance(){return this.ds;}
    public LinearOpMode getLL(){return this.ll;}
    public OpMode getOP(){return this.op;}
    public OPMode getMode(){return this.currentMode;}
    public RobotUtil getCalc(){return this.calc;}
    public synchronized LLResult updateLimelight(){return null;}
}