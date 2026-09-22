package RobotUtil;

import com.bylazar.field.Line;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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

    //Driving Motors
    private final List<DcMotor> drivingMotors;

    //Utility Motors
    private final List<DcMotor> utilMotors;

    //Led Controller
    private final RevBlinkinLedDriver led;

    //Pinpoint Controller
    private final GoBildaPinpointDriver pinpoint;

    //Limelight Camera
    private final Limelight3A limelight;

    //HuskyLens Camera
    private final HuskyLens huskyLens;

    //Team (Red or Blue)
    //Useful for most autos
    private final Team team;
    private LinearOpMode ll;
    private OpMode op;
    private final Driving_Systems ds;

    //Helpful for singleton -ing
    private static Robot instance;

    //Initialize all the variables in the singleton instance
    protected Robot(OpMode op, Team team){
        this.drivingMotors = setupDrivingMotors(op);
        this.utilMotors = setupMotors(op);
        this.led = LEDSetUP(op);
        this.pinpoint = setUpPinpoint(op);
        this.limelight = setupLimeLight(op, this.pinpoint);
        this.huskyLens = setupHuskyLens(op);
        this.team = team;
        this.ds = Driving_Systems.getInstance(team);
    }
    protected Robot(LinearOpMode ll, Team team, Boolean nu_uh){
        this.drivingMotors = setupDrivingMotors(ll);
        this.utilMotors = setupMotors(ll);
        this.led = LEDSetUP(ll);
        this.pinpoint = setUpPinpoint(ll);
        this.limelight = setupLimeLight(ll, this.pinpoint);
        this.huskyLens = setupHuskyLens(ll);
        this.team = team;
        this.ll=ll;
        this.op=ll;
        this.ds = Driving_Systems.getInstance(team);
    }

    //public Get Instance
    public static Robot getInstance()throws MonkeyBusiness{if(instance==null) {
        throw new MonkeyBusiness();
    }else{
        return instance;
    }}
    public static Robot getInstance(OpMode op, Team team){
        instance = (instance==null)?new Robot(op, team):instance;
        return instance;
    }
    public static Robot getInstance(LinearOpMode ll, Team team, Boolean nu_uh){
        instance = (instance==null)?new Robot(ll, team, nu_uh):instance;
        return instance;
    }

    //Have the subclasses worry about this stuff

    protected List<DcMotor> setupDrivingMotors(OpMode op){return null;}
    protected List<DcMotor> setupMotors(OpMode op){return null;}
    protected RevBlinkinLedDriver LEDSetUP(OpMode op){return null;}
    protected GoBildaPinpointDriver setUpPinpoint(OpMode op){return null;}
    protected Limelight3A setupLimeLight(OpMode op, GoBildaPinpointDriver pinpoint){return null;}
    protected HuskyLens setupHuskyLens(OpMode op){return null;}

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
}
