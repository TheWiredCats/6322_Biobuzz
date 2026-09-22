package RobotUtil;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Team;

import java.util.List;

public class Robot {
    private final List<DcMotor> drivingMotors;
    private final List<DcMotor> utilMotors;
    private final RevBlinkinLedDriver led;
    private final GoBildaPinpointDriver pinpoint;
    private final Limelight3A limelight;
    private final HuskyLens huskyLens;
    private final Team team;
    private static Robot instance;
    Robot(OpMode op, Team team){
        this.drivingMotors = setupDrivingMotors(op);
        this.utilMotors = setupMotors(op);
        this.led = LEDSetUP(op);
        this.pinpoint = setUpPinpoint(op);
        this.limelight = setupLimeLight(op, this.pinpoint);
        this.huskyLens = setupHuskyLens(op);
        this.team = team;
    }
    public static Robot getInstance(OpMode op, Team team){
        instance = (instance==null)?new Robot(op, team):instance;
        return instance;
    }

    protected List<DcMotor> setupDrivingMotors(OpMode op){return null;}
    protected List<DcMotor> setupMotors(OpMode op){return null;}
    protected RevBlinkinLedDriver LEDSetUP(OpMode op){return null;}
    protected GoBildaPinpointDriver setUpPinpoint(OpMode op){return null;}
    protected Limelight3A setupLimeLight(OpMode op, GoBildaPinpointDriver pinpoint){
        return null;
    }
    protected HuskyLens setupHuskyLens(OpMode op){return null;}

    public List<DcMotor> getDrivingMotors(){return this.drivingMotors;}
    public List<DcMotor> getUtilMotors(){return this.utilMotors;}
    public RevBlinkinLedDriver getLed(){return this.led;}
    public GoBildaPinpointDriver getPinpoint(){return this.pinpoint;}
    public Limelight3A getLimelight(){return this.limelight;}
    public HuskyLens getHuskyLens(){return this.huskyLens;}
    public Team getTeam(){return this.team;}
}
