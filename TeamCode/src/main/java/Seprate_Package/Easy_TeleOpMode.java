package Seprate_Package;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import RobotUtil.*;

@Disabled
public abstract class Easy_TeleOpMode extends OpMode implements TeleOp{

    private String currentMode;
    private final Robot robot = Robot.startTele(this);

    protected final List<DcMotor> drivingMotors = robot.getDrivingMotors();
    protected final List<DcMotor> utilMotors = robot.getUtilMotors();
    protected final RevBlinkinLedDriver LED = robot.getLed();
    protected final Limelight3A limelight = robot.getLimelight();
    protected final HuskyLens huskyLens = robot.getHuskyLens();
    protected final GoBildaPinpointDriver pinpoint = robot.getPinpoint();
    protected final RobotUtil calc =  robot.getCalc();

    @Override
    public final void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
    }

    protected final void updateStatusTelemetry() {
        telemetry.addData("Current Mode ", this.currentMode);
        telemetry.update();
    }

    @Override public final void init(){
        currentMode = "Starting Initialization.";
        initialize();
        updateStatusTelemetry();
    }

    @Override public final void init_loop(){
        if(!currentMode.equals("Initializing..."))currentMode = "Initializing...";
        preLoop();
        updateStatusTelemetry();
    }

    @Override public final void start(){
        currentMode = "Starting TeleOp.";
        onStart();
        updateStatusTelemetry();
    }

    @Override public final void loop(){
        if(!currentMode.equals("Running ..."))currentMode = "Running ...";
        TeleOpMode();
        updateStatusTelemetry();
    }

    @Override public final void stop(){
        currentMode = "Ending.";
        atEnd();
        updateStatusTelemetry();
    }
}
