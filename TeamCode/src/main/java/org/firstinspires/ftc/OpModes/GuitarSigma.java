package org.firstinspires.ftc.OpModes;

import androidx.core.math.MathUtils;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import RobotUtil.*;

@TeleOp(name = "Guitar Op")

public class GuitarSigma extends OpMode {

    private DcMotor Intake;
    private DcMotor Transfer;
    private DcMotor Shoot;
    private DcMotor fl;
    private DcMotor bl;
    private DcMotor fr;
    private DcMotor br;
    private Servo servo;
    private GoBildaPinpointDriver pinpoint;
    //private float frcHeading;
    private float speedMult;
    //private RobotUtil calc;
    @Override
    public void init() {
        /*
        Robot robot = Robot.startTele(this);
        calc = robot.getCalc();
        pinpoint = robot.getPinpoint();
        frcHeading = (float)pinpoint.getHeading(Constants.unit.AU);
        Intake = robot.getUtilMotors().get(0);
        Transfer = robot.getUtilMotors().get(1);
        Shoot = robot.getUtilMotors().get(2);
        //servo = robot.getServo();
         */
        fl = hardwareMap.dcMotor.get("fl");
        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("br");
        Intake = hardwareMap.dcMotor.get("intake");
        Transfer = hardwareMap.dcMotor.get("transfer");
        Shoot = hardwareMap.dcMotor.get("shoot");
        servo = hardwareMap.servo.get("servo");
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Shoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setBulkReadScope(GoBildaPinpointDriver.Register.X_VELOCITY, GoBildaPinpointDriver.Register.X_POSITION,
                                  GoBildaPinpointDriver.Register.Y_VELOCITY, GoBildaPinpointDriver.Register.Y_POSITION,
                                  GoBildaPinpointDriver.Register.H_VELOCITY, GoBildaPinpointDriver.Register.H_ORIENTATION);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);
        //frcHeading =(float)pinpoint.getHeading(Constants.unit.AU);
        speedMult = 0.5f + MathUtils.clamp(gamepad1.right_stick_x, -0.4f, 5f);
    }

    @Override
    public void loop() {
        //float temp = (float)pinpoint.getHeading(Constants.unit.AU.getUnnormalized());
        pinpoint.update();
        //frcHeading += gamepad1.back?-frcHeading: AngleUnit.normalizeDegrees((float) pinpoint.getHeading(Constants.unit.AU.getUnnormalized())-temp);
        int pow = gamepad2.left_stick_button?-1:1;
        float shootMult = 0.5f + MathUtils.clamp(gamepad2.right_stick_x, -0.4f, 0.4f);
        Intake.setPower(gamepad2.a?pow:0);
        Transfer.setPower(gamepad2.b?gamepad2.right_bumper||gamepad2.left_bumper?-1:1:0);
        Shoot.setPower(gamepad2.y?pow*shootMult:0);
        if(gamepad2.right_stick_x>0&&servo.getPosition()!=1){
            servo.setPosition(1);
        }else if(gamepad2.right_stick_x<0&&servo.getPosition()!=0){
            servo.setPosition(0);
        }


        speedMult = 0.5f + MathUtils.clamp(gamepad1.right_stick_x, -0.4f, 5f);
        //float roboYaw = AngleUnit.RADIANS.fromUnit(Constants.unit.AU, frcHeading);
        float y = - (gamepad1.a?(gamepad1.b?0:1):gamepad1.b?-1:0);
        float x = -1.1f* (gamepad1.x?(gamepad1.y?0:1):gamepad1.y?-1:0);
        float rx = -(gamepad1.dpad_down?(gamepad1.dpad_up?0:1):gamepad1.dpad_up?-1:0);
        //float x =(float) (lx * Math.cos(roboYaw) + ly * Math.sin(roboYaw));
        //float y =(float) (ly * Math.cos(roboYaw) - lx * Math.sin(roboYaw));

        float FL = (y + x + rx) * speedMult;
        float BL = (y - x + rx) * speedMult;
        float FR = (y - x - rx) * speedMult;
        float BR = (y + x - rx) * speedMult;
        float max = Math.max(Math.max(Math.abs(FL),Math.abs(BL)),Math.max(Math.abs(FR),Math.abs(BR)));
        if(max>1){
            FL/=max;
            BL/=max;
            FR/=max;
            BR/=max;
        }
        fl.setPower(FL);
        bl.setPower(BL);
        fr.setPower(FR);
        br.setPower(BR);
        //calc.setPowers(FL, BL, FR, BR);
    }
}
