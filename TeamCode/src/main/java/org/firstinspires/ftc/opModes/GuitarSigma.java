package org.firstinspires.ftc.opModes;

import androidx.core.math.MathUtils;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import robotUtil.*;

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
    private RobotUtil calc;
    //doin bit flag stuff
    byte servoData = 0;
    @Override
    public void init() {
        Robot robot = Robot.startTeleOp(this);
        calc = robot.getCalc();
        pinpoint = robot.getPinpoint();
        //frcHeading = (float)pinpoint.getHeading(Constants.unit.AU);
        Intake = robot.getUtilMotors().get(0);
        Transfer = robot.getUtilMotors().get(1);
        Shoot = robot.getUtilMotors().get(2);
        servo = robot.getServo();
        fl = robot.getDrivingMotors().get(0);
        bl = robot.getDrivingMotors().get(1);
        fr = robot.getDrivingMotors().get(2);
        br = robot.getDrivingMotors().get(3);
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
        pinpoint.setBulkReadScope(GoBildaPinpointDriver.Register.X_VELOCITY, GoBildaPinpointDriver.Register.X_POSITION,
                                  GoBildaPinpointDriver.Register.Y_VELOCITY, GoBildaPinpointDriver.Register.Y_POSITION,
                                  GoBildaPinpointDriver.Register.H_VELOCITY, GoBildaPinpointDriver.Register.H_ORIENTATION);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);
        //frcHeading =(float)pinpoint.getHeading(Constants.unit.AU);
        speedMult = 0.5f + MathUtils.clamp(gamepad1.right_stick_x, -0.4f, 5f);
        if(servo.getPosition()!=0)servo.setPosition(0);
    }

    @Override
    public void loop() {
        //float temp = (float)pinpoint.getHeading(Constants.unit.AU.getUnnormalized());
        pinpoint.update();
        //frcHeading += gamepad1.back?-frcHeading:
        // AngleUnit.normalizeDegrees((float) pinpoint.getHeading(Constants.unit.AU.getUnnormalized())-temp);
        float shootMult;
        float maxShootSpeed = 1;
        float minShootSpeed = 0.2f;
        switch((Math.round(gamepad2.right_stick_x*100))){
            case(75):
                shootMult= maxShootSpeed;
                break;
            case(25):
                shootMult= maxShootSpeed - (minShootSpeed);
                break;
            case(0):
                shootMult= maxShootSpeed - (minShootSpeed*2);
                break;
            case(-25):
                shootMult= maxShootSpeed - (minShootSpeed*3);
                break;
            case(-75):
                shootMult= maxShootSpeed - (minShootSpeed*4);
                break;
            default:
                shootMult=(maxShootSpeed+minShootSpeed)/2.0f;
                telemetry.addLine("Error in Guitar Sigma switch statements");
                telemetry.update();
                break;
        }
        Intake.setPower(gamepad2.a?1:gamepad2.left_bumper?-1:0);
        Transfer.setPower(gamepad2.b?1:gamepad2.x?-1:0);
        Shoot.setPower(shootMult*(gamepad2.y?1:0));
        if(gamepad1.left_bumper&&((servoData&0x01)==0)){
            servoData|=0x01;
            servoData^=0x10;
            servo.setPosition((float)(servoData&0x10)/0x10);
        }else if(!gamepad1.left_bumper&&(servoData&0x01)!=0)servoData &= ~0x01;

        speedMult = 0.5f + MathUtils.clamp(gamepad1.right_stick_x, -0.4f, 5f);
        //float roboYaw = AngleUnit.RADIANS.fromUnit(Constants.unit.AU, frcHeading);
        float y = - (gamepad1.a?(gamepad1.b?0:1):gamepad1.b?-1:0);
        float x = 1.1f * (gamepad1.x?(gamepad1.y?0:-1):gamepad1.y?1:0);
        float rx = (gamepad1.dpad_down?(gamepad1.dpad_up?0:-1):gamepad1.dpad_up?1:0);
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
        telemetry.addData("Position: ", calc.getPosition());
        telemetry.addData("Servo Position: ", servo.getPosition());
    }
}
