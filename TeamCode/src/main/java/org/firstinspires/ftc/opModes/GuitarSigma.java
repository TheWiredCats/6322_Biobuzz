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
    private float speedMult = 0.8f;
    private RobotUtil calc;
    private float shootMult =1;
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
        //frcHeading =(float)pinpoint.getHeading(Constants.unit.AU);
    }
    @Override
    public void start(){
        speedMult = 0.5f + MathUtils.clamp(gamepad1.right_stick_x, -0.4f, 5f);
        shootMult = getShootMult();
        try{
            Thread.sleep(250);
        }catch (InterruptedException ignored){}
        servoData=0x01;
        servo.setPosition(0);
    }

    @Override
    public void loop() {
        //float temp = (float)pinpoint.getHeading(Constants.unit.AU.getUnnormalized());
        pinpoint.update();
        //frcHeading += gamepad1.back?-frcHeading:
        // AngleUnit.normalizeDegrees((float) pinpoint.getHeading(Constants.unit.AU.getUnnormalized())-temp);
        shootMult = getShootMult();
        Intake.setPower(gamepad2.a?1:gamepad2.x?-1:0);
        Transfer.setPower(gamepad2.b?1:gamepad2.y?-1:0);
        Shoot.setPower(shootMult*(gamepad2.dpad_down?1:gamepad2.dpad_up?-1:0));
        if(gamepad1.left_bumper&&((servoData&0x01)==0)){
            servoData|=0x01;
            servoData^=0x10;
            servo.setPosition(1-((float)(servoData&0x10)/0x10));
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
        calc.setPowers(FL, BL, FR, BR);
        telemetry.addData("Position: ", calc.getPosition());
        telemetry.addData("Servo Position: ", servo.getPosition());
    }
    private float getShootMult(){
        float maxShootSpeed = 0.8f;
        float minShootSpeed = 0.2f;
        switch((int)(Math.ceil(gamepad2.right_stick_x*10)+7)){
            case(14):
                return maxShootSpeed;
            case(9):
                return maxShootSpeed - (minShootSpeed);
            case(7):
                return maxShootSpeed - (minShootSpeed *2);
            case(5):
                return maxShootSpeed - (minShootSpeed *3);
            case(0):
                return maxShootSpeed - (minShootSpeed *4);
            default:
                return (maxShootSpeed + minShootSpeed)/2.0f;
        }
    }
}
