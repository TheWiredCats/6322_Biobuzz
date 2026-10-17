package org.firstinspires.ftc.OpModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@TeleOp(name = "Temporary OpMode")
public class TemporaryTest extends OpMode {
    float RHD=0f;
    float lastHeading=0f;
    Servo servo;
    DcMotor intake;
    DcMotor transfer;
    DcMotor shoot;
    DcMotor bl;
    DcMotor fl;
    DcMotor fr;
    DcMotor br;
    GoBildaPinpointDriver pinpoint;
    //boolean[] toggler = new boolean[]{false,false,false};

    @Override
    public void init() {
        //stinger = hardwareMap.crservo.get("stinger");
        intake = hardwareMap.dcMotor.get("intake");
        transfer = hardwareMap.dcMotor.get("transfer");
        shoot = hardwareMap.dcMotor.get("shoot");
        servo = hardwareMap.servo.get("servo");
        bl = hardwareMap.dcMotor.get("bl");
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("br");
        pinpoint = (hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint"));
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        pinpoint.setBulkReadScope(GoBildaPinpointDriver.Register.H_ORIENTATION, GoBildaPinpointDriver.Register.H_VELOCITY,
                                  GoBildaPinpointDriver.Register.X_POSITION, GoBildaPinpointDriver.Register.X_VELOCITY, GoBildaPinpointDriver.Register.Y_POSITION,
                                  GoBildaPinpointDriver.Register.Y_VELOCITY);
        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);
    }

    @Override
    public void loop() {
        lastHeading=(float)pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);
        pinpoint.update();
        RHD+= ((float)pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES)-lastHeading);

        /*
        if(gamepad1.b&&!toggler[0]){
            transfer.setPower((transfer.getPower()+1)%2);
            toggler[0]=true;
        }else if(!gamepad1.a)toggler[0] = false;
        if(gamepad1.a&&!toggler[1]){
            intake.setPower((intake.getPower()+1)%2);
            toggler[1] = true;
        }else if(!gamepad1.b)toggler[1] = false;
        if(gamepad1.x&&!toggler[2]){
            shoot.setPower((shoot.getPower()+1)%2);
            toggler[2] = true;
        }else if(!gamepad1.x)toggler[2] = false;
        */

        intake.setPower(gamepad1.a?1:0);
        transfer.setPower(gamepad1.b?1:0);
        shoot.setPower(gamepad1.x?0.9:0);
        if(gamepad1.dpad_down&&!(servo.getPosition()==0)){
            servo.setPosition(0);
        }else if(gamepad1.dpad_up&&!(servo.getPosition()==1)){
            servo.setPosition(1);
        }

        if(gamepad1.y)RHD=0;
        float roboYaw = (float)Math.toRadians(RHD);
        float xPower = (float) (gamepad1.left_stick_x * 1.1f * Math.cos(roboYaw) - gamepad1.left_stick_y * Math.sin(roboYaw));
        float yPower = (float) -(gamepad1.left_stick_y * Math.cos(roboYaw) + gamepad1.left_stick_x * 1.1f * Math.sin(roboYaw));
        float denominator = Math.max(Math.abs(gamepad1.left_stick_y) + Math.abs(1.1f* gamepad1.left_stick_x)+Math.abs(gamepad1.right_stick_x), 1);
        float slower = ((0.75f)*(2-gamepad1.left_trigger-gamepad1.right_trigger)/2)+0.25f;

        float flPower = slower*(yPower+xPower+gamepad1.right_stick_x) / denominator;
        float blPower = slower*(yPower-xPower+gamepad1.right_stick_x) / denominator;
        float frPower = slower*(yPower-xPower-gamepad1.right_stick_x) / denominator;
        float brPower = slower*(yPower+xPower-gamepad1.right_stick_x) / denominator;

        fl.setPower(flPower);
        bl.setPower(blPower);
        fr.setPower(frPower);
        br.setPower(brPower);
        Pose2D pos = pinpoint.getPosition();
        float posX = (float)pos.getX(DistanceUnit.INCH);
        float posY = (float)pos.getY(DistanceUnit.INCH);
        float heading = (float)pos.getHeading(AngleUnit.DEGREES);
        telemetry.addData("True Position: ", new Pose2D(DistanceUnit.INCH, posX, posY, AngleUnit.DEGREES, heading));
        telemetry.addData("Loop Time: ", pinpoint.getLoopTime());
        telemetry.addData("Robot's Heading difference: ", roboYaw);
        telemetry.addLine("Multiplier: x"+slower);
        telemetry.update();
    }

}
