/*
Copyright 2026 FIRST Tech Challenge Team 10022

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.opModes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;

import robotUtil.*;

/**
 * This file contains a minimal example of an iterative (Non-Linear) "OpMode". An OpMode is a
 * 'program' that runs in either the autonomous or the TeleOp period of an FTC match. The names
 * of OpModes appear on the menu of the FTC Driver Station. When a selection is made from the
 * menu, the corresponding OpMode class is instantiated on the Robot Controller and executed.
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@TeleOp(name = "Control Op")
@SuppressWarnings("unused")
public class OffSeasonPrototype1I extends OpMode {
    /* Declare OpMode members. */
    //as soon as teleop selected

    private volatile boolean locatorRunning;
    private volatile Pose2D locatorPos;
    private DcMotor Intake;
    private DcMotor Transfer;
    //private HuskyLens huskyLens;
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;
    List<Integer> brokenId;
    long lastConfirmation;
    double FRCHeading;
    double lastHeading;
    private DcMotor Shoot;
    volatile LLResult result;
    private Robot robot;
    private RobotUtil calc;
    private List<DcMotor> motors;
    Servo servo;
    private byte servoData = 0x00;
    @Override
    public void init() {
        robot=Robot.startTeleOp(this);

        calc = robot.getCalc();
        motors = robot.getDrivingMotors();
        brokenId = new ArrayList<>();
        //runs once as soon as "init" is pressed
        Intake = robot.getUtilMotors().get(0);
        Transfer = robot.getUtilMotors().get(1);
        Shoot = robot.getUtilMotors().get(2);

        //imu = hardwareMap.get(IMU.class, "imu");
        pinpoint = robot.getPinpoint();
        FRCHeading=0;

        limelight = robot.getLimelight();
        //huskyLens = robot.getHuskyLens();
        servo = robot.getServo();
    }
    /*
    *Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        locatorRunning=false;
        //Thread locatorThread = new Thread(this::runCamera);
        //locatorThread.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //if(!huskyLens.knock()){telemetry.addLine("No HuskyLens Detected");}
        if(locatorPos!=null){
            pinpoint.setPosition(locatorPos);
            locatorPos=null;
        }

        Intake.setPower(gamepad1.a ? 1 : 0);
        Transfer.setPower(gamepad1.y ? 1 : 0);
        Shoot.setPower(gamepad1.b ? 1 : 0);
        if(gamepad1.left_bumper&&((servoData&0x01)==0)){
            servoData|=0x01;
            servoData^=0x10;
            servo.setPosition(1-((float)(servoData&0x10) /0x10));
        }else if(gamepad1.right_bumper&&(servoData&0x01)!=0)servoData &= ~0x01;
        //result = robot.updateLimelight();

        //Calculates how far the minimum is from the middle of the 2
        // (to know how much each should affect)
        float Difference = (Constants.MAXIMUM - Constants.MINIMUM)/2;
        //Readability of code
        float TotalTrigger=gamepad1.right_trigger+gamepad1.left_trigger;

        float speedMultiplier = (Constants.MAXIMUM-(Difference*TotalTrigger));

        if(gamepad1.start)FRCHeading=0;

        //drive variables
        double roboYaw = Math.toRadians(FRCHeading);
        float ly = -gamepad1.left_stick_y;
        float lx = gamepad1.left_stick_x * 1.1f;
        float rx = gamepad1.right_stick_x; //controls turning

        double x = lx * Math.cos(roboYaw) + ly * Math.sin(roboYaw);
        double y = ly * Math.cos(roboYaw) - lx * Math.sin(roboYaw);

        double FLMotorPower = (y + x + rx) * speedMultiplier;
        double BLMotorPower = (y - x + rx) * speedMultiplier;
        double FRMotorPower = (y - x - rx) * speedMultiplier;
        double BRMotorPower = (y + x - rx) * speedMultiplier;

        calc.setPowers(FLMotorPower, BLMotorPower, FRMotorPower, BRMotorPower);
        lastHeading=pinpoint.getHeading(AngleUnit.DEGREES);
        pinpoint.update();
        FRCHeading+=calc.wrapAngle(pinpoint.getHeading(AngleUnit.DEGREES),lastHeading);
        calc.addTelemetry();

        //long secs=(System.currentTimeMillis()/1000)-lastConfirmation;
        //long mins=secs/60;
        //if(result.isValid()){
        //    telemetry.addLine("Conforming Odometry :D");
        //}else if (lastConfirmation>0)telemetry.addLine("Odometry Last Confirmed "+((mins>0)?(mins+"Mins and "):"")+(secs%60)+" Secs Ago");
        //    else telemetry.addLine("Not yet confirmed");
        telemetry.addData("X/Y/RX:", "\nX:%.2f,\nY:%.2f,\nRX:%.2f", x, y, rx);
        telemetry.addData("SpeedMult", speedMultiplier);
        telemetry.addData("Robot Yaw", roboYaw);
        telemetry.addData("Motor Data maybe", "FL: %.2f,\nBL: %.2f,\nFR: %.2f,\nBR: %.2f", FLMotorPower, BLMotorPower, FRMotorPower, BRMotorPower);
        telemetry.addData("Motor Data", "FL: %.2f,\nBL: %.2f,\nFR: %.2f,\nBR: %.2f", motors.get(0).getPower(), motors.get(1).getPower(), motors.get(2).getPower(), motors.get(3).getPower());
        telemetry.update();
    }

    @Override
    public void stop(){
        locatorRunning=false;
    }
    @SuppressWarnings("unused")
    private void runCamera(){
        while (locatorRunning) {
            result = robot.updateLimelight();
            if(!result.isValid()){
                try{Thread.sleep(50);} catch (InterruptedException ignored) {}
                continue;
            }
            try {
                LLResultTypes.FiducialResult tags = calc.getBiggest(result.getFiducialResults());
                if (result.isValid() && tags.getTargetPoseCameraSpace().getOrientation().getPitch() > -70) {
                    limelight.pipelineSwitch(TagData.tagData.get(tags.getFiducialId() - 30).value);
                    while (limelight.getStatus().getPipelineIndex() == 0) Thread.sleep(20);
                    if (limelight.getStatus().getPipelineIndex() != 0) {
                        Pose3D position = limelight.getLatestResult().getBotpose_MT2();
                        locatorPos = (new Pose2D(DistanceUnit.METER, -position.getPosition().x,
                                -position.getPosition().y, AngleUnit.DEGREES,
                                calc.wrapAngle(180, -position.getOrientation().getYaw())));
                        lastConfirmation = System.currentTimeMillis();
                        limelight.pipelineSwitch(0);
                    }
                }
            } catch (Exception e) {
                locatorPos = null;
                if (limelight.getStatus().getPipelineIndex() != 0) limelight.pipelineSwitch(0);
            }
        }
    }
}
