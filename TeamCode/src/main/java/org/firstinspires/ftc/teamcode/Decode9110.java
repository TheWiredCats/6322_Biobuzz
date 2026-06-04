package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp

public class Decode9110 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //Declare our motors
        //Make sure your ID's match your configuration on the Driver Hub
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor cannonOne = hardwareMap.dcMotor.get("cannonOne");
        DcMotor cannonTwo = hardwareMap.dcMotor.get("cannonTwo");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        //Inputs raw voltage instead of reaching a speed value
        cannonOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        cannonTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Declare the Servos
        //Make sure the ID's also math your configuration on the Drive Hub
        CRServo launchOne;
        CRServo launchTwo;
        CRServo arm;

        //Setting variables for the servos
        launchOne = hardwareMap.get(CRServo.class, "launchOne");
        launchTwo = hardwareMap.get(CRServo.class, "launchTwo");
        arm = hardwareMap.get(CRServo.class, "arm");


        //Reverses direction of left motors for easier coding/math
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //So it doesn't move when we press "INIT"
        waitForStart();

        //So it stops when it is told to
        if (isStopRequested()) return;

        //Where the code executed during TeleOp happens
        while (opModeIsActive()) {
            //Reading the controller
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double Rx = gamepad1.right_stick_x;
            double speedDecrease=1+gamepad1.right_trigger;

            //Creating power numbers to give to the Motors
            double Denominator = speedDecrease*Math.max(Math.abs(y) + Math.abs(x) + Math.abs(Rx), 1);
            double FrontLeftPower = (y + x + Rx) / Denominator;
            double BackLeftPower = (y - x + Rx) / Denominator;
            double FrontRightPower = (y - x - Rx) / Denominator;
            double BackRightPower = (y + x - Rx) / Denominator;
            //Setting power to the motors
            frontLeftMotor.setPower(FrontLeftPower/speedDecrease);
            backLeftMotor.setPower(BackLeftPower/speedDecrease);
            frontRightMotor.setPower(FrontRightPower/speedDecrease);
            backRightMotor.setPower(BackRightPower/speedDecrease);

            double power=0.8;
            double powerReverse=-0.8;
            double powerOff=0;


            //Button to turn on cannon
            if(gamepad1.right_bumper){
                cannonOne.setPower(power);
                cannonTwo.setPower(powerReverse);
            }else if(!gamepad1.dpad_down){
                cannonOne.setPower(powerOff);
                cannonTwo.setPower(powerOff);
            }


            //To intake from ground
            if(gamepad1.a){
                intakeMotor.setPower(powerReverse);
            }else if (!gamepad1.x){
                intakeMotor.setPower(powerOff);
            }

            //Turns on the servos to move it up
            //After a quarter of a second the arm turns on
            if(gamepad1.b){
                launchOne.setPower(power);
                launchTwo.setPower(powerReverse);
                sleep(125);
                arm.setPower(powerReverse);
            }else if(!gamepad1.dpad_down){
                launchOne.setPower(powerOff);
                launchTwo.setPower(powerOff);
            }else if(!gamepad1.dpad_left){
                arm.setPower(powerOff);
            }

            //Moves the arm the other way
            if(gamepad1.dpad_left){
                arm.setPower(power);
            }else if (!gamepad1.b){
                arm.setPower(powerOff);
            }

            //To take an artifact out of the cannons/drop it
            //Used for patterns
            if(gamepad1.dpad_down){
                launchOne.setPower(powerReverse);
                launchTwo.setPower(power);
                cannonOne.setPower(powerReverse);
                cannonTwo.setPower(power);
            }else if(!gamepad1.right_bumper) {
                cannonOne.setPower(powerOff);
                cannonTwo.setPower(powerOff);
            }else if (!gamepad1.b){
                launchOne.setPower(powerOff);
                launchTwo.setPower(powerOff);
            }

            //Starts inputting into the robot
            if(gamepad1.x){
                intakeMotor.setPower(power);
            }else if (!gamepad1.a){
                intakeMotor.setPower(powerOff);
            }
            telemetry.addData("cannonOne Power:", cannonOne.getPower());
            telemetry.addData("cannonTwo Power:", cannonTwo.getPower());
            telemetry.addData("launchOne Power", launchOne.getPower());
            telemetry.addData("launchTwo Power", launchTwo.getPower());
            telemetry.addData("intakeMotor Power", intakeMotor.getPower());
            telemetry.addData("arm Power", arm.getPower());
            telemetry.addData("backLeftMotor Power", backLeftMotor.getPower());
            telemetry.addData("frontLeftMotor Power", frontLeftMotor.getPower());
            telemetry.addData("backRightMotor Power", backRightMotor.getPower());
            telemetry.addData("frontRightMotor Power", frontRightMotor.getPower());
            telemetry.addLine("Is Joey Tuff?");
            telemetry.addLine("FUH NAH");
            telemetry.addLine("Is Noah Cute As Hell?");
            telemetry.addLine("FUH YEH");
            telemetry.update();
        }
    }
}