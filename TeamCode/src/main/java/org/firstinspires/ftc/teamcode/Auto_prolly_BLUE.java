package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous
public class Auto_prolly_BLUE extends LinearOpMode {
    private void setPowers(double FL, double BL, double FR, double BR ){
        //clipping extra could cause some problems and going in a circle
        //so we have to divide 1 by the greatest so we can multiply the rest by that
        //we can only do that if the greatest is over 1 tho

        //without rx, fl and br have the same power
        //aswell as bl and fr
        double greatest = Math.max(Math.abs(FL),Math.abs(BL));
         if(greatest>1){
             //if the greatest is over 1 divide evreything by the greatest to ensure the max is one
             FL/=greatest;
             BL/=greatest;
             FR/=greatest;
             BR/=greatest;
         }

        FLMotor.setPower(FL);
        BLMotor.setPower(BL);
        FRMotor.setPower(FR);
        BRMotor.setPower(BR);
    }
    private void driveTo(DistanceUnit sigma, double x, double y){
        //PID BS
        //needs testing
        final double KP = 0.0125;
        final double KD = 0;
        final double KI = 0;

        //dt is a tiny amount of times, it's a mystery tool that will help us later.
        //(cool calculus kids know what's up)
        double dt;

        //Some more mystery tools that will help us later
        double integral =0;
        double readingIntegral;
        double derivative;
        double proportional;

        //dt measuring stuff
        double currentTime;
        double previousTime=System.currentTimeMillis();

        //update it once to get fresh data
        pinpoint.update();

        double startingHeading = Math.atan2((y-pinpoint.getPosY(sigma)),(x-pinpoint.getPosX(sigma)));
        double directionX = Math.cos(startingHeading);
        double directionY = Math.sin(startingHeading);

        //PID LOOP HELL
        while(opModeIsActive()&&
                ((Math.sqrt(Math.pow(pinpoint.getVelX(sigma),2))+Math.pow(pinpoint.getVelY(sigma),2))>.5)||
                (Math.sqrt(Math.pow(x-pinpoint.getPosX(sigma),2)+Math.pow(y-pinpoint.getPosY(sigma),2))>.5)){

            //the last pinpoint data before we update to the newest
            double previousError = Math.sqrt(Math.pow(x-pinpoint.getPosX(sigma),2)+Math.pow(y-pinpoint.getPosY(sigma),2));


            //update pinpoint for some fresh data
            pinpoint.update();

            //For PID we need magnitude and Direction, the heading im gonna use for direction,
            //and for magnitude im just gonna use distance formula

            //where i want to go
            double desiredHeading=Math.atan2((y-pinpoint.getPosY(sigma)),(x-pinpoint.getPosX(sigma)));
            //where i should tell the robot were pointing so we can go where we want to go
            double roboYaw = -(pinpoint.getHeading(AngleUnit.RADIANS)+desiredHeading);

            //were using distance formula to find out the absolute value of how far away we are
            //from the target
            double error = Math.sqrt(Math.pow(x-pinpoint.getPosX(sigma),2)+Math.pow(y-pinpoint.getPosY(sigma),2));

            //P part of the PID
            proportional=error*KP;

            //dt should be the length of the loop but this is as close to that so i could use
            currentTime=System.currentTimeMillis();
            dt=Math.max((currentTime-previousTime)/1000.0,0.001);
            previousTime=currentTime;

            //without the extra part the integral could not decrease
            integral+=(((x-pinpoint.getPosX(sigma))*directionX)+(directionY*(y-pinpoint.getPosY(sigma))))*dt;

            //dont want to change the actual integral, bc that would mess up calculations
            // so we make a new variable and multiply that one by the KI
            // also I part of the PID
            readingIntegral=integral*KI;

            //and finally D part of the PID
            derivative=KD*((error-previousError)/dt);

            //use PID as magnitude
            double output = -(readingIntegral+derivative+proportional);

            //telemetry
            telemetry.addData("PID DATA","KP: %.2f, KI: %.2f, KD: %.2f, error: %.2f", KP, KI,KD, error);
            telemetry.addData("PID Data", "P: %.2f, I: %.2f, D: %.2f, Total: %2.f",proportional, readingIntegral, derivative, output );
            telemetry.update();
            //math bs i havent read
            double xPower = output * Math.sin(roboYaw);// +OutputX*Math.cos(roboYaw);
            double yPower = output * Math.cos(roboYaw);// - OutputX * Math.sin(roboYaw);
            setPowers((yPower+xPower),(yPower-xPower),(yPower-xPower),(yPower+xPower));
        }
        //break after we get to the x,y
        setPowers(0,0,0,0);
    }
    private DcMotor Intake = null;
    private DcMotor Transfer = null;
    private DcMotor FLMotor = null;
    private DcMotor BLMotor = null;
    private DcMotor FRMotor = null;
    private DcMotor BRMotor = null;
    private HuskyLens huskyLens = null;
    private Limelight3A limelight = null;
    private GoBildaPinpointDriver pinpoint;
    @Override
    public void runOpMode() throws InterruptedException {
        //Start by initallizing all the cameras, motors, and also the pinpoint

        //intake and transfer motor
        Intake = hardwareMap.dcMotor.get("intake");
        Transfer = hardwareMap.dcMotor.get("transfer");

        //camera 1 and 2
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        //pinpoint, aka the odometry computer, stuff
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,9,132, AngleUnit.DEGREES,0));

        //driving motors
        FLMotor = hardwareMap.dcMotor.get("FL");
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        BLMotor = hardwareMap.dcMotor.get("BL");
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        FRMotor = hardwareMap.dcMotor.get("FR");
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        BRMotor = hardwareMap.dcMotor.get("BR");
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        //won't move on till u click start
        waitForStart();

        //run this code once
        if (opModeIsActive()){
            driveTo(DistanceUnit.INCH,67,67);
            Intake.setPower(-1);
            Transfer.setPower(1);
        }

    }
}
