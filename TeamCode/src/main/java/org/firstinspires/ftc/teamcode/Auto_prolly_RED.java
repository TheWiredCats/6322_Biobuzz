package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

@Autonomous
public class Auto_prolly_RED extends LinearOpMode {
    /*
    private void confirmPosition(LLResult results){
        if (results.isValid()) {
            for (LLResultTypes.FiducialResult fr : results.getFiducialResults()) {
                // get rid of - 20 after kick off
                int id = fr.getFiducialId()-20;
                double tx=-results.getTx();
                double ty=-results.getTy();
                if (Math.abs(tx)<60&&Math.abs(ty)<60) {
                    double apriltagX = cc.APRIL_TAG_POSITIONS[id][0];
                    double apriltagY = cc.APRIL_TAG_POSITIONS[id][1];
                    //how far away the april tag is
                    double ZDifference = cc.APRIL_TAG_HEIGHT / Math.tan(Math.toRadians(ty));
                    //how far left or right it is, negative is left and right is positive
                    double LRDifference = ZDifference * Math.tan(Math.toRadians(tx));
                    switch ((int) cc.APRIL_TAG_POSITIONS[id][2]) {
                        case (0):
                            currentX = apriltagX - ZDifference;
                            currentY = apriltagY - LRDifference;
                            break;
                        case (1):
                            currentX = apriltagX + ZDifference;
                            currentY = apriltagY + LRDifference;
                            break;
                        case (2):
                            currentX = apriltagX + LRDifference;
                            currentY = apriltagY - ZDifference;
                            break;
                        case (3):
                            currentX = apriltagX - LRDifference;
                            currentY = apriltagY + ZDifference;
                            break;
                        default:
                            currentX = pinpoint.getPosX(DistanceUnit.INCH) - cc.CAMERA_X_OFFSET;
                            currentY = pinpoint.getPosY(DistanceUnit.INCH) - cc.CAMERA_Y_OFFSET;
                            break;
                    }
                    pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, currentX + cc.CAMERA_X_OFFSET, currentY + cc.CAMERA_Y_OFFSET, AngleUnit.DEGREES, pinpoint.getHeading(AngleUnit.DEGREES)));
                }
            }
        }
    }
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
        //run untill either opmode turns off or untill were both moving less than 5 inches per second
        //and also .5 inches away from the position
        while(opModeIsActive()&&
                ((Math.sqrt(Math.pow(pinpoint.getVelX(sigma),2))+Math.pow(pinpoint.getVelY(sigma),2))>.5)||
                (Math.sqrt(Math.pow(x-pinpoint.getPosX(sigma),2)+Math.pow(y-pinpoint.getPosY(sigma),2))>.5)){

            //Confirming current position using limelight
            confirmPosition(limelight.getLatestResult());

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

            //dt is the length of the loop

            //current time represents the current time
            currentTime=System.currentTimeMillis();
            //we subtract current time by the last time we ran this and then divide by 1000
            //so we could get dt in seconds rather than in miliseconds
            dt=Math.max((currentTime-previousTime)/1000.0,0.001);
            //change previous time to the old current time so that when it loops previousTime
            //now represents the previous currentTime
            previousTime=currentTime;

            //without the extra part the integral could not decrease, due to the fact that error
            //represents the magnitute of the error, meaning its only the absolut value
            integral+=(((x-pinpoint.getPosX(sigma))*directionX)+(directionY*(y-pinpoint.getPosY(sigma))))*dt;

            //dont want to change the actual integral, bc that would mess up inner calculations
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


            //cos represents x but bc shawn dosent know how  to place a pinpoint it now represnts Y
            //by that logic sin now reprsents X
            double xPower = output * Math.sin(roboYaw);// +OutputX*Math.cos(roboYaw);
            double yPower = output * Math.cos(roboYaw);// - OutputX * Math.sin(roboYaw);
            setPowers((yPower+xPower),(yPower-xPower),(yPower-xPower),(yPower+xPower));
        }
        //break after we get to the x,y
        setPowers(0,0,0,0);
    }
     */

    private DcMotor Intake = null;
    private DcMotor Transfer = null;
    private DcMotor FLMotor = null;

    private DcMotor BLMotor = null;
    private DcMotor FRMotor = null;
    private DcMotor BRMotor = null;
    private HuskyLens huskyLens = null;
    private Limelight3A limelight = null;
    private GoBildaPinpointDriver pinpoint;
    PID_System PID = new PID_System();
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

        //pinpoint stuff
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,9,9, AngleUnit.DEGREES,0));

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

        List<DcMotor> motors = List.of(FLMotor, BLMotor,FRMotor,BRMotor);
        //won't move on till u click start
        waitForStart();

        //run this code once
        if (opModeIsActive()){
            Intake.setPower(-1);
            Transfer.setPower(1);
            PID.driveTo(DistanceUnit.INCH,pinpoint,limelight,motors,this,new CameraConstants(),67,67);
            Intake.setPower(0);
            Transfer.setPower(0);
        }

    }
}
