package org.firstinspires.ftc.teamcode;

import androidx.core.math.MathUtils;

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
import org.opencv.core.Mat;

@Autonomous
public class Auto_prolly_BLUE extends LinearOpMode {
    private void setPowers(double FL, double BL, double FR, double BR ){
        FLMotor.setPower(Math.max(-1,Math.min(FL,1)));
        BLMotor.setPower(Math.max(-1,Math.min(BL,1)));
        FRMotor.setPower(Math.max(-1,Math.min(FR,1)));
        BRMotor.setPower(Math.max(-1,Math.min(BR,1)));
    }
    private void driveTo(DistanceUnit sigma, double x, double y){
        //dt is a tiny amount of times, it's a mystery tool that will help us later.
        //(cool calculus kids know what's up)
        double dt;

        //Some more mystery tools that will help us later
        double integralX = 0;
        double integralY = 0;
        double readingIntegralX;
        double readingIntegralY;
        double derivativeX;
        double derivativeY;
        double proportionalX;
        double proportionalY;

        pinpoint.update();
        while(opModeIsActive()&&Math.sqrt(Math.pow(x-pinpoint.getPosX(sigma),2)+Math.pow(y-pinpoint.getPosY(sigma),2))>1){
            double previousX=pinpoint.getPosX(sigma);
            double previousY=pinpoint.getPosY(sigma);
            double timeSinceLastUpdate=System.currentTimeMillis();
            pinpoint.update();
            double desiredHeading=Math.atan2((y-pinpoint.getPosY(sigma)),(x-pinpoint.getPosX(sigma)));
            //where i should tell the robot were pointing so we can go where i want to go
            double roboYaw = -(pinpoint.getHeading(AngleUnit.RADIANS)+desiredHeading);
            //This got a lil complicated but essentially its just a vector
            double errorX = x-pinpoint.getPosX(sigma);
            double errorY = y-pinpoint.getPosY(sigma);
            double previousErrorX = x-previousX;
            double previousErrorY = y-previousY;
            dt=Math.max(0.0001,System.currentTimeMillis()-timeSinceLastUpdate)/1000;
            integralX+=errorX*dt;
            integralY+=errorY*dt;
            //dont want to change the actual integral, bc that would mess up calculations
            // so we make a new variable and multiply that one by the KI
            readingIntegralX=integralX*KI;
            readingIntegralY=integralY*KI;
            derivativeX=KD*((errorX-previousErrorX)/dt);
            derivativeY=KD*((errorY-previousErrorY)/dt);
            proportionalX=errorX*KP;
            proportionalY=errorY*KP;

            double OutputX = 1.1*(readingIntegralX+derivativeX+proportionalX);
            double OutputY = -(readingIntegralY+derivativeY+proportionalY);

            double xPower = OutputX * Math.cos(roboYaw) + OutputY * Math.sin(roboYaw);
            double yPower = OutputY * Math.cos(roboYaw) - OutputX * Math.sin(roboYaw);
            setPowers((yPower+xPower),(yPower-xPower),(yPower-xPower),(yPower+xPower));
        }
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

    //PID BS
    //needs testing
    private final double KP = 0.0125;
    private final double KD = 0;
    private final double KI = 0;
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

        }

    }
}
