package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.jetbrains.annotations.NotNull;

import java.util.List;

public class PID_Systems {
    private PID_Systems(){
        //so u don't accidentally make an instance of it and only call it as needed
    }
    //2 or more uses of the same code just make a damn function for it
    private static void driveTo(DistanceUnit sigma, GoBildaPinpointDriver pinpoint, Limelight3A limelight,
                        List<DcMotor> motors, LinearOpMode ll,
                        double x, double y){

        //PID BS

        //needs testing/tuning
        //multiplier for each part of the PID
        final double KP = 0.25275;
        final double KD = 0.155;
        final double KI = -0.075;

        //dt is a tiny amount of times, it's a mystery tool that will help us later.
        //(cool calculus kids know what's up)
        double dt;

        //Some more mystery tools that will help us later
        double integral =0;
        double readingIntegral;
        double derivative;
        double proportional;

        //dt measuring stuff
        long currentTime;
        long previousTime=System.nanoTime();

        //This tells us the angel we want to travel in
        double startingHeading = Math.atan2(-(y-pinpoint.getPosY(sigma)),(x-pinpoint.getPosX(sigma)));

        //We cant really read 2d direction so we split it into its X and Y direction
        double directionX = Math.sin(startingHeading);
        double directionY = Math.cos(startingHeading);

        //PID LOOP HELL
        //run until either op mode turns off or until we're both moving less than .5 inches per second
        //and also .5 inches away from the position
        while(ll.opModeIsActive()&&((Math.sqrt(Math.pow(pinpoint.getVelX(sigma), 2) +
                Math.pow(pinpoint.getVelY(sigma), 2)) > 0.5) || (Math.sqrt(
                        Math.pow(x - pinpoint.getPosX(sigma), 2) +
                        Math.pow(y - pinpoint.getPosY(sigma), 2)) > 0.5))){

            //Confirming current position using limelight
            try {
                LimelightCalculator.confirmPosition(limelight.getLatestResult(), pinpoint);
            }catch(NullPointerException ignored){}

            //the last pinpoint data before we update to the newest
            double previousError = Math.sqrt(Math.pow(x-pinpoint.getPosX(sigma),2)+
                    Math.pow(y-pinpoint.getPosY(sigma),2));


            //update pinpoint for some fresh data
            pinpoint.update();

            //For PID, we need magnitude and Direction, the heading I'm gonna use for direction,
            //and for magnitude I'm just gonna use distance formula

            //The angel of where I am compared to where I want to be going
            double desiredHeading=Math.atan2((y-pinpoint.getPosY(sigma)),-(x-pinpoint.getPosX(sigma)));
            //where I should tell the robot we're pointing so we can go where we want to go
            double roboYaw = (pinpoint.getHeading(AngleUnit.RADIANS)-desiredHeading);

            //we're using distance formula to find out the absolute value of how far away we are
            //from the target, or if you've taken algebra 2, we're finding the magnitude
            double error = Math.sqrt(Math.pow(x - pinpoint.getPosX(sigma), 2) +
                    Math.pow(y-pinpoint.getPosY(sigma), 2));

            //This part tells us how fast to go depending on how far away we CURRENTLY are
            //This is known as the present or proportional part of the pid or in other words
            //P part of the PID
            proportional=error*KP;

            //dt is the length of the loop

            //current time represents the current time
            currentTime=System.nanoTime();

            //we subtract current time by the last time we ran this and then divide by 1000
            //so we could get dt in seconds rather than in milliseconds
            dt=Math.max((currentTime-previousTime)/10000000.0, 0.000001);

            //change previous time to the old current time so that when it loops previousTime
            //now represents the previous currentTime
            previousTime=currentTime;

            //without the extra part the integral could not decrease, due to the fact that error
            //represents the magnitude of the error, meaning it's only the absolute value
            integral += (((x - pinpoint.getPosX(sigma)) * directionX) + (directionY *
                    (y - pinpoint.getPosY(sigma)))) * dt;

            //don't want to change the actual integral, bc that would mess up inner calculations
            // so we make a new variable and multiply that one by the KI also it represents the
            //integral, or inherited errors from PAST loops in other words
            // I part of the PID
            readingIntegral=integral*KI;

            //This represents the Derivative, or destiny of the error, it handles
            //where the error WILL become or in other words
            //D part of the PID
            derivative=KD*((error-previousError)/dt);

            //use PID as magnitude
            double output = (readingIntegral+derivative+proportional);

            //telemetry data being added
            ll.telemetry.addData("PID DATA","KP: %.2f, KI: %.2f, KD: %.2f, " +
                    "error: %.2f, dt in secs: %.4f", KP, KI,KD, error, dt);
            ll.telemetry.addData("PID Data", "P: %.2f, I: %.2f, D: %.2f, " +
                    "Total: %.2f",proportional, readingIntegral, derivative, output);
            ll.telemetry.addData("Position", new Pose2D(CONSTANTS.DISTANCE,
                    pinpoint.getPosX(CONSTANTS.DISTANCE),
                    pinpoint.getPosY(CONSTANTS.DISTANCE),
                    CONSTANTS.ANGLE,
                    pinpoint.getHeading(CONSTANTS.ANGLE)));
            ll.telemetry.update();


            //cos represents x but bc Shawn doesn't know how to place a pinpoint it now represents Y
            //by that logic sin now represents X
            double xPower = 1.1*output * Math.sin(roboYaw);// +OutputX*Math.cos(roboYaw);
            double yPower = -output * Math.cos(roboYaw);// - OutputX * Math.sin(roboYaw);

            //We set the power to each motor using this math, and the motors list
            Motors.setPowers((yPower+xPower),(yPower-xPower),(yPower-xPower),(yPower+xPower),motors);
        }
        //brake after we get to the x y positions
        Motors.setPowers(0,0,0,0,motors);
    }
    private static void turnTo(AngleUnit sigma, LinearOpMode ll, GoBildaPinpointDriver pinpoint,
                        List<DcMotor> motors, double desiredHeading){
        //needs tuning
        //multiplier for each part of the PID
        final double KP = 0.2;
        final double KD = 0;
        final double KI = 0;

        //dt is the time between loops, it's gonna be a very small amount of time
        double dt;
        //set previous time to rn as a default value
        long previousTime=System.nanoTime();
        long currentTime;

        //each part of the PID
        double proportional;
        double integral=0;
        double derivative;

        //update pinpoint before we start loop for fresh data
        pinpoint.update();
        while(ll.opModeIsActive()&&
                ((Math.abs((pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)))>0.5)||
                        (Math.abs(
                                LimelightCalculator.wrapAngle(sigma, pinpoint.getHeading(sigma)-desiredHeading))>
                                (sigma==AngleUnit.DEGREES?5
                                        :Math.toRadians(5))))){

            //grab the previous error to use for D
            double previousError = LimelightCalculator.wrapAngle(sigma, desiredHeading-pinpoint.getHeading(sigma));

            //update the pinpoint for fresh data
            pinpoint.update();

            //error is the current difference between the 2 angels
            double error = LimelightCalculator.wrapAngle(sigma, desiredHeading-pinpoint.getHeading(sigma));


            //P part of PID represents how much change we still need to do
            //but is often the cause of oscillation when KP is too high
            proportional=error*KP;

            //obvious name is obvious
            currentTime=System.nanoTime();
            //for all but the first loop this measure almost the time it takes to do the entire loop
            //we divide by 1000 so it gives the data to us in seconds
            dt=Math.max((currentTime-previousTime),1)/1000000.0;
            //set the last current time to previous time to be used in the next loop
            previousTime=currentTime;

            //D part of the PID represents how much error is changing, we're taking the derivative
            //of the different positions by simply using the limit definition
            derivative=KD * (LimelightCalculator.wrapAngle(sigma,error-previousError)/dt);

            //I part of the PID represents how much the error has changed, we take the integral
            //by simply multiplying by dt and adding over every loop
            integral+=error*dt;

            //the output of the PID is represented by the addition of each part of the pid
            //multiplied by their respective multiplier
            double total=(proportional+derivative+(integral*KI));

            //telemetry data for tuning and testing
            ll.telemetry.addData("PID DATA","KP: %.2f, KI: %.2f, KD: %.2f, error: " +
                    "%.2f, dt in secs: %.2f", KP, KI,KD, error, dt);
            ll.telemetry.addData("PID Data", "P: %.2f, I: %.2f, D: %.2f, " +
                    "Total: %.2f",proportional, integral*KI, derivative, total);
            ll.telemetry.addData("Position", new Pose2D(CONSTANTS.DISTANCE,
                    pinpoint.getPosX(CONSTANTS.DISTANCE),
                    pinpoint.getPosY(CONSTANTS.DISTANCE),
                    CONSTANTS.ANGLE,
                    pinpoint.getHeading(CONSTANTS.ANGLE)));
            ll.telemetry.update();

            //set the motors to either positive or negative motors
            Motors.setPowers(-total,-total,total,total,motors);
        }
        //brake after we arrive at our destination
        Motors.setPowers(0,0,0,0,motors);
    }
    public static void goTo(GoBildaPinpointDriver pinpoint, Limelight3A limelight, List<DcMotor> motors,
                     LinearOpMode ll, DistanceUnit sigma, int id,
                     double distance)throws NullPointerException{
        double convertedDistance = (CONSTANTS.DISTANCE).fromUnit(sigma, distance);
        double x = CONSTANTS.APRIL_TAG_POSITIONS[id][0] - convertedDistance * Math.sin(
                AngleUnit.RADIANS.fromUnit(CONSTANTS.ANGLE, CONSTANTS.APRIL_TAG_POSITIONS[id][2]));
        double y = CONSTANTS.APRIL_TAG_POSITIONS[id][1] - convertedDistance * Math.cos(
                AngleUnit.RADIANS.fromUnit(CONSTANTS.ANGLE, CONSTANTS.APRIL_TAG_POSITIONS[id][2]));
        headTo(pinpoint, limelight, motors, ll , CONSTANTS.DISTANCE,
                CONSTANTS.ANGLE, x, y, CONSTANTS.APRIL_TAG_POSITIONS[id][2]);
        lockIn(CONSTANTS.DISTANCE, ll, limelight, pinpoint, motors, convertedDistance);
    }
    public static void headTo(GoBildaPinpointDriver pinpoint, Limelight3A limelight, List<DcMotor> motors,
                       LinearOpMode ll, DistanceUnit sigmaDis, AngleUnit sigmaAng,
                       double x, double y, double heading){
        driveTo(sigmaDis, pinpoint, limelight , motors, ll, x, y);
        turnTo(sigmaAng, ll, pinpoint, motors, heading);
    }
    public static boolean lockOn(LinearOpMode ll, Limelight3A limelight, GoBildaPinpointDriver pinpoint,
                          List<DcMotor> motors, double initialHeading, double shimmy) {

        //Make sure im not trying to shimmy too much or trying to move while I shouldn't be
        if(ll.opModeIsActive()&&Math.abs(shimmy)<50) {

            //if this isn't our first time looping then move a lil to the right or left
            if(shimmy!=0)turnTo(AngleUnit.DEGREES,ll,pinpoint,motors,
                    initialHeading + shimmy);

            shimmy = (shimmy<0?5-shimmy:(shimmy>0?-shimmy:5));

            //give it a chance to scan after we shimmy
            ll.sleep(50);

            //get latest results from the ll
            LLResult results = limelight.getLatestResult();

            try{

                LLResultTypes.FiducialResult result = LimelightCalculator.getBiggest(results);

                //if its valid head towards it
                turnTo(AngleUnit.DEGREES, ll, pinpoint, motors,
                pinpoint.getHeading(AngleUnit.DEGREES) - result.getTargetXDegrees());
                return true;
            } catch (NullPointerException e) {
                //recursive hehe
                //but in serious, we're just gonna repeat the code but move a lil to the right or left
                return lockOn(ll, limelight, pinpoint, motors, initialHeading,shimmy);

            }
        }else if(ll.opModeIsActive()){
            //otherwise turn to where we were at the beginning
            turnTo(AngleUnit.DEGREES, ll, pinpoint, motors, initialHeading);
            return false;
        }
        return false;
    }
    public static void lockIn(DistanceUnit sigma, LinearOpMode ll, Limelight3A limelight,
                       GoBildaPinpointDriver pinpoint, List<DcMotor> motors,
                       double Distance){
        //make sure we're facing the right way
        if(lockOn(ll,limelight,pinpoint,motors, pinpoint.getHeading(AngleUnit.DEGREES),0)) {

            //declare it outside so we can increase the scope beyond the fore loop
            double ZDistance;

            //get fresh limelight data
            LLResult results = limelight.getLatestResult();

            //just to have some data
            LimelightCalculator.confirmPosition(results, pinpoint);

            //
            LLResultTypes.FiducialResult result = LimelightCalculator.getBiggest(results);

            if(0>=result.getFiducialId()-20&&result.getFiducialId()-20<CONSTANTS.APRIL_TAG_POSITIONS.length){

            //get how many degrees above us, it is
            double ty = result.getTargetYDegrees();

            if(Math.abs(ty) < 60 && Math.abs(ty) > 1) {
                //we make a right triangle to find how far a way we are and use basic trig
                ZDistance = CONSTANTS.APRIL_TAG_HEIGHT / Math.tan(Math.toRadians(-ty));

                //we subtract the total from how far we want to be from the tag to find
                //out how far we have to move
                double difference = ZDistance - Distance;

                //we have to get the coordinates -y,x because the whole graph is rotated 90 to the left
                double x = pinpoint.getPosX(sigma) + (difference * Math.sin(
                        pinpoint.getHeading(AngleUnit.RADIANS)));
                double y = pinpoint.getPosY(sigma) + (difference * Math.cos(
                        pinpoint.getHeading(AngleUnit.RADIANS)));
                driveTo(sigma, pinpoint, limelight, motors, ll, x, y);
                }
            }
        }
    }
}
