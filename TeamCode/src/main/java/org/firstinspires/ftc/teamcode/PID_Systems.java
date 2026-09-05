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
    //2 or more uses of the same code just make a damn function for it
    public LLResultTypes.FiducialResult getBiggest(@NotNull LLResult results)throws NullPointerException{
        if(!results.isValid())throw new NullPointerException();

        //make a new fiducial result that has nothing in it
        LLResultTypes.FiducialResult result = null;

        //run through all the April tags and checks for which one is the bigger
        //cuz that means it the closest to the robot and least likely to have errors
        for(LLResultTypes.FiducialResult fr:results.getFiducialResults()){

            //makes result equal to this new result if the new result is greater than the old
            //result, or if there is no old result
            if(result==null||result.getTargetArea()<fr.getTargetArea())result = fr;

        }
        //return the result
        return result;
    }
    private double wrapAngle(AngleUnit sigma, double currentAngle){
        //declaring variables
        double adjustedAngle;
        double fixedAdjustedAngle;
        //not have to type it over and over again
        double pi2=2*Math.PI;

        //checks if the angle is in degrees or in radians
        if(sigma==AngleUnit.DEGREES) {

            //we add right now to subtract by 179 later
            adjustedAngle = currentAngle + 179;
            //makes it between 0 and 360
            fixedAdjustedAngle = ((adjustedAngle % 360) + 360) % 360;
            //subtract the 179 to make our bounds now [-179, 180]
            return fixedAdjustedAngle - 179;
        }else{

            //Does the exact same thing as the top one just in radians instead of degrees

            adjustedAngle = currentAngle + Math.toRadians(179);
            fixedAdjustedAngle = ((adjustedAngle % pi2) + pi2) % pi2;
            return  fixedAdjustedAngle-Math.toRadians(179);
        }
    }
    private void confirmPosition(LLResult results, GoBildaPinpointDriver pinpoint,
                                 CameraConstants cc)throws NullPointerException{
        //noinspection CaughtExceptionImmediatelyRethrown
        try {
            LLResultTypes.FiducialResult result = getBiggest(results);

            // get rid of - 20 after kick off
            // instead of having to write it the long way we can just make a variable to write
            // the short way, also improves computations time
            int id = result.getFiducialId();

            if (id >= 0 && id < cc.APRIL_TAG_POSITIONS.length) {
                //declare and reset the real X and Y
                double currentX;
                double currentY;

                //Tx and Ty from the limelight are graphed in degrees and are also reversed so
                //we have to make them negative and turn them into radians before we can use them
                double tx = Math.toRadians(-result.getTargetXDegrees());
                double ty = Math.toRadians(-result.getTargetYDegrees());

                //tan of something that is too close to 90 starts to make it head towards
                //infinity, and  FAST
                if((Math.abs(tx)<(Math.PI/3)&&Math.abs(ty)<(Math.PI/3))&&Math.abs(ty)>Math.toRadians(1)){

                    //Get the X and Y positions of each tag
                    double apriltagX = cc.APRIL_TAG_POSITIONS[id][0];
                    double apriltagY = cc.APRIL_TAG_POSITIONS[id][1];

                    //how far away the april tag is
                    double ZDifference = cc.APRIL_TAG_HEIGHT / Math.tan(ty);

                    //how far left or right it is, negative is left and right is positive
                    double LRDifference = ZDifference * Math.tan(tx);

                    double apriltagAngle = AngleUnit.RADIANS.fromUnit((AngleUnit)cc.MEASUREMENTS.get(1),
                            cc.APRIL_TAG_POSITIONS[id][2]);

                    if(cc.APRIL_TAG_POSITIONS[id][2]<0) {
                        currentX = apriltagX
                                - ZDifference * Math.cos(apriltagAngle)
                                + LRDifference * Math.sin(apriltagAngle);

                        currentY = apriltagY
                                - ZDifference * Math.sin(apriltagAngle)
                                - LRDifference * Math.cos(apriltagAngle);
                    }else{
                        currentX=pinpoint.getPosX((DistanceUnit) cc.MEASUREMENTS.get(0));
                        currentY=pinpoint.getPosY((DistanceUnit) cc.MEASUREMENTS.get(0));
                    }

                    //set the position to what the tag says we are, and the position to what
                    //it already is
                    pinpoint.setPosition(new Pose2D((DistanceUnit) cc.MEASUREMENTS.get(0),
                            (currentX + cc.CAMERA_X_OFFSET),
                            (currentY + cc.CAMERA_Y_OFFSET),
                            (AngleUnit)cc.MEASUREMENTS.get(1),
                            pinpoint.getHeading((AngleUnit)cc.MEASUREMENTS.get(1))));

                }
            }
        }catch (NullPointerException e) {
            throw e;
        }
    }
    private void setPowers(double FL, double BL, double FR, double BR,List<DcMotor> motors){
        //clipping extra could cause some problems and going in a circle
        //so we have to divide 1 by the greatest so we can multiply the rest by that
        //we can only do that if the greatest is over 1 tho

        //check all the powers and divide everything by the greatest one if its over one
        double greatest = Math.max(Math.max(Math.abs(FL),Math.abs(BL)), Math.max(Math.abs(FR),Math.abs(BR)));
        if(greatest>1){
            //if the greatest is over 1 divide everything by the greatest to ensure the max is one
            FL/=greatest;
            BL/=greatest;
            FR/=greatest;
            BR/=greatest;
        }

        //The motors in the list should always be placed in order
        //FLMotor, BLMotor, FRMotor, BRMotor
        //So we can set their power directly
        motors.get(0).setPower(FL);
        motors.get(1).setPower(BL);
        motors.get(2).setPower(FR);
        motors.get(3).setPower(BR);
    }
    private void driveTo(DistanceUnit sigma, GoBildaPinpointDriver pinpoint, Limelight3A limelight,
                        List<DcMotor> motors, LinearOpMode ll, CameraConstants cc,
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
                confirmPosition(limelight.getLatestResult(), pinpoint, cc);
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
                    "Total: %.2f",proportional, readingIntegral, derivative, output );
            ll.telemetry.addData("Position", new Pose2D((DistanceUnit)cc.MEASUREMENTS.get(0),
                    pinpoint.getPosX((DistanceUnit) cc.MEASUREMENTS.get(0)),
                    pinpoint.getPosY((DistanceUnit) cc.MEASUREMENTS.get(0)),
                    ((AngleUnit)cc.MEASUREMENTS.get(1)),
                    pinpoint.getHeading((AngleUnit)cc.MEASUREMENTS.get(1))));
            ll.telemetry.update();


            //cos represents x but bc Shawn doesn't know how to place a pinpoint it now represents Y
            //by that logic sin now represents X
            double xPower = 1.1*output * Math.sin(roboYaw);// +OutputX*Math.cos(roboYaw);
            double yPower = -output * Math.cos(roboYaw);// - OutputX * Math.sin(roboYaw);

            //We set the power to each motor using this math, and the motors list
            setPowers((yPower+xPower),(yPower-xPower),(yPower-xPower),(yPower+xPower),motors);
        }
        //brake after we get to the x y positions
        setPowers(0,0,0,0,motors);
    }
    private void turnTo(AngleUnit sigma, LinearOpMode ll,GoBildaPinpointDriver pinpoint,
                       List<DcMotor> motors, CameraConstants cc, double desiredHeading){
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
                                wrapAngle(sigma, pinpoint.getHeading(sigma)-desiredHeading))>
                                (sigma==AngleUnit.DEGREES?5
                                        :Math.toRadians(5))))){

            //grab the previous error to use for D
            double previousError = wrapAngle(sigma, desiredHeading-pinpoint.getHeading(sigma));

            //update the pinpoint for fresh data
            pinpoint.update();

            //error is the current difference between the 2 angels
            double error = wrapAngle(sigma, desiredHeading-pinpoint.getHeading(sigma));


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
            derivative=KD*(wrapAngle(sigma,error-previousError)/dt);

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
            ll.telemetry.addData("Position", new Pose2D((DistanceUnit)cc.MEASUREMENTS.get(0),
                    pinpoint.getPosX((DistanceUnit) cc.MEASUREMENTS.get(0)),
                    pinpoint.getPosY((DistanceUnit)cc.MEASUREMENTS.get(0)),
                    (AngleUnit)cc.MEASUREMENTS.get(1),
                    pinpoint.getHeading((AngleUnit)cc.MEASUREMENTS.get(1))));
            ll.telemetry.update();

            //set the motors to either positive or negative motors
            setPowers(-total,-total,total,total,motors);
        }
        //brake after we arrive at our destination
        setPowers(0,0,0,0,motors);
    }
    public void goTo(GoBildaPinpointDriver pinpoint, Limelight3A limelight, List<DcMotor> motors,
                     LinearOpMode ll, CameraConstants cc, DistanceUnit sigma, int id,
                     double distance)throws NullPointerException{
        double convertedDistance = ((DistanceUnit)cc.MEASUREMENTS.get(0)).fromUnit(sigma, distance);
        double x = cc.APRIL_TAG_POSITIONS[id][0] - convertedDistance * Math.sin(
                cc.APRIL_TAG_POSITIONS[id][2]);
        double y = cc.APRIL_TAG_POSITIONS[id][1] - convertedDistance * Math.cos(
                cc.APRIL_TAG_POSITIONS[id][2]);
        headTo(pinpoint, limelight, motors, ll , cc, (DistanceUnit) cc.MEASUREMENTS.get(0),
                (AngleUnit)cc.MEASUREMENTS.get(1), x, y, cc.APRIL_TAG_POSITIONS[id][2]);
        lockIn((DistanceUnit)cc.MEASUREMENTS.get(0), ll, limelight, pinpoint, cc, motors, distance);


    }
    public void headTo(GoBildaPinpointDriver pinpoint, Limelight3A limelight, List<DcMotor> motors,
                       LinearOpMode ll, CameraConstants cc, DistanceUnit sigmaDis, AngleUnit sigmaAng,
                       double x, double y, double heading){
        driveTo(sigmaDis, pinpoint, limelight , motors, ll, cc, x, y);
        turnTo(sigmaAng, ll, pinpoint, motors, cc, heading);
    }
    public boolean lockOn(LinearOpMode ll, Limelight3A limelight, GoBildaPinpointDriver pinpoint,
                          List<DcMotor> motors, CameraConstants cc, double initialHeading, double shimmy) {

        //Make sure im not trying to shimmy too much or trying to move while I shouldn't be
        if(ll.opModeIsActive()&&Math.abs(shimmy)<50) {

            //if this isn't our first time looping then move a lil to the right or left
            if(shimmy!=0)turnTo(AngleUnit.DEGREES,ll,pinpoint,motors, cc,
                    initialHeading + shimmy);

            shimmy = (shimmy<0?5-shimmy:(shimmy>0?-shimmy:5));

            //give it a chance to scan after we shimmy
            ll.sleep(50);

            //get latest results from the ll
            LLResult results = limelight.getLatestResult();

            try{

                LLResultTypes.FiducialResult result = getBiggest(results);

                //if its valid head towards it
                turnTo(AngleUnit.DEGREES, ll, pinpoint, motors, cc,
                        pinpoint.getHeading(AngleUnit.DEGREES)
                                - result.getTargetXDegrees());
                return true;
            } catch (NullPointerException e) {
                //recursive hehe
                //but in serious, we're just gonna repeat the code but move a lil to the right or left
                return lockOn(ll, limelight, pinpoint, motors, cc, initialHeading,shimmy);

            }
        }else if(ll.opModeIsActive()){
            //otherwise turn to where we were at the beginning
            turnTo(AngleUnit.DEGREES, ll, pinpoint, motors, cc, initialHeading);
            return false;
        }
        return false;
    }
    public void lockIn(DistanceUnit sigma,LinearOpMode ll, Limelight3A limelight,
                       GoBildaPinpointDriver pinpoint, CameraConstants cc,List<DcMotor> motors,
                       double Distance){
        //make sure we're facing the right way
        if(lockOn(ll,limelight,pinpoint,motors, cc, pinpoint.getHeading(AngleUnit.DEGREES),0)) {

            //declare it outside so we can increase the scope beyond the fore loop
            double ZDistance;

            //get fresh limelight data
            LLResult results = limelight.getLatestResult();

            //just to have some data
            confirmPosition(results, pinpoint, cc);

            //
            LLResultTypes.FiducialResult result = getBiggest(results);

            if(0>=result.getFiducialId()-20&&result.getFiducialId()-20<cc.APRIL_TAG_POSITIONS.length){

            //get how many degrees above us, it is
            double ty = result.getTargetYDegrees();

            if(Math.abs(ty) < 60 && Math.abs(ty) > 1) {
                //we make a right triangle to find how far a way we are and use basic trig
                ZDistance = cc.APRIL_TAG_HEIGHT / Math.tan(Math.toRadians(-ty));

                //we subtract the total from how far we want to be from the tag to find
                //out how far we have to move
                double difference = ZDistance - Distance;

                //we have to get the coordinates -y,x because the whole graph is rotated 90 to the left
                double x = pinpoint.getPosX(sigma) + (difference * Math.sin(
                        pinpoint.getHeading(AngleUnit.RADIANS)));
                double y = pinpoint.getPosY(sigma) + (difference * Math.cos(
                        pinpoint.getHeading(AngleUnit.RADIANS)));
                driveTo(sigma, pinpoint, limelight, motors, ll, cc, x, y);
                }
            }
        }
    }
}
