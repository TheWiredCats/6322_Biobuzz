package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.jetbrains.annotations.NotNull;

public class LimelightCalculator {
    private LimelightCalculator(){
        //so u don't accidentally make an instance of it and only call it as needed
    }
    public static LLResultTypes.FiducialResult getBiggest(@NotNull LLResult results)throws NullPointerException{
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
    public static double wrapAngle(AngleUnit sigma, double currentAngle){
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
    public static void confirmPosition(LLResult results, GoBildaPinpointDriver pinpoint
    )throws NullPointerException{
        //noinspection CaughtExceptionImmediatelyRethrown
        try {
            LLResultTypes.FiducialResult result = getBiggest(results);

            // get rid of - 20 after kick off
            // instead of having to write it the long way we can just make a variable to write
            // the short way, also improves computations time
            int id = result.getFiducialId();

            if (id >= 0 && id < CONSTANTS.APRIL_TAG_POSITIONS.length) {
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
                    double apriltagX = CONSTANTS.APRIL_TAG_POSITIONS[id][0];
                    double apriltagY = CONSTANTS.APRIL_TAG_POSITIONS[id][1];

                    //how far away the april tag is
                    double ZDifference = CONSTANTS.APRIL_TAG_HEIGHT / Math.tan(ty);

                    //how far left or right it is, negative is left and right is positive
                    double LRDifference = ZDifference * Math.tan(tx);

                    double apriltagAngle = AngleUnit.RADIANS.fromUnit(CONSTANTS.ANGLE,
                            CONSTANTS.APRIL_TAG_POSITIONS[id][2]);

                    if(CONSTANTS.APRIL_TAG_POSITIONS[id][2]<0) {
                        currentX = apriltagX
                                - ZDifference * Math.cos(apriltagAngle)
                                + LRDifference * Math.sin(apriltagAngle);

                        currentY = apriltagY
                                - ZDifference * Math.sin(apriltagAngle)
                                - LRDifference * Math.cos(apriltagAngle);
                    }else {
                        currentX = pinpoint.getPosX(CONSTANTS.DISTANCE);
                        currentY = pinpoint.getPosY(CONSTANTS.DISTANCE);
                    }
                    //how far away we are from what it says we are
                    double distanceDifference = Math.sqrt(
                            Math.pow((currentX - pinpoint.getPosX(CONSTANTS.DISTANCE)), 2)
                                    + Math.pow(currentY - pinpoint.getPosY(CONSTANTS.DISTANCE), 2));

                    //only runs if our "actual" distance is somewhat close to what we think we are
                    //to prevent glitches messing with our odometry
                    if(distanceDifference < 20){
                        //set the position to what the tag says we are, and the position to what
                        //it already is
                        pinpoint.setPosition(new Pose2D(CONSTANTS.DISTANCE,
                                (currentX + CONSTANTS.CAMERA_X_OFFSET),
                                (currentY + CONSTANTS.CAMERA_Y_OFFSET),
                                CONSTANTS.ANGLE,
                                pinpoint.getHeading(CONSTANTS.ANGLE)));
                    }

                }
            }
        }catch (NullPointerException e) {
            throw e;
        }
    }
}
