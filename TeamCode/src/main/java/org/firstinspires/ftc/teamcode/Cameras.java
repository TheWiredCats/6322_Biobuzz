package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;

public final class Cameras {
    public static HuskyLens setupHuskyLens(OpMode op){
        HuskyLens huskyLens = op.hardwareMap.get(HuskyLens.class, CONSTANTS.HUSKY_LENS);
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
        return huskyLens;
    }

    public static Limelight3A setupLimeLight(OpMode op, GoBildaPinpointDriver pinpoint){
        Limelight3A limelight = op.hardwareMap.get(Limelight3A.class, CONSTANTS.LIMELIGHT);
        limelight.pipelineSwitch(0);
        limelight.start();
        limelight.updateRobotOrientation(pinpoint.getHeading(CONSTANTS.unit.AU));
        return limelight;
    }

    private Cameras(){
        //so u don't accidentally make an instance of it and only call it as needed
    }

    public static List<LLResultTypes.FiducialResult> get4Biggest(LLResult results)throws NullPointerException{
        if(!results.isValid()||results.getFiducialResults().size()<4)throw new NullPointerException();

        List<LLResultTypes.FiducialResult> output = new ArrayList<>(results.getFiducialResults());

        for(int i = 0; i < 4; i++){
            for(int j = output.size()-1; j > 0; j--){
                if (output.get(j - 1).getTargetArea() < output.get(j).getTargetArea()) {
                    LLResultTypes.FiducialResult placeHolder = output.get(j - 1);
                    output.set(j - 1, output.get(j));
                    output.set(j, placeHolder);
                }
            }
        }
        Families checker = TagData.tagData.get(output.get(0).getFiducialId());
        for(int i=0;i<3;i++)if (checker!=TagData.tagData.get(output.get(i).getFiducialId()))throw new NullPointerException();

        return List.of(output.get(0), output.get(1), output.get(2), output.get(3));
    }

    public static LLResultTypes.FiducialResult getBiggest(List<LLResultTypes.FiducialResult> results)throws NullPointerException{
        if(results.isEmpty())throw new NullPointerException();
        if(results.size()==1)return results.get(0);
        //make a new fiducial result that has nothing in it
        LLResultTypes.FiducialResult result = null;

        //run through all the April tags and checks for which one is the bigger
        //cuz that means it the closest to the robot and least likely to have errors
        for(LLResultTypes.FiducialResult fr:results){

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
    public static void confirmPosition(Limelight3A limelight, LLResult results, GoBildaPinpointDriver pinpoint
    )throws NullPointerException{
        if(!results.isValid())throw new NullPointerException();
        List<LLResultTypes.FiducialResult> tags = get4Biggest(results);
        if(tags.get(0).getTargetPoseCameraSpace().getOrientation().getPitch()<-70)throw new NullPointerException();
        
    }
}
