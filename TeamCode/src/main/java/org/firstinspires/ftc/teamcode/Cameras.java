package org.firstinspires.ftc.teamcode;

import androidx.core.math.MathUtils;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

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
        List<LLFieldMap.Fiducial> map = new ArrayList<>();
        for(int i = 30; i <= 45;i++){
            map.add(new LLFieldMap.Fiducial(i, 82.55, "apriltag3_36h11_classic", TagData.transforms.get(i-30), true));
        }
        limelight.uploadFieldmap(new LLFieldMap(map, "ftc"), null);
        limelight.updateRobotOrientation(pinpoint.getHeading(CONSTANTS.unit.AU));
        return limelight;
    }

    private Cameras(){
        //so u don't accidentally make an instance of it and only call it as needed
    }

    public static List<LLResultTypes.FiducialResult> get4Biggest(LLResult results)throws MonkeyBusiness {
        if(!results.isValid()||results.getFiducialResults().size()<4)throw new MonkeyBusiness();

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
        for(int i=0;i<3;i++)if (checker!=TagData.tagData.get(output.get(i).getFiducialId()))throw new MonkeyBusiness();

        return List.of(output.get(0), output.get(1), output.get(2), output.get(3));
    }

    public static LLResultTypes.FiducialResult getBiggest(List<LLResultTypes.FiducialResult> results)throws MonkeyBusiness {
        if(results.isEmpty())throw new MonkeyBusiness();
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
    public static double wrapAngle(double angle1, double angle2){
        double adjustedAngle1 = angle1 - Math.copySign(360, angle1);
        return Math.min(adjustedAngle1 - angle2, angle1 - angle2);
    }
    public static void confirmPosition(Limelight3A limelight, GoBildaPinpointDriver pinpoint, LinearOpMode ll)throws MonkeyBusiness {
        LLResult result = limelight.getLatestResult();
        double oldTimestamp = result.getTimestamp();
        LLResultTypes.FiducialResult tags = Cameras.getBiggest(result.getFiducialResults());
        if(tags.getTargetPoseCameraSpace().getOrientation().getPitch()<-70)throw new MonkeyBusiness();
        limelight.pipelineSwitch(TagData.tagData.get(tags.getFiducialId()-30).value);
        long startTime = System.currentTimeMillis();
        LLResult freshResult = limelight.getLatestResult();
        while (ll.opModeIsActive() && (freshResult.getTimestamp() <= oldTimestamp)) {
            if (System.currentTimeMillis() - startTime > 40)break;
            ll.sleep(2);
            freshResult = limelight.getLatestResult();
        }
        Pose3D position = freshResult.getBotpose_MT2();
        pinpoint.setPosition(new Pose2D(DistanceUnit.METER, -position.getPosition().x,
                -position.getPosition().y, AngleUnit.DEGREES,
                Cameras.wrapAngle(180, -position.getOrientation().getYaw(AngleUnit.DEGREES))));
        limelight.pipelineSwitch(0);
    }
}
