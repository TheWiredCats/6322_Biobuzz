package RobotUtil;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

final class Cameras extends Robot {
    private Cameras(OpMode op){
        super(op);
    }
    private static Cameras instance;
    static Cameras getCameras(OpMode op){return instance==null?instance=new Cameras(op):instance;}
    @Override
    protected HuskyLens setupHuskyLens(OpMode op){
        HuskyLens huskyLens = op.hardwareMap.get(HuskyLens.class, CONSTANTS.HUSKY_LENS);
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
        return huskyLens;
    }

    @Override
    protected Limelight3A setupLimeLight(OpMode op, GoBildaPinpointDriver pinpoint){
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

    @Override
    public synchronized LLResult updateLimelight(){
        super.getLimelight().updateRobotOrientation(super.getPinpoint().getHeading(AngleUnit.DEGREES));
        return super.getLimelight().getLatestResult();
    }


}
