package robotUtil;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public enum Team {
    RED(new Pose2D(Constants.unit.DU, 36, 63, Constants.unit.AU, 0)),
    BLUE(new Pose2D(Constants.unit.DU, -36, -63, Constants.unit.AU, 0));
    private final Pose2D PARK;
    Team(Pose2D park){
        this.PARK = park;
    }
    public Pose2D getPark(){return this.PARK;}
}
