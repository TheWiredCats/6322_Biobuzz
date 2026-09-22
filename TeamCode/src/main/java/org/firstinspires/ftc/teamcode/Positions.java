package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import RobotUtil.CONSTANTS;

public enum Positions {
    TopLeft(new Pose2D(CONSTANTS.unit.DU, 60, 60, CONSTANTS.unit.AU, 0)),
    BottomLeft(new Pose2D(CONSTANTS.unit.DU, -60, 60, CONSTANTS.unit.AU, 0)),
    TopCenter(new Pose2D(CONSTANTS.unit.DU, 60, 0, CONSTANTS.unit.AU, 0)),
    BottomCenter(new Pose2D(CONSTANTS.unit.DU, -60, 0, CONSTANTS.unit.AU, 0)),
    TopRight(new Pose2D(CONSTANTS.unit.DU, 60, -60, CONSTANTS.unit.AU, 0)),
    BottomRight(new Pose2D(CONSTANTS.unit.DU, +60, -60, CONSTANTS.unit.AU, 0));

    private final Pose2D saveStation;
    Positions(Pose2D sigma){
        this.saveStation = sigma;
    }

    /**
     * Supposedly Correct Way To Do OOP
     * @return the closest save station
     */
    public Pose2D getSaveStation(){
        return this.saveStation;
    }
}
