package org.firstinspires.ftc.teamcode;

public enum AprilTagPosition {
    DOWN(0),
    UP(1);

    private final int value;

    AprilTagPosition(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}
