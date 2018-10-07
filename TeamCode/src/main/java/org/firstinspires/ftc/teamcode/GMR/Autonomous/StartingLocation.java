package org.firstinspires.ftc.teamcode.GMR.Autonomous;

public enum StartingLocation {
    BLUE_1 (true, true),
    BLUE_2 (true, false),
    RED_1 (false, true),
    RED_2 (false, false);

    private final boolean blue;
    private final boolean sideOne;

    StartingLocation(boolean blue, boolean sideOne) {
        this.blue = blue;
        this.sideOne = sideOne;
    }

    public boolean isBlue() {
        return blue;
    }

    public boolean isSideOne() {
        return sideOne;
    }


}
