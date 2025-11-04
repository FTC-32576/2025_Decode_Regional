package ftc.team.Java_Is_AllMight.Utils;

public enum Alliance {
    RED(24),   // ID da AprilTag para RED
    BLUE(20);  // ID da AprilTag para BLUE
    private final int tagID;

    Alliance(int tagID) {
        this.tagID = tagID;
    }

    public int getTagID() {
        return tagID;
    }
}
