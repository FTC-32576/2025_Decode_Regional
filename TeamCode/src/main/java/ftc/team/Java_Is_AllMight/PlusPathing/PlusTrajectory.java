package ftc.team.Java_Is_AllMight.PlusPathing;

import java.util.List;

public class PlusTrajectory {

    private final List<PlusSegment> segments;

    public PlusTrajectory(List<PlusSegment> segments) {
        this.segments = segments;
    }

    public List<PlusSegment> getSegments() {
        return segments;
    }
}
