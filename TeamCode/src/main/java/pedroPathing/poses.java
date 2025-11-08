package pedroPathing;

import com.pedropathing.geometry.Pose;

public class poses {
    //Blue alliance poses
    private final Pose startPoseBlueTop = new Pose(56, 88, Math.toRadians(180)); //Aligned with top right of robot on W4
    private final Pose startPoseBlueBottom = new Pose(56, 16, Math.toRadians(180)); //Aligned with top right of robot on W1
    private final Pose blueTopRowStart = new Pose(40, 84, Math.toRadians(180));
    private final Pose blueTopRowEnd = new Pose(20, 84, Math.toRadians(180));
    private final Pose blueMiddleRowStart = new Pose(40, 60, Math.toRadians(180));
    private final Pose blueMiddleRowEnd = new Pose(20, 60, Math.toRadians(180));
    private final Pose blueBottomRowStart = new Pose(40, 36, Math.toRadians(180));
    private final Pose blueBottomRowEnd = new Pose(20, 36, Math.toRadians(180));
    private final Pose blueScore = new Pose(); //Scoring Pose will be determined by testing

    //Red Alliance Poses
    private final Pose startPoseRedTop = new Pose(88, 88, Math.toRadians(180)); //Aligned with top left of robot on Y4
    private final Pose startPoseRedBottom = new Pose(88, 16, Math.toRadians(180)); //Aligned with left right of robot on Y1
    private final Pose redTopRowStart = new Pose(104, 84, Math.toRadians(180));
    private final Pose redTopRowEnd = new Pose(124, 84, Math.toRadians(180));
    private final Pose redMiddleRowStart = new Pose(104, 60, Math.toRadians(180));
    private final Pose redMiddleRowEnd = new Pose(124, 60, Math.toRadians(180));
    private final Pose redBottomRowStart = new Pose(104, 36, Math.toRadians(180));
    private final Pose redBottomRowEnd = new Pose(124, 36, Math.toRadians(180));
    private final Pose redScore = new Pose(); //Scoring Pose will be determined by testing

    //advit will add the april tag poses later

}

