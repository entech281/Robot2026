package util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.util.PoseCreator;

public class TestPoseCreator {

    @Test
    public void testNoTransform() {
        PoseCreator.estimateRobot(0, new Transform3d(), new Transform3d());
    }
}
