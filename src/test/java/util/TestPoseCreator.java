package util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.util.PoseCreator;

public class TestPoseCreator {

    @Test
    public void testNoTransform() {        
        assertEquals(11.8779798, PoseCreator.estimateRobot(1, new Transform3d(), new Transform3d()).getX());
        assertEquals(7.4247756, PoseCreator.estimateRobot(1, new Transform3d(), new Transform3d()).getY());
        assertEquals(0.889, PoseCreator.estimateRobot(1, new Transform3d(), new Transform3d()).getZ());
        assertEquals(11.3118646, PoseCreator.estimateRobot(4, new Transform3d(), new Transform3d()).getX());
        assertEquals(4.0346376, PoseCreator.estimateRobot(4, new Transform3d(), new Transform3d()).getY());
        assertEquals(1.12395, PoseCreator.estimateRobot(4, new Transform3d(), new Transform3d()).getZ());
    }

    @Test
    public void testCameraTransform() {
        assertEquals(12.8779798, PoseCreator.estimateRobot(1, new Transform3d(1, 0, 0, new Rotation3d()), new Transform3d()).getX());
        assertEquals(7.4247756, PoseCreator.estimateRobot(1, new Transform3d(1, 0, 0, new Rotation3d()), new Transform3d()).getY());
        assertEquals(0.889, PoseCreator.estimateRobot(1, new Transform3d(1, 0, 0, new Rotation3d()), new Transform3d()).getZ());
        assertEquals(11.3118646, PoseCreator.estimateRobot(4, new Transform3d(0, -1, 0, new Rotation3d()), new Transform3d()).getX());
        assertEquals(3.0346376, PoseCreator.estimateRobot(4, new Transform3d(0, -1, 0, new Rotation3d()), new Transform3d()).getY());
        assertEquals(1.12395, PoseCreator.estimateRobot(4, new Transform3d(0, -1, 0, new Rotation3d()), new Transform3d()).getZ());
        assertEquals(12.519177399999998, PoseCreator.estimateRobot(9, new Transform3d(0, 0, 2, new Rotation3d()), new Transform3d()).getX());
        assertEquals(3.6790375999999996, PoseCreator.estimateRobot(9, new Transform3d(0, 0, 2, new Rotation3d()), new Transform3d()).getY());
        System.out.println(PoseCreator.estimateRobot(9, new Transform3d(0, 0, 2, new Rotation3d()), new Transform3d()).getZ());
        assertEquals(3.12395, PoseCreator.estimateRobot(9, new Transform3d(0, 0, 2, new Rotation3d()), new Transform3d()).getZ());
    }

    @Test
    public void testRobotTransform() {
        assertEquals(12.8779798, PoseCreator.estimateRobot(1, new Transform3d(), new Transform3d(1, 0, 0, new Rotation3d())).getX());
        assertEquals(7.4247756, PoseCreator.estimateRobot(1, new Transform3d(), new Transform3d(1, 0, 0, new Rotation3d())).getY());
        assertEquals(0.889, PoseCreator.estimateRobot(1, new Transform3d(), new Transform3d(1, 0, 0, new Rotation3d())).getZ());
        assertEquals(11.3118646, PoseCreator.estimateRobot(4, new Transform3d(), new Transform3d(0, -1, 0, new Rotation3d())).getX());
        assertEquals(3.0346376, PoseCreator.estimateRobot(4, new Transform3d(), new Transform3d(0, -1, 0, new Rotation3d())).getY());
        assertEquals(1.12395, PoseCreator.estimateRobot(4, new Transform3d(), new Transform3d(0, -1, 0, new Rotation3d())).getZ());
    }
}
