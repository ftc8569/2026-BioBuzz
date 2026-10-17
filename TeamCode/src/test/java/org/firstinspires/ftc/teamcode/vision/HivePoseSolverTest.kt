package org.firstinspires.ftc.teamcode.vision

import org.junit.Assert.*
import org.junit.Test

class HivePoseSolverTest {
    private val solver = HivePoseSolver()

    @Test fun oneTagRecoversRobotAndTipFromOneFrame() {
        val expected = RobotPose(0.42, -0.31, Math.toRadians(28.0))
        val tag = HiveFieldModel.tagPose(30, Math.toRadians(49.0))!!
        val robotToTag = expected.asTransform().inverse() * tag

        val result = solver.solve(listOf(TagObservation(30, robotToTag)))!!

        assertEquals(expected.xMeters, result.robot.xMeters, 0.005)
        assertEquals(expected.yMeters, result.robot.yMeters, 0.005)
        assertEquals(expected.headingRadians, result.robot.headingRadians, 0.005)
        assertEquals(Math.toRadians(49.0), result.redHiveAngleRadians!!, 0.005)
        assertNull(result.blueHiveAngleRadians)
    }

    @Test fun independentHivesAndSeveralTagsShareOneFrameSolution() {
        val expected = RobotPose(-0.26, 0.51, Math.toRadians(-35.0))
        val observations = listOf(30, 31, 38, 40).map { id ->
            val angle = if (id < 38) Math.toRadians(42.0) else Math.toRadians(-53.0)
            TagObservation(id, expected.asTransform().inverse() * HiveFieldModel.tagPose(id, angle)!!)
        }

        val result = solver.solve(observations)!!

        assertEquals(expected.xMeters, result.robot.xMeters, 0.005)
        assertEquals(expected.yMeters, result.robot.yMeters, 0.005)
        assertEquals(expected.headingRadians, result.robot.headingRadians, 0.005)
        assertEquals(Math.toRadians(42.0), result.redHiveAngleRadians!!, 0.005)
        assertEquals(Math.toRadians(-53.0), result.blueHiveAngleRadians!!, 0.005)
        assertEquals(4, result.tagCount)
    }

    @Test fun noTagsProduceNoCurrentPose() {
        assertNull(solver.solve(emptyList()))
    }

    @Test fun halfTurnIsNotMistakenForZeroOrientationError() {
        assertEquals(Math.PI, Mat3.rotZ(Math.PI).log().norm(), 1e-6)
    }

    @Test fun oneBadDetectionDoesNotMoveTheCombinedEstimate() {
        val expected = RobotPose(0.16, 0.34, Math.toRadians(22.0))
        val good = listOf(30, 31, 32, 33).map { id ->
            TagObservation(id, expected.asTransform().inverse() * HiveFieldModel.tagPose(id, Math.toRadians(35.0))!!)
        }
        val bad = TagObservation(34, Transform3(Vec3(1.8, -0.8, 0.2), Mat3.IDENTITY))

        val result = solver.solve(good + bad)!!

        assertEquals(expected.xMeters, result.robot.xMeters, 0.03)
        assertEquals(expected.yMeters, result.robot.yMeters, 0.03)
        assertEquals(expected.headingRadians, result.robot.headingRadians, 0.03)
    }
}
