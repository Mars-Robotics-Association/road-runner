package com.acmerobotics.roadrunner

import org.junit.jupiter.api.Test
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sqrt
import kotlin.test.assertEquals
import kotlin.test.assertTrue

private fun testPath() = TangentPath(
    PositionPathSeqBuilder(
        Vector2d(0.0, 0.0),
        Rotation2d.exp(0.0),
        1e-6,
    )
        .splineTo(
            Vector2d(30.0, 30.0),
            Rotation2d.exp(PI / 2),
        )
        .splineTo(
            Vector2d(10.0, 60.0),
            Rotation2d.exp(PI),
        )
        .build()
        .first(),
    0.0,
)

class VelDependentConstraintsTest {
    private val feedforward = MotorFeedforward(0.5, 0.15, 0.02)
    private val maxVoltage = 12.0

    @Test
    fun testConstantConstraintsMatchConstantProfile() {
        // a straight path with constant constraints should reproduce the exact trapezoidal profile
        val path = TangentPath(
            PositionPathSeqBuilder(
                Vector2d(0.0, 0.0),
                Rotation2d.exp(0.0),
                1e-6,
            )
                .lineToX(10.0)
                .build()
                .first(),
            0.0,
        )

        val p = TimeProfile(
            profile(
                TEST_PROFILE_PARAMS,
                path,
                0.0,
                TranslationalVelConstraint(5.0),
                ProfileAccelConstraint(-5.0, 5.0),
            ).baseProfile
        )

        // accelerate 2.5in to 5in/s in 1s, cruise 5in for 1s, decelerate 2.5in in 1s
        assertEquals(3.0, p.duration, 1e-6)
        assertEquals(5.0, p[0.0][2], 1e-6)
        assertEquals(5.0, p[1.5][1], 1e-6)
    }

    @Test
    fun testAccelConstraintVelDependence() {
        val kinematics = MecanumKinematics(12.0)
        val path = testPath()
        val constraint = kinematics.WheelVoltageConstraint(feedforward, maxVoltage)

        val pose = path[0.0, 3]
        val accelAtRest = constraint.minMaxProfileAccel(pose, path, 0.0, 0.0)
        val accelAtSpeed = constraint.minMaxProfileAccel(pose, path, 0.0, 40.0)

        // back-EMF costs forward acceleration and buys braking at speed
        assertTrue(accelAtSpeed.max < accelAtRest.max)
        assertTrue(accelAtSpeed.min < accelAtRest.min)
        assertTrue(accelAtRest.max > 0.0)
        assertTrue(accelAtRest.min < 0.0)

        // the velocity-independent method matches the rest evaluation
        assertEquals(accelAtRest, constraint.minMaxProfileAccel(pose, path, 0.0))
    }

    private fun maxWheelVoltage(
        path: PosePath,
        profile: DisplacementProfile,
        wheelVels: (Pose2dDual<Time>) -> List<DualNum<Time>>,
    ): Double {
        val traj = DisplacementTrajectory(path, profile)
        var maxWheelVoltage = 0.0
        for (s in range(0.0, path.length(), 1000)) {
            val txWorldRobot = traj[s]
            for (wheelVel in wheelVels(txWorldRobot)) {
                maxWheelVoltage = maxOf(maxWheelVoltage, abs(feedforward.compute(wheelVel)))
            }
        }
        return maxWheelVoltage
    }

    @Test
    fun testMecanumWheelVoltageProfile() {
        val kinematics = MecanumKinematics(12.0)
        val path = testPath()
        val constraint = kinematics.WheelVoltageConstraint(feedforward, maxVoltage)

        fun generate(params: ProfileParams) =
            profile(params, path, 0.0, constraint, constraint).baseProfile

        fun wheelVels(txWorldRobot: Pose2dDual<Time>) =
            kinematics.inverse(txWorldRobot.inverse() * txWorldRobot.velocity()).all()

        // center-sampled constraints leave a resolution-dependent discretization error between samples
        val coarse = maxWheelVoltage(path, generate(TEST_PROFILE_PARAMS), ::wheelVels)
        assertTrue(coarse < maxVoltage + 0.3, "max wheel voltage: $coarse")

        val fineProfile = generate(TEST_PROFILE_PARAMS.copy(dispResolution = 0.05, angResolution = 0.02))
        val fine = maxWheelVoltage(path, fineProfile, ::wheelVels)
        assertTrue(fine < maxVoltage + 0.1, "max wheel voltage: $fine")

        // the profile actually moves the robot at speed (not degenerate)
        assertTrue(TimeProfile(fineProfile).duration < 10.0)
    }

    @Test
    fun testTankWheelVoltageProfile() {
        val kinematics = TankKinematics(12.0)
        val path = testPath()
        val constraint = kinematics.WheelVoltageConstraint(feedforward, maxVoltage)

        val profile = profile(
            TEST_PROFILE_PARAMS.copy(dispResolution = 0.05, angResolution = 0.02),
            path,
            0.0,
            constraint,
            constraint,
        ).baseProfile

        val max = maxWheelVoltage(path, profile) { txWorldRobot ->
            kinematics.inverse(txWorldRobot.inverse() * txWorldRobot.velocity()).all()
        }
        assertTrue(max < maxVoltage + 0.1, "max wheel voltage: $max")
    }

    @Test
    fun testCentripetalAccelVelConstraint() {
        val maxCentripetalAccel = 15.0
        val path = testPath()

        val profile = profile(
            TEST_PROFILE_PARAMS,
            path,
            0.0,
            MinVelConstraint(
                listOf(
                    TranslationalVelConstraint(60.0),
                    CentripetalAccelVelConstraint(maxCentripetalAccel),
                )
            ),
            ProfileAccelConstraint(-30.0, 30.0),
        ).baseProfile

        var maxObserved = 0.0
        for (s in range(0.0, path.length(), 1000)) {
            val pose = path[s, 3]
            val curvature = sqrt(
                pose.position.x[2] * pose.position.x[2] + pose.position.y[2] * pose.position.y[2]
            )
            val v = profile[s][1]
            maxObserved = maxOf(maxObserved, v * v * curvature)
        }

        // the centripetal limit is respected everywhere (small tolerance for constraint sampling)
        assertTrue(maxObserved < maxCentripetalAccel + 0.5, "max centripetal accel: $maxObserved")
        // and the constraint actually binds somewhere on this path
        assertTrue(maxObserved > 0.9 * maxCentripetalAccel, "max centripetal accel: $maxObserved")
    }
}
