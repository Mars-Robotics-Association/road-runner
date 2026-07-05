package com.acmerobotics.roadrunner

import org.junit.jupiter.api.Test
import kotlin.math.PI
import kotlin.math.abs
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class YawCouplingFeedforwardTest {
    private val axialFf = MotorFeedforward(0.5, 0.15, 0.02)
    private val lateralFf = MotorFeedforward(0.8, 0.21, 0.03)
    private val anisoFf = AnisotropicMotorFeedforward(axialFf, lateralFf)

    // exaggerated curl (~10% of kV) so budget violations are unmistakable
    private val coupling = YawCouplingFeedforward(0.4, 0.015, 0.3, 0.02)
    private val maxVoltage = 12.0

    // curvy path driven tangentially, so the chassis velocity is purely axial
    private fun forwardPath(): PosePath = TangentPath(
        PositionPathSeqBuilder(
            Vector2d(0.0, 0.0),
            Rotation2d.exp(0.0),
            1e-6,
        )
            .splineTo(
                Vector2d(30.0, 30.0),
                Rotation2d.exp(PI / 2),
            )
            .build()
            .first(),
        0.0,
    )

    // constant-heading path whose tangent stays inside the first quadrant, so both chassis velocity components
    // remain positive along the whole path and both coupling terms are exercised without sign crossings
    private fun diagonalPath(): PosePath {
        val posPath = PositionPathSeqBuilder(
            Vector2d(0.0, 0.0),
            Rotation2d.exp(PI / 4),
            1e-6,
        )
            .splineTo(
                Vector2d(40.0, 35.0),
                Rotation2d.exp(PI / 3),
            )
            .build()
            .first()

        return HeadingPosePath(posPath, ConstantHeadingPath(Rotation2d.exp(0.0), posPath.length()))
    }

    @Test
    fun testComputeRegimes() {
        assertEquals(0.0, coupling.compute(0.0, 0.0), 1e-9)
        assertEquals(0.4 + 0.015 * 30.0, coupling.compute(30.0, 0.0), 1e-9)
        assertEquals(-(0.4 + 0.015 * 30.0), coupling.compute(-30.0, 0.0), 1e-9)
        assertEquals(0.3 + 0.02 * 20.0, coupling.compute(0.0, 20.0), 1e-9)
        assertEquals(-(0.3 + 0.02 * 20.0), coupling.compute(0.0, -20.0), 1e-9)
        assertEquals(
            coupling.compute(30.0, 0.0) + coupling.compute(0.0, -20.0),
            coupling.compute(30.0, -20.0),
            1e-9,
        )
    }

    @Test
    fun testCorrectionIsPureYaw() {
        val kinematics = MecanumKinematics(12.0, 1.1)
        val vel = PoseVelocity2d(Vector2d(30.0, -10.0), 2.0)
        val voltages = kinematics.yawCouplingVoltages(coupling, vel)

        // interpret the corrections as wheel motions and map them back through the forward kinematics
        val twist = kinematics.forward(
            MecanumKinematics.WheelIncrements(
                DualNum.constant<Time>(voltages[0], 1),
                DualNum.constant(voltages[1], 1),
                DualNum.constant(voltages[2], 1),
                DualNum.constant(voltages[3], 1),
            )
        )

        assertEquals(0.0, twist.line.x[0], 1e-9)
        assertEquals(0.0, twist.line.y[0], 1e-9)
        // positive yaw-mode voltage demands a counterclockwise moment
        assertEquals(coupling.compute(30.0, -10.0) / 12.0, twist.angle[0], 1e-9)
    }

    @Test
    fun testZeroCouplingMatchesBaseConstraint() {
        val kinematics = MecanumKinematics(12.0)
        val path = forwardPath()

        fun generate(constraint: MecanumKinematics.WheelVoltageConstraint) =
            profile(TEST_PROFILE_PARAMS, path, 0.0, constraint, constraint).baseProfile

        val baseProfile = generate(kinematics.WheelVoltageConstraint(anisoFf, maxVoltage))
        val coupledProfile = generate(
            kinematics.WheelVoltageConstraint(anisoFf, YawCouplingFeedforward(0.0, 0.0), maxVoltage)
        )

        assertEquals(baseProfile, coupledProfile)
    }

    private fun maxTotalWheelVoltage(
        kinematics: MecanumKinematics,
        path: PosePath,
        p: DisplacementProfile,
    ): Double {
        val traj = DisplacementTrajectory(path, p)
        var maxWheelVoltage = 0.0
        for (s in range(0.0, path.length(), 1000)) {
            val txWorldRobot = traj[s]
            val vel = txWorldRobot.inverse() * txWorldRobot.velocity()
            val components = kinematics.inverseComponents(vel)
            val corrections = kinematics.yawCouplingVoltages(coupling, vel.value())
            components.axial.all().zip(components.lateral.all()).forEachIndexed { i, (axialVel, lateralVel) ->
                maxWheelVoltage = maxOf(
                    maxWheelVoltage,
                    abs(anisoFf.compute(axialVel, lateralVel) + corrections[i]),
                )
            }
        }
        return maxWheelVoltage
    }

    @Test
    fun testCoupledProfileRespectsBudget() {
        val kinematics = MecanumKinematics(12.0)
        for (path in listOf(forwardPath(), diagonalPath())) {
            val constraint = kinematics.WheelVoltageConstraint(anisoFf, coupling, maxVoltage)
            val profile = profile(
                TEST_PROFILE_PARAMS.copy(dispResolution = 0.05, angResolution = 0.02),
                path,
                0.0,
                constraint,
                constraint,
            ).baseProfile

            val max = maxTotalWheelVoltage(kinematics, path, profile)
            assertTrue(max < maxVoltage + 0.1, "max wheel voltage: $max")

            // the profile actually moves the robot at speed (not degenerate)
            assertTrue(TimeProfile(profile).duration < 15.0)
        }
    }

    @Test
    fun testUncoupledPlanningViolatesBudget() {
        val kinematics = MecanumKinematics(12.0)
        val path = forwardPath()

        // planning without the coupling leaves no headroom for the correction the follower applies
        val constraint = kinematics.WheelVoltageConstraint(anisoFf, maxVoltage)
        val profile = profile(
            TEST_PROFILE_PARAMS.copy(dispResolution = 0.05, angResolution = 0.02),
            path,
            0.0,
            constraint,
            constraint,
        ).baseProfile

        val max = maxTotalWheelVoltage(kinematics, path, profile)
        assertTrue(max > maxVoltage + 0.3, "max wheel voltage: $max")
    }
}
