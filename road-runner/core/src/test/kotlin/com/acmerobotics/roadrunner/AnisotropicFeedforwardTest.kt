package com.acmerobotics.roadrunner

import org.junit.jupiter.api.Test
import kotlin.math.PI
import kotlin.math.abs
import kotlin.random.Random
import kotlin.test.assertEquals
import kotlin.test.assertTrue

// curvy position path driven with constant heading, so the robot strafes through the turns
private fun strafePath(): PosePath {
    val posPath = PositionPathSeqBuilder(
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
        .first()

    return HeadingPosePath(posPath, ConstantHeadingPath(Rotation2d.exp(0.0), posPath.length()))
}

class AnisotropicFeedforwardTest {
    private val axialFf = MotorFeedforward(0.5, 0.15, 0.02)
    private val lateralFf = MotorFeedforward(0.8, 0.21, 0.03)
    private val anisoFf = AnisotropicMotorFeedforward(axialFf, lateralFf)
    private val maxVoltage = 12.0

    @Test
    fun testLinearBlendAcrossWheelReversal() {
        val ff = anisoFf.copy(friction = LinearBlendFriction(2.0))
        for ((speed, fraction) in listOf(-3.0 to -1.0, -1.0 to -0.5, 0.0 to 0.0, 1.0 to 0.5, 3.0 to 1.0)) {
            val axial = 4.0
            val lateral = speed - axial
            val dynamicVoltage = axialFf.kV * axial + lateralFf.kV * lateral + lateralFf.kA
            assertEquals(
                ff.kSBlended(axial, lateral) * fraction + dynamicVoltage,
                ff.compute(axial, 0.0, lateral, 1.0),
                1e-12,
            )
        }
    }

    @Test
    fun testStribeckReversalAndSlidingFriction() {
        val curve = StribeckFriction(stictionSpeed = 2.0, peakRatio = 1.5)
        for (direction in listOf(-1.0, 1.0)) {
            for ((v, volts) in listOf(0.0 to 0.0, 1.0 to 0.75, 2.0 to 1.5, 4.0 to 1.25, 8.0 to 1.0)) {
                assertEquals(direction * volts, curve.voltage(1.0, direction * v), 1e-12)
            }
        }
        val ff = anisoFf.copy(friction = curve)
        // Opposing components leave zero wheel speed, but still require dynamic voltage.
        assertEquals(4 * (axialFf.kV - lateralFf.kV) + axialFf.kA, ff.compute(4.0, 1.0, -4.0, 0.0), 1e-12)
    }

    @Test
    fun testStribeckProfileRespectsBudget() {
        val ff = anisoFf.copy(friction = StribeckFriction(3.0, 3.0))
        val path = strafePath()
        val kinematics = MecanumKinematics(12.0)
        val constraint = kinematics.WheelVoltageConstraint(ff, maxVoltage)
        val planned = profile(
            TEST_PROFILE_PARAMS.copy(dispResolution = 0.05, angResolution = 0.02),
            path, 0.0, constraint, constraint,
        ).baseProfile
        assertTrue(maxAnisotropicWheelVoltage(kinematics, path, planned, ff) < maxVoltage + 0.1)
    }

    @Test
    fun testComputeReducesToIsotropic() {
        val ff = AnisotropicMotorFeedforward(axialFf, axialFf)
        val r = Random(0x5f3759df)
        repeat(100) {
            val axialVel = DualNum<Time>(doubleArrayOf(r.nextDouble(-50.0, 50.0), r.nextDouble(-100.0, 100.0)))
            val lateralVel = DualNum<Time>(doubleArrayOf(r.nextDouble(-50.0, 50.0), r.nextDouble(-100.0, 100.0)))
            assertEquals(
                axialFf.compute(axialVel + lateralVel),
                ff.compute(axialVel, lateralVel),
                1e-9,
            )
        }
    }

    @Test
    fun testComputePureRegimes() {
        val zero = DualNum.constant<Time>(0.0, 2)
        val r = Random(0x2545f491)
        repeat(100) {
            val vel = DualNum<Time>(doubleArrayOf(r.nextDouble(-50.0, 50.0), r.nextDouble(-100.0, 100.0)))
            assertEquals(axialFf.compute(vel), anisoFf.compute(vel, zero), 1e-9)
            assertEquals(lateralFf.compute(vel), anisoFf.compute(zero, vel), 1e-9)
        }
    }

    @Test
    fun testInverseComponentsSumToInverse() {
        val kinematics = MecanumKinematics(12.0, 1.1)
        val r = Random(0x9e3779b9)
        repeat(100) {
            val t = PoseVelocity2dDual<Time>(
                Vector2dDual(
                    DualNum(doubleArrayOf(r.nextDouble(-50.0, 50.0), r.nextDouble(-100.0, 100.0))),
                    DualNum(doubleArrayOf(r.nextDouble(-50.0, 50.0), r.nextDouble(-100.0, 100.0))),
                ),
                DualNum(doubleArrayOf(r.nextDouble(-5.0, 5.0), r.nextDouble(-10.0, 10.0))),
            )

            val combined = kinematics.inverse(t).all()
            val components = kinematics.inverseComponents(t)
            components.axial.all().zip(components.lateral.all()).zip(combined) { (axial, lateral), total ->
                for (i in 0 until total.size()) {
                    assertEquals(total[i], axial[i] + lateral[i], 1e-9)
                }
            }
        }
    }

    @Test
    fun testConstraintReducesToIsotropic() {
        val kinematics = MecanumKinematics(12.0)
        val path = TangentPath(
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

        fun generate(constraint: MecanumKinematics.WheelVoltageConstraint) =
            profile(TEST_PROFILE_PARAMS, path, 0.0, constraint, constraint).baseProfile

        val isoProfile = generate(kinematics.WheelVoltageConstraint(axialFf, maxVoltage))
        val anisoProfile = generate(
            kinematics.WheelVoltageConstraint(AnisotropicMotorFeedforward(axialFf, axialFf), maxVoltage)
        )

        assertEquals(isoProfile, anisoProfile)
    }

    private fun maxAnisotropicWheelVoltage(
        kinematics: MecanumKinematics,
        path: PosePath,
        p: DisplacementProfile,
        ff: AnisotropicMotorFeedforward = anisoFf,
    ): Double {
        val traj = DisplacementTrajectory(path, p)
        var maxWheelVoltage = 0.0
        for (s in range(0.0, path.length(), 1000)) {
            val txWorldRobot = traj[s]
            val components = kinematics.inverseComponents(txWorldRobot.inverse() * txWorldRobot.velocity())
            components.axial.all().zip(components.lateral.all()) { axialVel, lateralVel ->
                maxWheelVoltage = maxOf(maxWheelVoltage, abs(ff.compute(axialVel, lateralVel)))
            }
        }
        return maxWheelVoltage
    }

    @Test
    fun testStrafeProfileRespectsBudget() {
        val kinematics = MecanumKinematics(12.0)
        val path = strafePath()

        val constraint = kinematics.WheelVoltageConstraint(anisoFf, maxVoltage)
        val profile = profile(
            TEST_PROFILE_PARAMS.copy(dispResolution = 0.05, angResolution = 0.02),
            path,
            0.0,
            constraint,
            constraint,
        ).baseProfile

        val max = maxAnisotropicWheelVoltage(kinematics, path, profile)
        assertTrue(max < maxVoltage + 0.1, "max wheel voltage: $max")

        // the profile actually moves the robot at speed (not degenerate)
        assertTrue(TimeProfile(profile).duration < 15.0)
    }

    @Test
    fun testIsotropicPlanningViolatesAnisotropicBudget() {
        val kinematics = MecanumKinematics(12.0)
        val path = strafePath()

        // planning with the forward-calibrated constants everywhere overdrives the strafe segments
        val constraint = kinematics.WheelVoltageConstraint(axialFf, maxVoltage)
        val profile = profile(
            TEST_PROFILE_PARAMS.copy(dispResolution = 0.05, angResolution = 0.02),
            path,
            0.0,
            constraint,
            constraint,
        ).baseProfile

        val max = maxAnisotropicWheelVoltage(kinematics, path, profile)
        assertTrue(max > maxVoltage + 1.0, "max wheel voltage: $max")
    }
}
