package com.acmerobotics.roadrunner

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt
import kotlin.math.withSign

/**
 * Intersects the profile acceleration intervals implied by the voltage budget of each wheel. Each [wheelVels] entry
 * gives the wheel velocity per unit profile velocity and its arclength derivative, so at profile velocity [vel] and
 * profile acceleration \(a\) the wheel velocity is \(u = c v\) and the wheel acceleration is
 * \(\dot{u} = c a + c' v^2\).
 */
private fun minMaxProfileAccelForWheels(
    feedforward: MotorFeedforward,
    maxVoltage: Double,
    wheelVels: List<DualNum<Arclength>>,
    vel: Double,
): MinMax {
    var minAccel = Double.NEGATIVE_INFINITY
    var maxAccel = Double.POSITIVE_INFINITY
    for (wheelVel in wheelVels) {
        val c = wheelVel[0]
        val cPrime = wheelVel[1]
        if (abs(c) < 1e-9) {
            // profile acceleration has no effect on a (locally) stationary wheel
            continue
        }

        // the wheel voltage less the contribution of the profile acceleration
        val partialVoltage = feedforward.kS.withSign(c) + feedforward.kV * c * vel +
            feedforward.kA * cPrime * vel * vel
        val bound1 = (-maxVoltage - partialVoltage) / (feedforward.kA * c)
        val bound2 = (maxVoltage - partialVoltage) / (feedforward.kA * c)
        minAccel = max(minAccel, min(bound1, bound2))
        maxAccel = min(maxAccel, max(bound1, bound2))
    }
    return MinMax(minAccel, maxAccel)
}

/**
 * Computes the highest profile velocity at which every wheel can at least cruise (sustain zero profile acceleration)
 * within the voltage budget. Above this speed no achievable profile acceleration satisfies the budget, so it is the
 * velocity-constraint counterpart of [minMaxProfileAccelForWheels].
 */
private fun maxProfileVelForWheels(
    feedforward: MotorFeedforward,
    maxVoltage: Double,
    wheelVels: List<DualNum<Arclength>>,
): Double {
    var maxVel = Double.POSITIVE_INFINITY
    for (wheelVel in wheelVels) {
        val c = wheelVel[0]
        val cPrime = wheelVel[1]
        val kSSigned = feedforward.kS.withSign(c)
        // find where the cruise voltage kS + kV c v + kA c' v² first leaves [-V, V]
        maxVel = min(
            maxVel,
            smallestPositiveRoot(feedforward.kA * cPrime, feedforward.kV * c, kSSigned - maxVoltage)
        )
        maxVel = min(
            maxVel,
            smallestPositiveRoot(feedforward.kA * cPrime, feedforward.kV * c, kSSigned + maxVoltage)
        )
    }
    return maxVel
}

/**
 * Returns the smallest positive root of \(a x^2 + b x + c\), or positive infinity if there is none.
 */
private fun smallestPositiveRoot(a: Double, b: Double, c: Double): Double {
    if (a == 0.0) {
        if (b == 0.0) {
            return Double.POSITIVE_INFINITY
        }
        val root = -c / b
        return if (root > 0.0) root else Double.POSITIVE_INFINITY
    }

    val disc = b * b - 4 * a * c
    if (disc < 0.0) {
        return Double.POSITIVE_INFINITY
    }

    val q = -0.5 * (b + sqrt(disc).withSign(b))
    if (q == 0.0) {
        return Double.POSITIVE_INFINITY
    }

    var minRoot = Double.POSITIVE_INFINITY
    val root1 = q / a
    if (root1 > 0.0) {
        minRoot = min(minRoot, root1)
    }
    val root2 = c / q
    if (root2 > 0.0) {
        minRoot = min(minRoot, root2)
    }
    return minRoot
}

/**
 * @param[trackWidth] distance between wheels on opposite sides; see the diagram below
 * ![Wheelbase and track width diagram](https://upload.wikimedia.org/wikipedia/commons/5/52/Wheelbase_and_Track.png)
 * @param[lateralMultiplier] factor that multiplies strafe velocity to compensate for slip; increase it to boost the
 * distance traveled in the strafe direction
 */
data class MecanumKinematics @JvmOverloads constructor(
    @JvmField
    val trackWidth: Double,
    @JvmField
    val lateralMultiplier: Double = 1.0
) {
    /**
     * @param[wheelbase] distance between wheels on the same side; see the diagram in [MecanumKinematics]
     */
    constructor(
        trackWidth: Double,
        wheelbase: Double,
        lateralMultiplier: Double = 1.0
    ) : this((trackWidth + wheelbase) / 2, lateralMultiplier)

    data class WheelIncrements<Param>(
        @JvmField
        val leftFront: DualNum<Param>,
        @JvmField
        val leftBack: DualNum<Param>,
        @JvmField
        val rightBack: DualNum<Param>,
        @JvmField
        val rightFront: DualNum<Param>,
    )

    fun <Param> forward(w: WheelIncrements<Param>) = Twist2dDual(
        Vector2dDual(
            (w.leftFront + w.leftBack + w.rightBack + w.rightFront) * 0.25,
            (-w.leftFront + w.leftBack - w.rightBack + w.rightFront) * (0.25 / lateralMultiplier),
        ),
        (-w.leftFront - w.leftBack + w.rightBack + w.rightFront) * (0.25 / trackWidth),
    )

    data class WheelVelocities<Param>(
        @JvmField
        val leftFront: DualNum<Param>,
        @JvmField
        val leftBack: DualNum<Param>,
        @JvmField
        val rightBack: DualNum<Param>,
        @JvmField
        val rightFront: DualNum<Param>,
    ) {
        fun all() = listOf(leftFront, leftBack, rightBack, rightFront)
    }

    fun <Param> inverse(t: PoseVelocity2dDual<Param>) = WheelVelocities(
        t.linearVel.x - t.linearVel.y * lateralMultiplier - t.angVel * trackWidth,
        t.linearVel.x + t.linearVel.y * lateralMultiplier - t.angVel * trackWidth,
        t.linearVel.x - t.linearVel.y * lateralMultiplier + t.angVel * trackWidth,
        t.linearVel.x + t.linearVel.y * lateralMultiplier + t.angVel * trackWidth,
    )

    inner class WheelVelConstraint(@JvmField val maxWheelVel: Double) : VelConstraint {
        override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): Double {
            val txRobotWorld = robotPose.value().inverse()
            val robotVelWorld = robotPose.velocity().value()
            val robotVelRobot = txRobotWorld * robotVelWorld
            return inverse(PoseVelocity2dDual.constant<Arclength>(robotVelRobot, 1))
                .all()
                .minOf { abs(maxWheelVel / it.value()) }
        }
    }

    /**
     * @usesMathJax
     *
     * Velocity- and acceleration-limiting constraint derived from the voltage budget of the drive motors. The
     * feedforward voltage \(k_s + k_v u + k_a \dot{u}\) of every wheel is kept within \(\pm\)[maxVoltage],
     * capturing the loss of achievable acceleration at speed due to back-EMF.
     *
     * Use an instance as both the velocity and the acceleration constraint of a trajectory (the velocity limit
     * keeps every wheel below the speed at which the achievable acceleration vanishes, subsuming a
     * [WheelVelConstraint] at the free speed). [feedforward] must map wheel velocities in path units to voltage
     * (in the units of [maxVoltage]) and have positive kA. Plan against a conservative [maxVoltage] (e.g., 11 for
     * a 12-volt battery) to leave headroom for battery sag and feedback corrections.
     *
     * The velocity limit only draws [cruiseFraction] of [maxVoltage] so that some acceleration always remains
     * available; without this reserve, reaching the voltage-limited top speed would take unbounded time.
     */
    inner class WheelVoltageConstraint @JvmOverloads constructor(
        @JvmField val feedforward: MotorFeedforward,
        @JvmField val maxVoltage: Double,
        @JvmField val cruiseFraction: Double = 0.95,
    ) : VelConstraint, AccelConstraint {
        init {
            require(feedforward.kV > 0.0) { "kV (${feedforward.kV}) must be positive" }
            require(feedforward.kA > 0.0) { "kA (${feedforward.kA}) must be positive" }
            require(feedforward.kS >= 0.0) { "kS (${feedforward.kS}) must be non-negative" }
            require(cruiseFraction > 0.0 && cruiseFraction < 1.0) {
                "cruiseFraction ($cruiseFraction) must be in (0, 1)"
            }
            require(maxVoltage * cruiseFraction > feedforward.kS) {
                "maxVoltage ($maxVoltage) times cruiseFraction ($cruiseFraction) must exceed kS (${feedforward.kS})"
            }
        }

        private fun wheelVels(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): List<DualNum<Arclength>> {
            // the wheel acceleration term requires a second derivative of the pose
            val pose = if (robotPose.position.x.size() >= 3) robotPose else path[s, 3]
            return inverse(pose.inverse() * pose.velocity()).all()
        }

        override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double) =
            maxProfileVelForWheels(feedforward, maxVoltage * cruiseFraction, wheelVels(robotPose, path, s))

        override fun minMaxProfileAccel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): MinMax =
            minMaxProfileAccel(robotPose, path, s, 0.0)

        override fun minMaxProfileAccel(
            robotPose: Pose2dDual<Arclength>,
            path: PosePath,
            s: Double,
            vel: Double
        ): MinMax = minMaxProfileAccelForWheels(feedforward, maxVoltage, wheelVels(robotPose, path, s), vel)
    }
}

/**
 * @param[trackWidth] distance between wheels on opposite sides; see the diagram below
 * ![Wheelbase and track width diagram](https://upload.wikimedia.org/wikipedia/commons/5/52/Wheelbase_and_Track.png)
 */
data class TankKinematics(@JvmField val trackWidth: Double) {
    data class WheelIncrements<Param>(
        @JvmField
        val left: DualNum<Param>,
        @JvmField
        val right: DualNum<Param>,
    )

    fun <Param> forward(w: WheelIncrements<Param>) = Twist2dDual(
        Vector2dDual(
            (w.left + w.right) * 0.5,
            DualNum.constant(0.0, w.left.size()),
        ),
        (-w.left + w.right) / trackWidth,
    )

    data class WheelVelocities<Param>(
        @JvmField
        val left: DualNum<Param>,
        @JvmField
        val right: DualNum<Param>,
    ) {
        fun all() = listOf(left, right)
    }

    fun <Param> inverse(t: PoseVelocity2dDual<Param>): WheelVelocities<Param> {
        require(t.linearVel.y.values().all { abs(it) < 1e-6 }) {
            "Tank drive does not support lateral motion"
        }

        return WheelVelocities(
            t.linearVel.x - t.angVel * 0.5 * trackWidth,
            t.linearVel.x + t.angVel * 0.5 * trackWidth,
        )
    }

    // TODO: can probably be made generic, though lack of associated types may pose a difficulty
    inner class WheelVelConstraint(@JvmField val maxWheelVel: Double) : VelConstraint {
        override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): Double {
            val txRobotWorld = robotPose.value().inverse()
            val robotVelWorld = robotPose.velocity().value()
            val robotVelRobot = txRobotWorld * robotVelWorld
            return inverse(PoseVelocity2dDual.constant<Arclength>(robotVelRobot, 1))
                .all()
                .minOf { abs(maxWheelVel / it.value()) }
        }
    }

    /**
     * @usesMathJax
     *
     * Velocity- and acceleration-limiting constraint derived from the voltage budget of the drive motors. The
     * feedforward voltage \(k_s + k_v u + k_a \dot{u}\) of every wheel is kept within \(\pm\)[maxVoltage],
     * capturing the loss of achievable acceleration at speed due to back-EMF.
     *
     * Use an instance as both the velocity and the acceleration constraint of a trajectory (the velocity limit
     * keeps every wheel below the speed at which the achievable acceleration vanishes, subsuming a
     * [WheelVelConstraint] at the free speed). [feedforward] must map wheel velocities in path units to voltage
     * (in the units of [maxVoltage]) and have positive kA. Plan against a conservative [maxVoltage] (e.g., 11 for
     * a 12-volt battery) to leave headroom for battery sag and feedback corrections.
     *
     * The velocity limit only draws [cruiseFraction] of [maxVoltage] so that some acceleration always remains
     * available; without this reserve, reaching the voltage-limited top speed would take unbounded time.
     */
    inner class WheelVoltageConstraint @JvmOverloads constructor(
        @JvmField val feedforward: MotorFeedforward,
        @JvmField val maxVoltage: Double,
        @JvmField val cruiseFraction: Double = 0.95,
    ) : VelConstraint, AccelConstraint {
        init {
            require(feedforward.kV > 0.0) { "kV (${feedforward.kV}) must be positive" }
            require(feedforward.kA > 0.0) { "kA (${feedforward.kA}) must be positive" }
            require(feedforward.kS >= 0.0) { "kS (${feedforward.kS}) must be non-negative" }
            require(cruiseFraction > 0.0 && cruiseFraction < 1.0) {
                "cruiseFraction ($cruiseFraction) must be in (0, 1)"
            }
            require(maxVoltage * cruiseFraction > feedforward.kS) {
                "maxVoltage ($maxVoltage) times cruiseFraction ($cruiseFraction) must exceed kS (${feedforward.kS})"
            }
        }

        private fun wheelVels(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): List<DualNum<Arclength>> {
            // the wheel acceleration term requires a second derivative of the pose
            val pose = if (robotPose.position.x.size() >= 3) robotPose else path[s, 3]
            return inverse(pose.inverse() * pose.velocity()).all()
        }

        override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double) =
            maxProfileVelForWheels(feedforward, maxVoltage * cruiseFraction, wheelVels(robotPose, path, s))

        override fun minMaxProfileAccel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): MinMax =
            minMaxProfileAccel(robotPose, path, s, 0.0)

        override fun minMaxProfileAccel(
            robotPose: Pose2dDual<Arclength>,
            path: PosePath,
            s: Double,
            vel: Double
        ): MinMax = minMaxProfileAccelForWheels(feedforward, maxVoltage, wheelVels(robotPose, path, s), vel)
    }
}
