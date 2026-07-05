package com.acmerobotics.roadrunner

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt
import kotlin.math.withSign

/**
 * Coefficients of one wheel's feedforward voltage as a function of the profile velocity \(v\) and profile
 * acceleration \(a\): voltage \(= k_{S,signed} + b_v v + b_{v^2} v^2 + b_a a\). The coefficients derive from the
 * wheel velocity per unit profile velocity and its arclength derivative, so at profile velocity \(v\) the wheel
 * velocity is \(u = c v\) and the wheel acceleration is \(\dot{u} = c a + c' v^2\).
 */
private class WheelVoltageTerm(
    @JvmField val kSSigned: Double,
    @JvmField val velCoeff: Double,
    @JvmField val velSqCoeff: Double,
    @JvmField val accelCoeff: Double,
)

private fun wheelVoltageTerm(feedforward: MotorFeedforward, wheelVel: DualNum<Arclength>) =
    WheelVoltageTerm(
        feedforward.kS.withSign(wheelVel[0]),
        feedforward.kV * wheelVel[0],
        feedforward.kA * wheelVel[1],
        feedforward.kA * wheelVel[0],
    )

private fun wheelVoltageTerm(
    feedforward: AnisotropicMotorFeedforward,
    axialVel: DualNum<Arclength>,
    lateralVel: DualNum<Arclength>,
) = WheelVoltageTerm(
    feedforward.kSBlended(axialVel[0], lateralVel[0]).withSign(axialVel[0] + lateralVel[0]),
    feedforward.axial.kV * axialVel[0] + feedforward.lateral.kV * lateralVel[0],
    feedforward.axial.kA * axialVel[1] + feedforward.lateral.kA * lateralVel[1],
    feedforward.axial.kA * axialVel[0] + feedforward.lateral.kA * lateralVel[0],
)

/**
 * Intersects the profile acceleration intervals implied by the voltage budget of each wheel.
 */
private fun minMaxProfileAccelForWheels(
    maxVoltage: Double,
    terms: List<WheelVoltageTerm>,
    vel: Double,
): MinMax {
    var minAccel = Double.NEGATIVE_INFINITY
    var maxAccel = Double.POSITIVE_INFINITY
    for (term in terms) {
        if (abs(term.accelCoeff) < 1e-9) {
            // profile acceleration has no effect on this wheel's voltage
            continue
        }

        // the wheel voltage less the contribution of the profile acceleration
        val partialVoltage = term.kSSigned + term.velCoeff * vel + term.velSqCoeff * vel * vel
        val bound1 = (-maxVoltage - partialVoltage) / term.accelCoeff
        val bound2 = (maxVoltage - partialVoltage) / term.accelCoeff
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
    maxVoltage: Double,
    terms: List<WheelVoltageTerm>,
): Double {
    var maxVel = Double.POSITIVE_INFINITY
    for (term in terms) {
        // find where the cruise voltage kS + b_v v + b_v² v² first leaves [-V, V]
        maxVel = min(
            maxVel,
            smallestPositiveRoot(term.velSqCoeff, term.velCoeff, term.kSSigned - maxVoltage)
        )
        maxVel = min(
            maxVel,
            smallestPositiveRoot(term.velSqCoeff, term.velCoeff, term.kSSigned + maxVoltage)
        )
    }
    return maxVel
}

// signs of the angular column of the mecanum inverse kinematics, in [MecanumKinematics.WheelVelocities] order
private val mecanumYawSigns = doubleArrayOf(-1.0, -1.0, 1.0, 1.0)

// signs of the angular column of the tank inverse kinematics, in [TankKinematics.WheelVelocities] order
private val tankYawSigns = doubleArrayOf(-1.0, 1.0)

private fun requireValidVoltageParams(feedforward: MotorFeedforward, maxVoltage: Double, cruiseFraction: Double) {
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

    /**
     * Wheel velocities decomposed by the chassis motion that produces them: [axial] holds the contribution of
     * axial and angular motion (the wheel rolling in its drive direction), [lateral] the contribution of lateral
     * motion (the strafe component that engages the rollers). Corresponding entries sum to [inverse].
     */
    data class WheelVelocityComponents<Param>(
        @JvmField
        val axial: WheelVelocities<Param>,
        @JvmField
        val lateral: WheelVelocities<Param>,
    )

    /**
     * Like [inverse], but splits each wheel velocity into its axial+angular and lateral components for use with
     * direction-dependent feedforward models (see [AnisotropicMotorFeedforward]).
     */
    fun <Param> inverseComponents(t: PoseVelocity2dDual<Param>) = WheelVelocityComponents(
        WheelVelocities(
            t.linearVel.x - t.angVel * trackWidth,
            t.linearVel.x - t.angVel * trackWidth,
            t.linearVel.x + t.angVel * trackWidth,
            t.linearVel.x + t.angVel * trackWidth,
        ),
        WheelVelocities(
            -t.linearVel.y * lateralMultiplier,
            t.linearVel.y * lateralMultiplier,
            -t.linearVel.y * lateralMultiplier,
            t.linearVel.y * lateralMultiplier,
        ),
    )

    /**
     * Per-wheel voltages realizing the yaw-mode correction of [coupling] at chassis velocity [vel]; add each to
     * the corresponding wheel's base feedforward voltage. Entries follow [WheelVelocities] order and carry the
     * signs of the angular column of [inverse], so the correction is a pure yaw moment (it cancels in the
     * translational rows of [forward]).
     */
    fun yawCouplingVoltages(coupling: YawCouplingFeedforward, vel: PoseVelocity2d): List<Double> {
        val yawVoltage = coupling.compute(vel.linearVel.x, vel.linearVel.y)
        return mecanumYawSigns.map { it * yawVoltage }
    }

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
     *
     * The [AnisotropicMotorFeedforward] form applies separate axial and lateral constants to the corresponding
     * wheel velocity components (see [inverseComponents]), so strafe-heavy path segments are budgeted with the
     * strafe-calibrated constants. The [MotorFeedforward] form applies one set of constants isotropically.
     *
     * The [yawCoupling] correction (see [YawCouplingFeedforward]) is included in each wheel's budgeted voltage
     * with the signs of [yawCouplingVoltages], so a follower applying the correction at runtime stays within the
     * budget the planner assumed.
     */
    inner class WheelVoltageConstraint(
        @JvmField val feedforward: AnisotropicMotorFeedforward,
        @JvmField val yawCoupling: YawCouplingFeedforward,
        @JvmField val maxVoltage: Double,
        @JvmField val cruiseFraction: Double = 0.95,
    ) : VelConstraint, AccelConstraint {
        @JvmOverloads constructor(
            feedforward: AnisotropicMotorFeedforward,
            maxVoltage: Double,
            cruiseFraction: Double = 0.95,
        ) : this(feedforward, YawCouplingFeedforward(0.0, 0.0), maxVoltage, cruiseFraction)

        @JvmOverloads constructor(
            feedforward: MotorFeedforward,
            maxVoltage: Double,
            cruiseFraction: Double = 0.95,
        ) : this(AnisotropicMotorFeedforward(feedforward, feedforward), maxVoltage, cruiseFraction)

        init {
            requireValidVoltageParams(feedforward.axial, maxVoltage, cruiseFraction)
            requireValidVoltageParams(feedforward.lateral, maxVoltage, cruiseFraction)
            require(abs(yawCoupling.kVAxial) < feedforward.axial.kV) {
                "yaw coupling kVAxial (${yawCoupling.kVAxial}) must be smaller in magnitude than " +
                    "axial kV (${feedforward.axial.kV})"
            }
            require(abs(yawCoupling.kVLateral) < feedforward.lateral.kV) {
                "yaw coupling kVLateral (${yawCoupling.kVLateral}) must be smaller in magnitude than " +
                    "lateral kV (${feedforward.lateral.kV})"
            }
            require(
                maxVoltage * cruiseFraction >
                    max(feedforward.axial.kS, feedforward.lateral.kS) +
                    abs(yawCoupling.kSAxial) + abs(yawCoupling.kSLateral)
            ) {
                "maxVoltage ($maxVoltage) times cruiseFraction ($cruiseFraction) must exceed kS plus the " +
                    "yaw coupling kS magnitudes"
            }
        }

        private fun voltageTerms(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): List<WheelVoltageTerm> {
            // the wheel acceleration term requires a second derivative of the pose
            val pose = if (robotPose.position.x.size() >= 3) robotPose else path[s, 3]
            val vel = pose.inverse() * pose.velocity()
            val components = inverseComponents(vel)

            // chassis velocity components scale with the (nonnegative) profile velocity, so the coupling voltage
            // is affine in it: the sign terms join kS and the linear terms join the velocity coefficient
            val axialDir = vel.linearVel.x[0]
            val lateralDir = vel.linearVel.y[0]
            val yawKS = yawCoupling.computeKS(axialDir, lateralDir)
            val yawKV = yawCoupling.kVAxial * axialDir + yawCoupling.kVLateral * lateralDir

            return components.axial.all().zip(components.lateral.all()).mapIndexed { i, (axialVel, lateralVel) ->
                val base = wheelVoltageTerm(feedforward, axialVel, lateralVel)
                WheelVoltageTerm(
                    base.kSSigned + mecanumYawSigns[i] * yawKS,
                    base.velCoeff + mecanumYawSigns[i] * yawKV,
                    base.velSqCoeff,
                    base.accelCoeff,
                )
            }
        }

        override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double) =
            maxProfileVelForWheels(maxVoltage * cruiseFraction, voltageTerms(robotPose, path, s))

        override fun minMaxProfileAccel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): MinMax =
            minMaxProfileAccel(robotPose, path, s, 0.0)

        override fun minMaxProfileAccel(
            robotPose: Pose2dDual<Arclength>,
            path: PosePath,
            s: Double,
            vel: Double
        ): MinMax = minMaxProfileAccelForWheels(maxVoltage, voltageTerms(robotPose, path, s), vel)
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

    /**
     * Per-wheel voltages realizing the yaw-mode correction of [coupling] at chassis velocity [vel]; add each to
     * the corresponding wheel's base feedforward voltage. Entries follow [WheelVelocities] order and carry the
     * signs of the angular column of [inverse], so the correction is a pure yaw moment (it cancels in the
     * translational rows of [forward]). Only the axial coupling constants apply: a tank drive has no lateral
     * motion.
     */
    fun yawCouplingVoltages(coupling: YawCouplingFeedforward, vel: PoseVelocity2d): List<Double> {
        val yawVoltage = coupling.compute(vel.linearVel.x, 0.0)
        return tankYawSigns.map { it * yawVoltage }
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
     *
     * The [yawCoupling] correction (see [YawCouplingFeedforward]) is included in each wheel's budgeted voltage
     * with the signs of [yawCouplingVoltages], so a follower applying the correction at runtime stays within the
     * budget the planner assumed. Its lateral constants must be zero: a tank drive has no lateral motion.
     */
    inner class WheelVoltageConstraint(
        @JvmField val feedforward: MotorFeedforward,
        @JvmField val yawCoupling: YawCouplingFeedforward,
        @JvmField val maxVoltage: Double,
        @JvmField val cruiseFraction: Double = 0.95,
    ) : VelConstraint, AccelConstraint {
        @JvmOverloads constructor(
            feedforward: MotorFeedforward,
            maxVoltage: Double,
            cruiseFraction: Double = 0.95,
        ) : this(feedforward, YawCouplingFeedforward(0.0, 0.0), maxVoltage, cruiseFraction)

        init {
            requireValidVoltageParams(feedforward, maxVoltage, cruiseFraction)
            require(yawCoupling.kSLateral == 0.0 && yawCoupling.kVLateral == 0.0) {
                "yaw coupling lateral constants (${yawCoupling.kSLateral}, ${yawCoupling.kVLateral}) must be " +
                    "zero for a tank drive, which has no lateral motion"
            }
            require(abs(yawCoupling.kVAxial) < feedforward.kV) {
                "yaw coupling kVAxial (${yawCoupling.kVAxial}) must be smaller in magnitude than " +
                    "kV (${feedforward.kV})"
            }
            require(maxVoltage * cruiseFraction > feedforward.kS + abs(yawCoupling.kSAxial)) {
                "maxVoltage ($maxVoltage) times cruiseFraction ($cruiseFraction) must exceed kS plus the " +
                    "yaw coupling kS magnitude"
            }
        }

        private fun voltageTerms(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): List<WheelVoltageTerm> {
            // the wheel acceleration term requires a second derivative of the pose
            val pose = if (robotPose.position.x.size() >= 3) robotPose else path[s, 3]
            val vel = pose.inverse() * pose.velocity()

            // chassis velocity components scale with the (nonnegative) profile velocity, so the coupling voltage
            // is affine in it: the sign terms join kS and the linear terms join the velocity coefficient
            val axialDir = vel.linearVel.x[0]
            val yawKS = yawCoupling.computeKS(axialDir, 0.0)
            val yawKV = yawCoupling.kVAxial * axialDir

            return inverse(vel).all().mapIndexed { i, wheelVel ->
                val base = wheelVoltageTerm(feedforward, wheelVel)
                WheelVoltageTerm(
                    base.kSSigned + tankYawSigns[i] * yawKS,
                    base.velCoeff + tankYawSigns[i] * yawKV,
                    base.velSqCoeff,
                    base.accelCoeff,
                )
            }
        }

        override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double) =
            maxProfileVelForWheels(maxVoltage * cruiseFraction, voltageTerms(robotPose, path, s))

        override fun minMaxProfileAccel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): MinMax =
            minMaxProfileAccel(robotPose, path, s, 0.0)

        override fun minMaxProfileAccel(
            robotPose: Pose2dDual<Arclength>,
            path: PosePath,
            s: Double,
            vel: Double
        ): MinMax = minMaxProfileAccelForWheels(maxVoltage, voltageTerms(robotPose, path, s), vel)
    }
}
