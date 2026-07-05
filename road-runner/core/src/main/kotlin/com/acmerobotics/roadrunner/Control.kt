package com.acmerobotics.roadrunner

import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.withSign

/**
 * @usesMathJax
 *
 * Kinematic motor feedforward
 *
 * @property[kS] kStatic, \(k_s\)
 * @property[kV] kVelocity, \(k_v\)
 * @property[kA] kStatic, \(k_a\)
 */
data class MotorFeedforward(
    @JvmField
    val kS: Double,
    @JvmField
    val kV: Double,
    @JvmField
    val kA: Double,
) {
    /**
     * @usesMathJax
     *
     * Computes the (normalized) voltage \(k_s \cdot \operatorname{sign}(k_v \cdot v + k_a \cdot a) + k_v \cdot v + k_a \cdot a\).
     *
     * @param[vel] \(v\)
     * @param[accel] \(a\)
     */
    fun compute(vel: Double, accel: Double) = kS.withSign(vel) + kV * vel + kA * accel

    fun compute(vel: DualNum<Time>) = compute(vel[0], vel[1])
}

/**
 * @usesMathJax
 *
 * Direction-dependent motor feedforward for drives whose effective constants differ between axial and lateral
 * chassis motion, as mecanum drives do (roller scrub raises the effective constants when strafing). Wheel velocity
 * is decomposed into the component produced by axial and angular chassis motion and the component produced by
 * lateral motion (see [MecanumKinematics.inverseComponents]); the [axial] constants apply to the former, the
 * [lateral] constants to the latter, computing
 * \(k_{s,blend} + k_{v,ax} u_{ax} + k_{v,lat} u_{lat} + k_{a,ax} \dot{u}_{ax} + k_{a,lat} \dot{u}_{lat}\)
 * with \(k_{s,blend}\) the magnitude-weighted mean of the two kS values.
 *
 * The model reproduces [axial] exactly in pure forward/rotational motion and [lateral] exactly in pure strafing,
 * interpolating between the two calibrated regimes in mixed motion. Angular motion is assigned the axial constants
 * because rotation drives each wheel in its rolling direction; the scrub incurred during rotation is in truth
 * somewhere between the two regimes. With equal [axial] and [lateral] constants, [compute] matches
 * [MotorFeedforward.compute] on the total wheel velocity exactly.
 */
data class AnisotropicMotorFeedforward(
    @JvmField
    val axial: MotorFeedforward,
    @JvmField
    val lateral: MotorFeedforward,
) {
    /**
     * Computes the kS contribution magnitude for a wheel moving with velocity components [axialVel], [lateralVel]:
     * the magnitude-weighted mean of the axial and lateral kS values.
     */
    fun kSBlended(axialVel: Double, lateralVel: Double): Double {
        val magSum = abs(axialVel) + abs(lateralVel)
        return if (magSum < 1e-9) {
            axial.kS
        } else {
            (axial.kS * abs(axialVel) + lateral.kS * abs(lateralVel)) / magSum
        }
    }

    /**
     * Computes the (normalized) voltage for a wheel whose velocity decomposes into [axialVel] (axial and angular
     * chassis motion) and [lateralVel] (lateral chassis motion), with matching acceleration components.
     */
    fun compute(axialVel: Double, axialAccel: Double, lateralVel: Double, lateralAccel: Double) =
        kSBlended(axialVel, lateralVel).withSign(axialVel + lateralVel) +
            axial.kV * axialVel + lateral.kV * lateralVel +
            axial.kA * axialAccel + lateral.kA * lateralAccel

    fun compute(axialVel: DualNum<Time>, lateralVel: DualNum<Time>) =
        compute(axialVel[0], axialVel[1], lateralVel[0], lateralVel[1])
}

/**
 * @usesMathJax
 *
 * Feedforward for the parasitic yaw moment produced by chassis translation: the drivetrain asymmetry (motor
 * constant tolerance, unequal wheel wear, friction varying with the weight distribution) that makes a robot curve
 * left or right under straight open-loop driving. The asymmetry acts like a yaw disturbance proportional to
 * translation speed, so the compensation is a yaw-mode voltage
 * \(k_{s,ax} \operatorname{sign}(v_{ax}) + k_{v,ax} v_{ax} + k_{s,lat} \operatorname{sign}(v_{lat}) + k_{v,lat} v_{lat}\)
 * computed from the *chassis* velocity (unlike [MotorFeedforward] and [AnisotropicMotorFeedforward], which take
 * wheel velocities) and applied to the wheels with the signs of the angular column of the inverse kinematics
 * (see [MecanumKinematics.yawCouplingVoltages]). Positive output demands a counterclockwise yaw moment. Each
 * contribution vanishes with its source velocity: pure strafing incurs no axial-sourced correction and vice
 * versa. The sign terms treat velocities within a small threshold of zero as zero so that numerical residue in a
 * component that is zero by construction (e.g., lateral velocity while driving tangentially) cannot inject a
 * spurious correction.
 *
 * The constants are identifiable with only a body-frame (e.g., dead-wheel) localizer, with no per-wheel
 * encoders: during the forward ramp, regress the parasitic yaw rate \(\omega\) against forward velocity, and
 * scale by the small-signal yaw plant gain \(k_v \cdot \mathrm{trackWidth}\) to convert to voltage. The slope
 * gives [kVAxial] and the intercept [kSAxial]; the lateral ramp gives [kSLateral] and [kVLateral] likewise.
 * Because the coupling is fit from the chassis-level response, it absorbs kinematic error (e.g., wheel toe)
 * along with per-wheel constant mismatch; the constants describe the command-to-motion map, not any single
 * motor.
 */
data class YawCouplingFeedforward @JvmOverloads constructor(
    @JvmField
    val kSAxial: Double,
    @JvmField
    val kVAxial: Double,
    @JvmField
    val kSLateral: Double = 0.0,
    @JvmField
    val kVLateral: Double = 0.0,
) {
    /**
     * Computes the sign-dependent (kS) portion of the correction at chassis velocity ([axialVel], [lateralVel]).
     * Velocity components within `1e-6` of zero contribute nothing.
     */
    fun computeKS(axialVel: Double, lateralVel: Double) =
        kSAxial * thresholdSign(axialVel) + kSLateral * thresholdSign(lateralVel)

    /**
     * Computes the (normalized) yaw-mode voltage compensating the yaw disturbance at chassis velocity
     * ([axialVel], [lateralVel]).
     */
    fun compute(axialVel: Double, lateralVel: Double) =
        computeKS(axialVel, lateralVel) + kVAxial * axialVel + kVLateral * lateralVel

    companion object {
        private fun thresholdSign(x: Double) = if (abs(x) < 1e-6) 0.0 else sign(x)
    }
}

/**
 * Proportional position-velocity controller for a holonomic robot.
 */
class HolonomicController(
    @JvmField
    val axialPosGain: Double,
    @JvmField
    val lateralPosGain: Double,
    @JvmField
    val headingGain: Double,
    @JvmField
    val axialVelGain: Double,
    @JvmField
    val lateralVelGain: Double,
    @JvmField
    val headingVelGain: Double,
) {
    constructor(
        axialPosGain: Double,
        lateralPosGain: Double,
        headingGain: Double,
    ) : this(axialPosGain, lateralPosGain, headingGain, 0.0, 0.0, 0.0)

    /**
     * Computes the velocity and acceleration command. The frame `Target` is the reference robot, and the frame `Actual`
     * is the measured, physical robot.
     *
     * @return velocity command in the actual frame
     */
    fun compute(
        targetPose: Pose2dDual<Time>,
        actualPose: Pose2d,
        actualVelActual: PoseVelocity2d,
    ): PoseVelocity2dDual<Time> {
        // TODO: Are these names useful for anyone else?
        val targetVelWorld = targetPose.velocity()
        val txTargetWorld = Pose2dDual.constant<Time>(targetPose.value().inverse(), 2)
        val targetVelTarget = txTargetWorld * targetVelWorld

        val velErrorActual = targetVelTarget.value() - actualVelActual

        val error = targetPose.value().minusExp(actualPose)
        return targetVelTarget +
            PoseVelocity2d(
                Vector2d(
                    axialPosGain * error.position.x,
                    lateralPosGain * error.position.y,
                ),
                headingGain * error.heading.log(),
            ) +
            PoseVelocity2d(
                Vector2d(
                    axialVelGain * velErrorActual.linearVel.x,
                    lateralVelGain * velErrorActual.linearVel.y,
                ),
                headingVelGain * velErrorActual.angVel,
            )
    }
}

/**
 * @usesMathJax
 *
 * Ramsete controller for tracking tank trajectories.
 *
 * The standard Ramsete control law from equation \((5.12)\) of [this paper](https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf) is
 * \[
 *   \begin{pmatrix}v\\ \omega\end{pmatrix} =
 *   \begin{pmatrix}
 *     v_d \cos (\theta_d - \theta) + 2 \zeta \sqrt{\omega_d^2 + b v_d^2} \Big\lbrack (x_d - x) \cos \theta + (y_d - y) \sin \theta \Big\rbrack\\
 *     \omega_d + b v_d \frac{\sin(\theta_d - \theta)}{\theta_d - \theta} \Big\lbrack (x_d - x) \cos \theta - (y_d - y) \sin \theta \Big\rbrack + 2 \zeta \sqrt{\omega_d^2 + b v_d^2} (\theta_d - \theta)
 *   \end{pmatrix}
 * \]
 * where \(\zeta \in (0, 1)\) and \(b \gt 0\). Since \(b\) has units, we substitute \(b = \frac{\bar{b}}{l^2}\) where
 * \(l\) is the track width.
 */
// defaults taken from https://github.com/wpilibsuite/allwpilib/blob/3fdb2f767d466e00d19e487fdb64d33c22ccc7d5/wpimath/src/main/native/cpp/controller/RamseteController.cpp#L31-L33
// with a track width of 1 meter
class RamseteController @JvmOverloads constructor(
    @JvmField
    val trackWidth: Double,
    @JvmField
    val zeta: Double = 0.7,
    @JvmField
    val bBar: Double = 2.0,
) {
    @JvmField
    val b = bBar / (trackWidth * trackWidth)

    /**
     * Computes the velocity and acceleration command. The frame `Target` is the reference robot, and the frame `Actual`
     * is the measured, physical robot.
     *
     * @return velocity command in the actual frame
     */
    fun compute(
        s: DualNum<Time>,
        targetPose: Pose2dDual<Arclength>,
        actualPose: Pose2d,
    ): PoseVelocity2dDual<Time> {
        val targetHeading = targetPose.heading.value()
        val tangentHeading = targetPose.position.drop(1).value().angleCast()
        val dir = tangentHeading.real * targetHeading.real + tangentHeading.imag * targetHeading.imag
        val vRef = dir * s[1]

        val omegaRef = targetPose.reparam(s).heading.velocity()[0]

        val k = 2.0 * zeta * sqrt(omegaRef * omegaRef + b * vRef * vRef)

        fun sinc(x: Double): Double {
            val u = x + snz(x)
            return sin(u) / u
        }

        val error = targetPose.value().minusExp(actualPose)
        return PoseVelocity2dDual.constant(
            PoseVelocity2d(
                Vector2d(
                    vRef * error.heading.real + k * error.position.x,
                    0.0
                ),
                omegaRef + k * error.heading.log() + b * vRef * sinc(error.heading.log()) * error.position.y,
            ),
            2
        )
    }
}
