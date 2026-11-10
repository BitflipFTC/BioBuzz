package org.firstinspires.ftc.teamcode.subsystems

import club.bitflip.utils.hardware.ServoEx
import club.bitflip.utils.normalizeDegrees
import club.bitflip.utils.normalizeRadians
import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.math.Pose
import com.pedropathing.math.Velocity
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Alliance
import kotlin.math.atan2
import kotlin.math.exp

@Configurable
class Turret(hardwareMap: HardwareMap, telemetry: Telemetry) :
    Subsystem(hardwareMap, telemetry) {

    companion object {
        @JvmField
        var USE_SOTM = false

        // Robot acceleration low-pass filter time constant.
        @JvmField
        var ACCEL_FILTER_TIME_CONSTANT = 0.10

        // Ignore acceleration samples after long loop interruptions.
        @JvmField
        var MAX_ACCEL_SAMPLE_INTERVAL = 0.25

        // Software position feedforward gains.
        // Velocity gain: seconds.
        // Acceleration gain: seconds squared.
        @JvmField
        var TURRET_VELOCITY_FEEDFORWARD = 0.05

        @JvmField
        var TURRET_ACCELERATION_FEEDFORWARD = 0.002

        // Prevent excessive feedforward offsets.
        @JvmField
        var MAX_TURRET_FEEDFORWARD = 30.0

        private const val FLIGHT_TIME_ITERATIONS = 8
    }

    private val turret1 = ServoEx("turret1")
    private val turret2 = ServoEx("turret2")

    enum class Hive(val x: Double, val y: Double) {
        RED_AUDIENCE(58.0, 52.0),
        RED_NOT_AUDIENCE(58.0, 92.0),
        BLUE_AUDIENCE(84.0, 52.0),
        BLUE_NOT_AUDIENCE(84.0, 92.0)
    }

    lateinit var audienceHive: Pose
    lateinit var notAudienceHive: Pose

    var alliance: Alliance? = null
        set(newAlliance) {
            field = newAlliance

            if (newAlliance == Alliance.BLUE) {
                audienceHive = Pose(
                    Hive.BLUE_AUDIENCE.x,
                    Hive.BLUE_AUDIENCE.y
                )
                notAudienceHive = Pose(
                    Hive.BLUE_NOT_AUDIENCE.x,
                    Hive.BLUE_NOT_AUDIENCE.y
                )
            } else if (newAlliance == Alliance.RED) {
                audienceHive = Pose(
                    Hive.RED_AUDIENCE.x,
                    Hive.RED_AUDIENCE.y
                )
                notAudienceHive = Pose(
                    Hive.RED_NOT_AUDIENCE.x,
                    Hive.RED_NOT_AUDIENCE.y
                )
            }
        }

    var robotPose = Pose(0.0, 0.0, 0.0)
    var robotVelocity = Velocity(0.0, 0.0, 0.0)

    // Filtered field-relative robot acceleration.
    // Linear acceleration: in/s^2.
    // Angular acceleration: rad/s^2.

    var robotAx = 0.0
        private set

    var robotAy = 0.0
        private set

    var robotAlpha = 0.0
        private set

    // Projectile calibration distance.
    // Accounts for inherited robot velocity.

    var effectiveShotDistance = 32.0
        private set

    // Predicted robot-to-goal distance when the projectile arrives.
    // Accounts for robot velocity and acceleration.

    var distanceToGoal = 32.0
        private set

    var timeInAir = 0.5
        private set

    var predictedImpactX = 0.0
        private set

    var predictedImpactY = 0.0
        private set

    // Desired turret motion, calculated from successive
    // uncompensated target angles.

    var desiredTurretDegrees = 0.0
        private set

    var turretTargetVelocity = 0.0
        private set

    var turretTargetAcceleration = 0.0
        private set

    var feedforwardOffset = 0.0
        private set

    private var previousVelocityX = 0.0
    private var previousVelocityY = 0.0
    private var previousOmega = 0.0

    private var previousSampleTime = 0L
    private var hasPreviousVelocity = false

    private var previousTurretTarget: Double? = null
    private var previousRawVelocity = 0.0

    val turretPose: Pose
        get() = robotPose

    // Servo position.
    var position = 0.0
        set(newPosition) {
            turret1.position = newPosition
            turret2.position = newPosition
            field = newPosition
        }

    // Turret angle in degrees.
    var degrees: Double
        get() = (position - 0.5) * 320.0
        set(newDegrees) {
            if (!newDegrees.isFinite()) return

            var clippedAngle = newDegrees

            while (clippedAngle < -180.0) {
                clippedAngle += 360.0
            }

            while (clippedAngle > 180.0) {
                clippedAngle -= 360.0
            }

            position = (clippedAngle / 320.0 + 0.5).coerceIn(0.05, 0.95)
        }

    val trackGoalCommand: Command = Commands.infinite(::trackGoal)

    // ---------------------------------------------------------
    // FILTERING
    // ---------------------------------------------------------

    private fun filterCoefficient(dt: Double): Double {
        val tau = ACCEL_FILTER_TIME_CONSTANT.coerceAtLeast(0.001)

        return (1.0 - exp(-dt / tau)).coerceIn(0.0, 1.0)
    }

    // ---------------------------------------------------------
    // ROBOT ACCELERATION ESTIMATION
    // ---------------------------------------------------------

    private fun updateAccelerationEstimate(now: Long): Double {
        val vx = robotVelocity.vx
        val vy = robotVelocity.vy
        val omega = robotVelocity.omega

        if (!hasPreviousVelocity) {
            previousVelocityX = vx
            previousVelocityY = vy
            previousOmega = omega
            previousSampleTime = now
            hasPreviousVelocity = true

            return 0.0
        }

        val dt = (now - previousSampleTime) / 1e9

        if (dt <= 0.0 || dt > MAX_ACCEL_SAMPLE_INTERVAL) {
            previousVelocityX = vx
            previousVelocityY = vy
            previousOmega = omega
            previousSampleTime = now

            robotAx = 0.0
            robotAy = 0.0
            robotAlpha = 0.0

            return 0.0
        }

        // Differentiate field-relative velocity.
        val rawAx = (vx - previousVelocityX) / dt
        val rawAy = (vy - previousVelocityY) / dt
        val rawAlpha = (omega - previousOmega) / dt

        // Low-pass filter.
        val filter = filterCoefficient(dt)

        robotAx += filter * (rawAx - robotAx)
        robotAy += filter * (rawAy - robotAy)
        robotAlpha += filter * (rawAlpha - robotAlpha)

        previousVelocityX = vx
        previousVelocityY = vy
        previousOmega = omega
        previousSampleTime = now

        return dt
    }

    // ---------------------------------------------------------
    // ANGLE DIFFERENTIATION
    // ---------------------------------------------------------

    private fun wrappedAngleDifference(
        target: Double,
        previous: Double
    ): Double {
        var difference = target - previous

        while (difference > 180.0) {
            difference -= 360.0
        }

        while (difference <= -180.0) {
            difference += 360.0
        }

        return difference
    }

    // Calculate turret target velocity and acceleration.
    // IMPORTANT: Input is the uncompensated desired angle.

    private fun updateTurretTargetKinematics(
        targetDegrees: Double,
        dt: Double
    ) {
        val previous = previousTurretTarget

        if (previous == null || dt <= 0.0) {
            previousTurretTarget = targetDegrees
            previousRawVelocity = 0.0
            turretTargetVelocity = 0.0
            turretTargetAcceleration = 0.0
            return
        }

        val filter = filterCoefficient(dt)

        // Desired angular velocity.
        val rawVelocity = wrappedAngleDifference(
            targetDegrees,
            previous
        ) / dt

        val oldVelocity = turretTargetVelocity
        turretTargetVelocity = oldVelocity +
                filter * (rawVelocity - oldVelocity)

        // Desired angular acceleration (derived from raw velocity change over true dt).
        val rawAcceleration = (rawVelocity - previousRawVelocity) / dt
        val oldAcceleration = turretTargetAcceleration

        turretTargetAcceleration = oldAcceleration +
                filter * (rawAcceleration - oldAcceleration)

        previousRawVelocity = rawVelocity
        previousTurretTarget = targetDegrees
    }

    // ---------------------------------------------------------
    // PROJECTILE FLIGHT TIME
    // ---------------------------------------------------------

    private fun calculateFlightTime(
        distance: Double,
        fallback: Double
    ): Double {
        val result = Shooter.timeInAirLookupTable.calculate(distance)

        return if (result.isFinite() && result > 0.0) {
            result
        } else {
            fallback
        }
    }

    // ---------------------------------------------------------
    // MAIN TRACKING
    // ---------------------------------------------------------

    fun trackGoal() {
        if (alliance == null) return

        val dt = updateAccelerationEstimate(System.nanoTime())

        val goalPose = if (robotPose.y() >= 72.0) {
            notAudienceHive
        } else {
            audienceHive
        }

        // 1. Initial flight-time estimate.

        var flightTime = calculateFlightTime(
            goalPose.distance(robotPose),
            timeInAir
        )

        // 2. Refine flight time using SOTM.
        // Eight iterations, as requested.

        if (USE_SOTM) {
            repeat(FLIGHT_TIME_ITERATIONS) {
                val effectiveOrigin = Pose(
                    robotPose.x() + robotVelocity.vx * flightTime,
                    robotPose.y() + robotVelocity.vy * flightTime
                )

                flightTime = calculateFlightTime(
                    goalPose.distance(effectiveOrigin),
                    flightTime
                )
            }
        }

        timeInAir = flightTime

        // 3. Effective projectile shooting origin.
        // The projectile inherits instantaneous launch velocity.
        // Robot acceleration after launch does not affect the ball.

        val effectiveOrigin = Pose(
            robotPose.x() + if (USE_SOTM) robotVelocity.vx * timeInAir else 0.0,
            robotPose.y() + if (USE_SOTM) robotVelocity.vy * timeInAir else 0.0
        )

        effectiveShotDistance = goalPose.distance(effectiveOrigin)

        // 4. Calculate desired field-relative bearing.

        val bearing = normalizeDegrees(
            Math.toDegrees(
                atan2(
                    goalPose.y() - effectiveOrigin.y(),
                    goalPose.x() - effectiveOrigin.x()
                )
            ) + 180.0,
            false
        )

        // The ball is assumed to launch immediately.
        // Use current robot heading.

        val robotHeading = Math.toDegrees(
            normalizeRadians(robotPose.heading(), false)
        )

        // Uncompensated desired turret angle.
        val desiredDegrees = normalizeDegrees(
            bearing - robotHeading,
            false
        )

        desiredTurretDegrees = desiredDegrees

        // 5. Calculate desired turret angular velocity and acceleration.
        // These are calculated BEFORE feedforward is applied.

        updateTurretTargetKinematics(
            desiredDegrees,
            dt
        )

        // 6. Software position feedforward.
        //
        // Command ahead of the desired angle in the direction
        // of movement, anticipating the servo's response.

        val velocityOffset = turretTargetVelocity * TURRET_VELOCITY_FEEDFORWARD
        val accelerationOffset = turretTargetAcceleration * TURRET_ACCELERATION_FEEDFORWARD

        feedforwardOffset = (
                velocityOffset + accelerationOffset
                ).coerceIn(
                -MAX_TURRET_FEEDFORWARD,
                MAX_TURRET_FEEDFORWARD
            )

        val commandedDegrees = desiredDegrees + feedforwardOffset

        degrees = commandedDegrees

        // 7. Predict robot position when projectile arrives.
        //
        // This is NOT the projectile's shooting origin.
        // It is the robot's predicted physical position at impact.

        val t = timeInAir

        val impactX = robotPose.x() + robotVelocity.vx * t + 0.5 * robotAx * t * t
        val impactY = robotPose.y() + robotVelocity.vy * t + 0.5 * robotAy * t * t

        predictedImpactX = impactX
        predictedImpactY = impactY

        // Final requested distance:
        // predicted robot position at impact to the goal.

        distanceToGoal = goalPose.distance(
            Pose(impactX, impactY)
        )
    }

    // ---------------------------------------------------------
    // TELEMETRY
    // ---------------------------------------------------------

    override fun loop() {
        telemetry.addData("Turret desired angle", desiredTurretDegrees)
        telemetry.addData("Turret commanded angle", degrees)

        telemetry.addData("Turret FF offset", feedforwardOffset)

        telemetry.addData(
            "Turret target velocity (deg/s)",
            turretTargetVelocity
        )

        telemetry.addData(
            "Turret target acceleration (deg/s^2)",
            turretTargetAcceleration
        )

        telemetry.addData("Time in air", timeInAir)

        telemetry.addData(
            "Effective shot distance",
            effectiveShotDistance
        )

        telemetry.addData(
            "Impact distance",
            distanceToGoal
        )

        telemetry.addData("Impact X", predictedImpactX)
        telemetry.addData("Impact Y", predictedImpactY)

        telemetry.addData("Robot AX", robotAx)
        telemetry.addData("Robot AY", robotAy)

        telemetry.addData(
            "Robot angular acceleration (deg/s^2)",
            Math.toDegrees(robotAlpha)
        )
    }
}