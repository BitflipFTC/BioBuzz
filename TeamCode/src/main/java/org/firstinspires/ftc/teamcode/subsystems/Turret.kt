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
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.ceil
import kotlin.math.exp
import kotlin.math.sqrt

@Configurable
class Turret(hardwareMap: HardwareMap, telemetry: Telemetry) :
    Subsystem(hardwareMap, telemetry) {

    companion object {
        @JvmField
        var USE_SOTM = false

        // Time between reading robot state and the ball leaving the shooter.
        // Set to zero because the ball is assumed to launch immediately.
        // Increase this if shots consistently land behind a moving robot.
        @JvmField
        var LAUNCH_LATENCY_SECONDS = 0.0

        // Robot acceleration low-pass filter time constant.
        @JvmField
        var ACCEL_FILTER_TIME_CONSTANT = 0.10

        // Ignore acceleration samples after long loop interruptions.
        @JvmField
        var MAX_ACCEL_SAMPLE_INTERVAL = 0.25

        // Ignore tiny acceleration estimates caused by velocity noise.
        @JvmField
        var LINEAR_ACCEL_DEADBAND = 2.0

        @JvmField
        var ANGULAR_ACCEL_DEADBAND = 0.05

        // Clamp acceleration estimates to physically plausible values.
        @JvmField
        var MAX_LINEAR_ACCEL = 200.0

        @JvmField
        var MAX_ANGULAR_ACCEL = 8.0

        // Distance solver resolution.
        @JvmField
        var SOLVER_GRID_STEP = 0.5

        @JvmField
        var SOLVER_BISECTION_ITERATIONS = 24

        private const val SERVO_RANGE_DEGREES = 320.0
        private const val SERVO_CENTER_POSITION = 0.5
        private const val SERVO_MIN_POSITION = 0.05
        private const val SERVO_MAX_POSITION = 0.95

        private const val TURRET_MIN_DEGREES =
            (SERVO_MIN_POSITION - SERVO_CENTER_POSITION) * SERVO_RANGE_DEGREES
        private const val TURRET_MAX_DEGREES =
            (SERVO_MAX_POSITION - SERVO_CENTER_POSITION) * SERVO_RANGE_DEGREES

        // The turret is mounted facing backwards relative to the robot.
        private const val MOUNT_OFFSET_DEGREES = 180.0
    }

    private val turret1 = ServoEx("turret1", 0.0001)
    private val turret2 = ServoEx("turret2", 0.0001)

    enum class Hive(val x: Double, val y: Double) {
        RED_AUDIENCE(58.0, 54.0),
        RED_NOT_AUDIENCE(58.0, 90.0),
        BLUE_AUDIENCE(84.0, 54.0),
        BLUE_NOT_AUDIENCE(84.0, 90.0)
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
        private set

    var robotVelocity = Velocity(0.0, 0.0, 0.0)
        private set

    // Turret rotation center in the field frame.
    // Currently the turret is centered on the robot, so this equals robotPose.
    // If the turret moves, offset it in updateRobotState().
    var turretPose = Pose(0.0, 0.0, 0.0)

    // Filtered field-relative robot acceleration.
    // Linear acceleration: in/s^2.
    // Angular acceleration: rad/s^2.
    var robotAx = 0.0
        private set

    var robotAy = 0.0
        private set

    var robotAlpha = 0.0
        private set

    // Distance sent to the shooter lookup tables.
    // This is the equivalent stationary distance, not the direct
    // robot-to-goal distance when SOTM is active.
    var shooterDistance = 32.0
        private set

    var timeInAir = 0.5
        private set

    // True when the required distance is outside the shooter table range.
    var solverSaturated = false
        private set

    // True when the required angle is outside the servo travel range.
    var turretClamped = false
        private set

    var desiredTurretDegrees = 0.0
        private set

    var aimBearingDegrees = 0.0
        private set

    var predictedLaunchX = 0.0
        private set

    var predictedLaunchY = 0.0
        private set

    var predictedLaunchVx = 0.0
        private set

    var predictedLaunchVy = 0.0
        private set

    private var previousVelocityX = 0.0
    private var previousVelocityY = 0.0
    private var previousOmega = 0.0

    private var previousSampleTime = 0L
    private var hasPreviousVelocity = false

    // Servo position.
    var position = 0.0
        set(newPosition) {
            turret1.position = newPosition
            turret2.position = newPosition
            field = newPosition
        }

    // Turret angle in degrees.
    var degrees: Double
        get() = (position - SERVO_CENTER_POSITION) * SERVO_RANGE_DEGREES
        set(newDegrees) {
            if (!newDegrees.isFinite()) return

            val clippedAngle = normalizeDegrees(newDegrees, false)

            position = (clippedAngle / SERVO_RANGE_DEGREES + SERVO_CENTER_POSITION)
                .coerceIn(SERVO_MIN_POSITION, SERVO_MAX_POSITION)
        }

    val trackGoalCommand: Command = Commands.infinite(::trackGoal)

    private data class LaunchState(
        val x: Double,
        val y: Double,
        val vx: Double,
        val vy: Double,
        val headingRadians: Double
    )

    private data class ShotSolution(
        val distance: Double,
        val timeInAir: Double,
        val aimX: Double,
        val aimY: Double,
        val saturated: Boolean
    )

    // ---------------------------------------------------------
    // ROBOT STATE
    // ---------------------------------------------------------

    fun updateRobotState(pose: Pose, velocity: Velocity) {
        // Ignore invalid samples so a bad localization frame cannot command
        // a bad turret angle or shooter distance.
        if (!pose.x().isFinite() ||
            !pose.y().isFinite() ||
            !pose.heading().isFinite()
        ) {
            return
        }

        if (!velocity.vx.isFinite() ||
            !velocity.vy.isFinite() ||
            !velocity.omega.isFinite()
        ) {
            return
        }

        robotPose = pose
        robotVelocity = velocity

        // The turret rotates about the robot center. If that changes,
        // apply the offset here; launch prediction accounts for it.
        turretPose = Pose(pose.x(), pose.y(), pose.heading())

        updateAccelerationEstimate(System.nanoTime())
    }

    // ---------------------------------------------------------
    // FILTERING
    // ---------------------------------------------------------

    private fun filterCoefficient(dt: Double): Double {
        val tau = ACCEL_FILTER_TIME_CONSTANT.coerceAtLeast(0.001)

        return (1.0 - exp(-dt / tau)).coerceIn(0.0, 1.0)
    }

    private fun storeVelocitySample(
        vx: Double,
        vy: Double,
        omega: Double,
        now: Long
    ) {
        previousVelocityX = vx
        previousVelocityY = vy
        previousOmega = omega
        previousSampleTime = now
        hasPreviousVelocity = true
    }

    private fun applyAccelerationDeadband(
        value: Double,
        deadband: Double,
        maxAcceleration: Double
    ): Double {
        val limit = maxAcceleration.coerceAtLeast(0.0)
        val limited = value.coerceIn(-limit, limit)
        val threshold = deadband.coerceAtLeast(0.0)

        return if (abs(limited) < threshold) 0.0 else limited
    }

    // ---------------------------------------------------------
    // ROBOT ACCELERATION ESTIMATION
    // ---------------------------------------------------------

    private fun updateAccelerationEstimate(now: Long) {
        val vx = robotVelocity.vx
        val vy = robotVelocity.vy
        val omega = robotVelocity.omega

        if (!hasPreviousVelocity) {
            storeVelocitySample(vx, vy, omega, now)
            return
        }

        val dt = (now - previousSampleTime) / 1e9

        if (dt <= 0.0 || dt > MAX_ACCEL_SAMPLE_INTERVAL) {
            storeVelocitySample(vx, vy, omega, now)

            robotAx = 0.0
            robotAy = 0.0
            robotAlpha = 0.0

            return
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

        robotAx = applyAccelerationDeadband(
            robotAx,
            LINEAR_ACCEL_DEADBAND,
            MAX_LINEAR_ACCEL
        )
        robotAy = applyAccelerationDeadband(
            robotAy,
            LINEAR_ACCEL_DEADBAND,
            MAX_LINEAR_ACCEL
        )
        robotAlpha = applyAccelerationDeadband(
            robotAlpha,
            ANGULAR_ACCEL_DEADBAND,
            MAX_ANGULAR_ACCEL
        )

        storeVelocitySample(vx, vy, omega, now)
    }

    // ---------------------------------------------------------
    // LAUNCH PREDICTION
    // ---------------------------------------------------------

    private fun currentLaunchState(): LaunchState {
        return LaunchState(
            turretPose.x(),
            turretPose.y(),
            0.0,
            0.0,
            normalizeRadians(robotPose.heading(), false)
        )
    }

    private fun predictLaunchState(): LaunchState {
        val latency = LAUNCH_LATENCY_SECONDS.coerceAtLeast(0.0)

        val currentHeading = normalizeRadians(robotPose.heading(), false)
        val omega = robotVelocity.omega
        val alpha = robotAlpha

        val predictedHeading = currentHeading +
                omega * latency +
                0.5 * alpha * latency * latency

        // Turret rotation center relative to the robot center, in the field frame.
        val offsetX = turretPose.x() - robotPose.x()
        val offsetY = turretPose.y() - robotPose.y()

        // Rigid-body velocity and acceleration of the turret point.
        // These reduce to the robot center values when the turret is centered.
        val turretVelocityX = robotVelocity.vx - omega * offsetY
        val turretVelocityY = robotVelocity.vy + omega * offsetX

        val turretAccelerationX = robotAx -
                alpha * offsetY -
                omega * omega * offsetX

        val turretAccelerationY = robotAy +
                alpha * offsetX -
                omega * omega * offsetY

        val launchX = turretPose.x() +
                turretVelocityX * latency +
                0.5 * turretAccelerationX * latency * latency

        val launchY = turretPose.y() +
                turretVelocityY * latency +
                0.5 * turretAccelerationY * latency * latency

        return LaunchState(
            launchX,
            launchY,
            turretVelocityX + turretAccelerationX * latency,
            turretVelocityY + turretAccelerationY * latency,
            predictedHeading
        )
    }

    // ---------------------------------------------------------
    // SHOT SOLVER
    // ---------------------------------------------------------

    private fun tableTime(distance: Double): Double {
        val value = Shooter.timeInAirLookupTable.calculate(distance)

        return if (value.isFinite() && value > 1e-6) value else 0.5
    }

    // Ball ground velocity = robot velocity + launch velocity relative to the robot.
    // To reach the goal in time T, the ground velocity must be (goal - launch) / T.
    // Therefore the required relative launch velocity is:
    //     (goal - launch) / T - robotVelocity
    // The shooter table says distance d produces relative horizontal speed d / T(d).
    // Solve |(goal - launch) / T(d) - robotVelocity| = d / T(d).
    // f(d) is positive when the table speed is too high and negative when it is too low.
    //
    // The tables are empirical and were measured with the 72 mm flywheel, so the
    // flywheel diameter, hood angle, and wheel slip are already baked into d / T(d).
    // Do not replace d / T(d) with the raw flywheel surface speed; that ignores slip
    // and would overestimate the ball speed.
    private fun shotResidual(
        distance: Double,
        dx: Double,
        dy: Double,
        vx: Double,
        vy: Double
    ): Double {
        val time = tableTime(distance)

        val relativeX = dx / time - vx
        val relativeY = dy / time - vy
        val requiredSpeed = sqrt(relativeX * relativeX + relativeY * relativeY)

        return distance / time - requiredSpeed
    }

    private fun buildShotSolution(
        distance: Double,
        dx: Double,
        dy: Double,
        vx: Double,
        vy: Double,
        saturated: Boolean
    ): ShotSolution {
        val time = tableTime(distance)

        var aimX = dx / time - vx
        var aimY = dy / time - vy
        val aimMagnitude = sqrt(aimX * aimX + aimY * aimY)

        if (aimMagnitude > 1e-6) {
            aimX /= aimMagnitude
            aimY /= aimMagnitude
        } else {
            // Robot velocity exactly matches the required ground velocity.
            // Aim directly at the goal as a safe fallback.
            val goalDistance = sqrt(dx * dx + dy * dy)

            if (goalDistance > 1e-6) {
                aimX = dx / goalDistance
                aimY = dy / goalDistance
            } else {
                aimX = 0.0
                aimY = 1.0
            }
        }

        return ShotSolution(distance, time, aimX, aimY, saturated)
    }

    private fun solveShot(
        dx: Double,
        dy: Double,
        vx: Double,
        vy: Double
    ): ShotSolution {
        val minDistance = Shooter.distanceArray.first()
        val maxDistance = Shooter.distanceArray.last()
        val step = SOLVER_GRID_STEP.coerceAtLeast(0.01)
        val steps = ceil((maxDistance - minDistance) / step)
            .toInt()
            .coerceAtLeast(1)

        var previousDistance = minDistance
        var previousResidual = shotResidual(minDistance, dx, dy, vx, vy)
        var bestDistance = minDistance
        var bestAbsoluteResidual = abs(previousResidual)

        var bracketLow = Double.NaN
        var bracketHigh = Double.NaN

        for (i in 1..steps) {
            val distance = if (i == steps) {
                maxDistance
            } else {
                minDistance + i * step
            }

            val residual = shotResidual(distance, dx, dy, vx, vy)

            if (abs(residual) < bestAbsoluteResidual) {
                bestAbsoluteResidual = abs(residual)
                bestDistance = distance
            }

            if (bracketLow.isNaN() && previousResidual * residual <= 0.0) {
                bracketLow = previousDistance
                bracketHigh = distance
            }

            previousDistance = distance
            previousResidual = residual
        }

        if (!bracketLow.isNaN()) {
            var low = bracketLow
            var high = bracketHigh

            if (high - low < 1e-9) {
                return buildShotSolution(low, dx, dy, vx, vy, false)
            }

            var lowResidual = shotResidual(low, dx, dy, vx, vy)

            repeat(SOLVER_BISECTION_ITERATIONS.coerceAtLeast(0)) {
                val middle = 0.5 * (low + high)
                val middleResidual = shotResidual(middle, dx, dy, vx, vy)

                if (lowResidual * middleResidual <= 0.0) {
                    high = middle
                } else {
                    low = middle
                    lowResidual = middleResidual
                }
            }

            return buildShotSolution(
                0.5 * (low + high),
                dx,
                dy,
                vx,
                vy,
                false
            )
        }

        // No sign change means the shot is outside the table range.
        val minResidual = shotResidual(minDistance, dx, dy, vx, vy)
        val maxResidual = shotResidual(maxDistance, dx, dy, vx, vy)

        val saturatedDistance = when {
            minResidual > 0.0 -> minDistance
            maxResidual < 0.0 -> maxDistance
            else -> bestDistance
        }

        return buildShotSolution(
            saturatedDistance,
            dx,
            dy,
            vx,
            vy,
            true
        )
    }

    private fun buildStaticShot(dx: Double, dy: Double): ShotSolution {
        val minDistance = Shooter.distanceArray.first()
        val maxDistance = Shooter.distanceArray.last()

        val rawDistance = sqrt(dx * dx + dy * dy)
        val distance = rawDistance.coerceIn(minDistance, maxDistance)
        val time = tableTime(distance)

        var aimX = dx
        var aimY = dy
        val aimMagnitude = sqrt(aimX * aimX + aimY * aimY)

        if (aimMagnitude > 1e-6) {
            aimX /= aimMagnitude
            aimY /= aimMagnitude
        } else {
            aimX = 0.0
            aimY = 1.0
        }

        return ShotSolution(
            distance,
            time,
            aimX,
            aimY,
            rawDistance !in minDistance..maxDistance
        )
    }

    // ---------------------------------------------------------
    // MAIN TRACKING
    // ---------------------------------------------------------

    fun trackGoal() {
        if (alliance == null) return

        val goalPose = if (robotPose.y() >= 72.0) {
            notAudienceHive
        } else {
            audienceHive
        }

        val launch = if (USE_SOTM) {
            predictLaunchState()
        } else {
            currentLaunchState()
        }

        val dx = goalPose.x() - launch.x
        val dy = goalPose.y() - launch.y

        val solution = if (USE_SOTM) {
            solveShot(dx, dy, launch.vx, launch.vy)
        } else {
            buildStaticShot(dx, dy)
        }

        // Equivalent stationary distance, equal to the distance from the
        // launch point to the virtual goal (goal - robotVelocity * timeInAir).
        shooterDistance = solution.distance
        timeInAir = solution.timeInAir
        solverSaturated = solution.saturated

        aimBearingDegrees = normalizeDegrees(
            Math.toDegrees(atan2(solution.aimY, solution.aimX)),
            false
        )

        val robotHeadingDegrees = Math.toDegrees(
            normalizeRadians(launch.headingRadians, false)
        )

        val desiredDegrees = normalizeDegrees(
            aimBearingDegrees + MOUNT_OFFSET_DEGREES - robotHeadingDegrees,
            false
        )

        desiredTurretDegrees = desiredDegrees
        turretClamped = desiredDegrees !in TURRET_MIN_DEGREES..TURRET_MAX_DEGREES

        predictedLaunchX = launch.x
        predictedLaunchY = launch.y
        predictedLaunchVx = launch.vx
        predictedLaunchVy = launch.vy

        degrees = desiredDegrees
    }

    // ---------------------------------------------------------
    // TELEMETRY
    // ---------------------------------------------------------

    override fun loop() {
        telemetry.addData("SOTM enabled", USE_SOTM)
        telemetry.addData("Turret desired angle", desiredTurretDegrees)
        telemetry.addData("Turret servo angle", degrees)
        telemetry.addData("Turret clamped", turretClamped)

        telemetry.addData("Aim bearing", aimBearingDegrees)
        telemetry.addData("Shooter distance", shooterDistance)
        telemetry.addData("Time in air", timeInAir)
        telemetry.addData("Solver saturated", solverSaturated)

        telemetry.addData("Robot AX", robotAx)
        telemetry.addData("Robot AY", robotAy)
        telemetry.addData(
            "Robot angular acceleration (deg/s^2)",
            Math.toDegrees(robotAlpha)
        )

        telemetry.addData("Predicted launch X", predictedLaunchX)
        telemetry.addData("Predicted launch Y", predictedLaunchY)
        telemetry.addData("Predicted launch VX", predictedLaunchVx)
        telemetry.addData("Predicted launch VY", predictedLaunchVy)
    }
}
