package frckit.tools.pathview

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Transform2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.geometry.Twist2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import frckit.util.Geom
import java.awt.*
import java.lang.RuntimeException
import java.text.DecimalFormat
import java.util.concurrent.ScheduledThreadPoolExecutor
import java.util.concurrent.TimeUnit
import javax.swing.BorderFactory
import javax.swing.JPanel
import javax.swing.SwingUtilities
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.roundToInt
import kotlin.math.roundToLong

class TrajectoryViewCanvas(val ppi: Int, val fieldWidth: Double, val fieldHeight: Double, val trajectory: Trajectory, val trackWidthMeters: Double): JPanel(true) {
    private val M2I = 39.3701

    private fun horizontalInchesToPixels(inches: Double): Int {
        return width - (ppi * inches).roundToInt()
    }

    private fun verticalInchesToPixels(inches: Double): Int {
        return height - (ppi * inches).roundToInt()
    }

    data class TrajectoryStats(val maxVel: Double, val time: Double)

    private fun computeStats(): TrajectoryStats {
        var maxVel = 0.0

        for (i in 0 until trajectory.states.size) {
            val state = trajectory.states[i]
            val dx = state.velocityMetersPerSecond
            val dtheta = state.velocityMetersPerSecond * state.curvatureRadPerMeter
            val leftVelocity = abs(dx - (trackWidthMeters / 2.0 * dtheta))
            val rightVelocity = abs(dx + (trackWidthMeters / 2.0 * dtheta))
            if (leftVelocity > maxVel) maxVel = leftVelocity
            if (rightVelocity > maxVel) maxVel = rightVelocity
        }

        val time = trajectory.totalTimeSeconds

        return TrajectoryStats(maxVel, time)
    }

    private val executor = ScheduledThreadPoolExecutor(1)

    inner class Simulation(val rate: Double) {
        private fun timeSeconds(): Double {
            return System.nanoTime() * 1e-9 * rate
        }

        var startTime = 0.0
        var done = false

        var latestState = Geom.POSE_I;
        var latestTime = 0.0

        fun start() {
            reset()
            startTime = timeSeconds()
            //Run at 60 fps
            executor.scheduleAtFixedRate(::update, 0L, ((1000.0 / 60.0) * 1e6).roundToLong(), TimeUnit.NANOSECONDS)
        }

        fun reset() {
            done = false
            executor.queue.clear()
            SwingUtilities.invokeLater {
                latestTime = 0.0
                latestState = trajectory.states.first().poseMeters
                repaint()
            }
        }

        //Updates the simulation
        private fun update() {
            var time = timeSeconds() - startTime
            if (time > trajectory.totalTimeSeconds) {
                time = trajectory.totalTimeSeconds
                done = true
            }
            val state = trajectory.sample(time)

            SwingUtilities.invokeLater {
                latestTime = time
                latestState = state.poseMeters
                repaint()
            }

            if (done) throw RuntimeException() //This is a hack to easily stop the task on the executor
        }
    }

    private var activeSimulation = Simulation(1.0)

    private val stats = computeStats()
    private val pathStroke = BasicStroke(2.0f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL)
    private val leftTrackTransform = Transform2d(Translation2d(0.0, (trackWidthMeters * M2I) / 2.0), Geom.ROT_I)
    private val rightTrackTransform = Transform2d(Translation2d(0.0, (trackWidthMeters * M2I) / -2.0), Geom.ROT_I)
    private val frontTransform = Transform2d(Translation2d((trackWidthMeters * M2I) / 2.0, 0.0), Geom.ROT_I)
    private val backTransform = Transform2d(Translation2d((trackWidthMeters * M2I) / -2.0, 0.0), Geom.ROT_I)

    private fun drawRobot(g: Graphics2D) {
        val currentPose = Pose2d(activeSimulation.latestState.translation.x * M2I, activeSimulation.latestState.translation.y * M2I, activeSimulation.latestState.rotation)
        val horiz = horizontalInchesToPixels(currentPose.translation.y)
        val vert = verticalInchesToPixels(currentPose.translation.x)

        //Draw the dot on the robot origin

        g.color = Color.BLACK
        g.fillOval(horiz - 3, vert - 3, 6, 6)

        //Draw the robot bounding box
        val robotFrontLeft = currentPose.transformBy(frontTransform).transformBy(leftTrackTransform)
        val robotFrontRight = currentPose.transformBy(frontTransform).transformBy(rightTrackTransform)
        val robotBackLeft = currentPose.transformBy(backTransform).transformBy(leftTrackTransform)
        val robotBackRight = currentPose.transformBy(backTransform).transformBy(rightTrackTransform)

        val frontLeftHoriz = horizontalInchesToPixels(robotFrontLeft.translation.y)
        val frontLeftVert = verticalInchesToPixels(robotFrontLeft.translation.x)
        val frontRightHoriz = horizontalInchesToPixels(robotFrontRight.translation.y)
        val frontRightVert = verticalInchesToPixels(robotFrontRight.translation.x)
        val backLeftHoriz = horizontalInchesToPixels(robotBackLeft.translation.y)
        val backLeftVert = verticalInchesToPixels(robotBackLeft.translation.x)
        val backRightHoriz = horizontalInchesToPixels(robotBackRight.translation.y)
        val backRightVert = verticalInchesToPixels(robotBackRight.translation.x)

        g.drawLine(frontLeftHoriz, frontLeftVert, frontRightHoriz, frontRightVert) //front left -> front right
        g.drawLine(frontRightHoriz, frontRightVert, backRightHoriz, backRightVert) //front right -> back right
        g.drawLine(backRightHoriz, backRightVert, backLeftHoriz, backLeftVert) //back right -> back left
        g.drawLine(backLeftHoriz, backLeftVert, frontLeftHoriz, frontLeftVert) //back left -> front left
    }

    private val fmt = DecimalFormat("0.####")
    private val velGradient = GradientPaint(10f, 60f, Color.GREEN, 130f, 60f, Color.RED)


    private fun drawText(g: Graphics2D) {
        val time = fmt.format(activeSimulation.latestTime)
        val state = activeSimulation.latestState

        g.color = Color.BLACK
        g.drawString("time: $time s", 10, 20)
        g.drawString("pose: $state", 10, 40)

        g.paint = velGradient
        g.fillRect(10, 60, 120, 10)

        g.color = Color.BLACK
        g.drawString("${fmt.format(stats.maxVel)} m/s", 135, 70)
    }

    private fun drawTrajectory(g: Graphics2D) {
        g.stroke = pathStroke
        for (i in 1 until trajectory.states.size) {
            val curState = trajectory.states[i]
            val lastState = trajectory.states[i - 1]

            val curPose = Pose2d(curState.poseMeters.translation.x * M2I, curState.poseMeters.translation.y * M2I, curState.poseMeters.rotation)
            val lastPose = Pose2d(lastState.poseMeters.translation.x * M2I, lastState.poseMeters.translation.y * M2I, lastState.poseMeters.rotation)

            val curStateLeft = curPose.transformBy(leftTrackTransform)
            val lastStateLeft = lastPose.transformBy(leftTrackTransform)
            val curStateRight = curPose.transformBy(rightTrackTransform)
            val lastStateRight = lastPose.transformBy(rightTrackTransform)

            val curHoriz = horizontalInchesToPixels(curState.poseMeters.translation.y)
            val curVert = verticalInchesToPixels(curState.poseMeters.translation.x)
            val lastHoriz = horizontalInchesToPixels(lastState.poseMeters.translation.y)
            val lastVert = verticalInchesToPixels(lastState.poseMeters.translation.x)

            val curLeftHoriz = horizontalInchesToPixels(curStateLeft.translation.y)
            val curLeftVert = verticalInchesToPixels(curStateLeft.translation.x)
            val lastLeftHoriz = horizontalInchesToPixels(lastStateLeft.translation.y)
            val lastLeftVert = verticalInchesToPixels(lastStateLeft.translation.x)

            val curRightHoriz = horizontalInchesToPixels(curStateRight.translation.y)
            val curRightVert = verticalInchesToPixels(curStateRight.translation.x)
            val lastRightHoriz = horizontalInchesToPixels(lastStateRight.translation.y)
            val lastRightVert = verticalInchesToPixels(lastStateRight.translation.x)

            val dx = curState.velocityMetersPerSecond
            val dtheta = curState.velocityMetersPerSecond * curState.curvatureRadPerMeter
            val leftVelocity = abs(dx - (trackWidthMeters / 2.0 * dtheta))
            val rightVelocity = abs(dx + (trackWidthMeters / 2.0 * dtheta))

            val leftWeight = abs(leftVelocity) / stats.maxVel
            val rightWeight = abs(rightVelocity) / stats.maxVel
            val leftColor = Color((255 * leftWeight).roundToInt(), (255 * (1 - leftWeight)).roundToInt(), 0)
            val rightColor = Color((255 * rightWeight).roundToInt(), (255 * (1 - rightWeight)).roundToInt(), 0)
            g.color = Color.BLUE
            g.drawLine(curHoriz, curVert, lastHoriz, lastVert)
            g.color = leftColor
            g.drawLine(curLeftHoriz, curLeftVert, lastLeftHoriz, lastLeftVert)
            g.color = rightColor
            g.drawLine(curRightHoriz, curRightVert, lastRightHoriz, lastRightVert)
        }
    }

    //Main rendering function
    override fun paintComponent(g: Graphics) {
        super.paintComponent(g)
        val g2d = g as Graphics2D
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON)
        drawTrajectory(g2d)
        drawRobot(g2d)
        drawText(g2d)
    }

    fun startSimulation() {
        activeSimulation.start()
    }

    fun resetSimulation() {
        activeSimulation.reset()
    }

    init {
        preferredSize = Dimension(ceil(fieldWidth * ppi).toInt(), ceil(fieldHeight * ppi).toInt())
        border = BorderFactory.createLineBorder(Color.BLACK, 1)
        resetSimulation()
    }
}