package frckit.tools.pathview

import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.Trajectory
import frckit.util.GeomUtil
import java.awt.*
import java.awt.event.MouseEvent
import java.awt.event.MouseListener
import java.awt.event.MouseMotionListener
import java.lang.RuntimeException
import java.text.DecimalFormat
import java.util.concurrent.ScheduledThreadPoolExecutor
import java.util.concurrent.TimeUnit
import javax.swing.BorderFactory
import javax.swing.JPanel
import javax.swing.SwingUtilities
import kotlin.math.*

class TrajectoryViewCanvas(var ppm: Double, val fieldWidth: Double, val fieldHeight: Double, trajectories: List<Trajectory>, val trackWidthMeters: Double, val markers: List<TrajectoryMarker>): JPanel(true) {
    private val trajectory: Trajectory

    private var viewOffsetX = 0
    private var viewOffsetY = 0

    private val origPpm = ppm

    fun resetView() {
        viewOffsetX = 0
        viewOffsetY = 0
        ppm = origPpm
        repaint()
    }

    fun zoom(zoomIn: Boolean) {
        if (zoomIn) ppm += 25.0
        else ppm -= 25.0
        if (ppm < 0.0) ppm = 0.0 //Do not allow negative ppm (this inverts the view!)
        repaint()
    }

    private fun horizontalMetersToPixels(meters: Double): Int {
        return (width - (ppm * meters) + viewOffsetX).roundToInt()
    }

    private fun verticalMetersToPixels(meters: Double): Int {
        return (height - (ppm * meters) + viewOffsetY).roundToInt()
    }

    data class TrajectoryStats(val maxVel: Double, val time: Double)

    private fun computeStats(): TrajectoryStats {
        val maxVel = trajectory.states.maxOf { //Maximum of wheel speeds in a single trajectory
            val v = it.velocityMetersPerSecond
            val omega = it.velocityMetersPerSecond * it.curvatureRadPerMeter
            val leftVelocity = abs(v - (trackWidthMeters / 2.0 * omega)) //Forward kinematics
            val rightVelocity = abs(v + (trackWidthMeters / 2.0 * omega))
            max(leftVelocity, rightVelocity)
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

        var latestState = GeomUtil.POSE_ZERO;
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
            if (time > stats.time) {
                time = stats.time
                done = true
            }
            val state = trajectory.sample(time)

            SwingUtilities.invokeLater {
                latestTime = time
                latestState = state.poseMeters
                repaint()
            }

            if (done) throw RuntimeException("not a real exception - ignore") //This is a hack to easily stop the task on the executor
        }
    }

    private var activeSimulation = Simulation(1.0)

    private val stats: TrajectoryStats
    private val pathStroke = BasicStroke(2.0f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL)
    private val leftTrackTransform = Transform2d(Translation2d(0.0, (trackWidthMeters) / 2.0), GeomUtil.ROTATION_ZERO)
    private val rightTrackTransform = Transform2d(Translation2d(0.0, (trackWidthMeters) / -2.0), GeomUtil.ROTATION_ZERO)
    private val frontTransform = Transform2d(Translation2d((trackWidthMeters) / 2.0, 0.0), GeomUtil.ROTATION_ZERO)
    private val backTransform = Transform2d(Translation2d((trackWidthMeters) / -2.0, 0.0), GeomUtil.ROTATION_ZERO)

    private fun drawRobot(g: Graphics2D) {
        val currentPose = activeSimulation.latestState
        val horiz = horizontalMetersToPixels(currentPose.translation.y)
        val vert = verticalMetersToPixels(currentPose.translation.x)

        //Draw the dot on the robot origin

        g.color = Color.BLACK
        g.fillOval(horiz - 3, vert - 3, 6, 6)

        //Draw the robot bounding box
        val robotFrontLeft = currentPose.transformBy(frontTransform).transformBy(leftTrackTransform)
        val robotFrontRight = currentPose.transformBy(frontTransform).transformBy(rightTrackTransform)
        val robotBackLeft = currentPose.transformBy(backTransform).transformBy(leftTrackTransform)
        val robotBackRight = currentPose.transformBy(backTransform).transformBy(rightTrackTransform)

        val frontLeftHoriz = horizontalMetersToPixels(robotFrontLeft.translation.y)
        val frontLeftVert = verticalMetersToPixels(robotFrontLeft.translation.x)
        val frontRightHoriz = horizontalMetersToPixels(robotFrontRight.translation.y)
        val frontRightVert = verticalMetersToPixels(robotFrontRight.translation.x)
        val backLeftHoriz = horizontalMetersToPixels(robotBackLeft.translation.y)
        val backLeftVert = verticalMetersToPixels(robotBackLeft.translation.x)
        val backRightHoriz = horizontalMetersToPixels(robotBackRight.translation.y)
        val backRightVert = verticalMetersToPixels(robotBackRight.translation.x)

        g.drawLine(frontLeftHoriz, frontLeftVert, frontRightHoriz, frontRightVert) //front left -> front right
        g.drawLine(frontRightHoriz, frontRightVert, backRightHoriz, backRightVert) //front right -> back right
        g.drawLine(backRightHoriz, backRightVert, backLeftHoriz, backLeftVert) //back right -> back left
        g.drawLine(backLeftHoriz, backLeftVert, frontLeftHoriz, frontLeftVert) //back left -> front left
    }

    private val fmt = DecimalFormat("0.####")
    private val velGradient = GradientPaint(10f, 60f, Color.GREEN, 130f, 60f, Color.RED)


    private fun drawText(g: Graphics2D) {
        val time = fmt.format(activeSimulation.latestTime)
        val totalTime = fmt.format(trajectory.totalTimeSeconds)
        val state = activeSimulation.latestState

        g.color = Color.BLACK
        g.drawString("time - current: $time s,   total: $totalTime s", 10, 20)
        g.drawString("pose: $state", 10, 40)

        g.paint = velGradient
        g.fillRect(10, 60, 120, 10)

        g.color = Color.BLACK
        g.drawString("${fmt.format(stats.maxVel)} m/s", 135, 70)
    }

    private fun drawMarkers(g: Graphics2D) {
        markers.forEach {
            g.color = it.color
            val offset = ((it.diameterMeters / 2.0) * ppm).roundToInt()
            val size = (it.diameterMeters * ppm).roundToInt()
            g.fillOval(horizontalMetersToPixels(it.position.y) - offset, verticalMetersToPixels(it.position.x) - offset, size, size)
        }
    }

    private fun drawOrigin(g: Graphics2D) {
        g.color = Color.RED

        g.drawLine(
            horizontalMetersToPixels(0.0),
            verticalMetersToPixels(0.0),
            horizontalMetersToPixels(0.0),
            verticalMetersToPixels(0.5)
        )

        g.color = Color.GREEN

        g.drawLine(
            horizontalMetersToPixels(0.0),
            verticalMetersToPixels(0.0),
            horizontalMetersToPixels(0.5),
            verticalMetersToPixels(0.0)
        )
    }

    private fun drawTrajectory(g: Graphics2D) {
        g.stroke = pathStroke
        for (i in 1 until trajectory.states.size) {
            val curState = trajectory.states[i]
            val lastState = trajectory.states[i - 1]

            val curPose = curState.poseMeters //NOT METERS
            val lastPose = lastState.poseMeters

            val curStateLeft = curPose.transformBy(leftTrackTransform)
            val lastStateLeft = lastPose.transformBy(leftTrackTransform)
            val curStateRight = curPose.transformBy(rightTrackTransform)
            val lastStateRight = lastPose.transformBy(rightTrackTransform)

            val curHoriz = horizontalMetersToPixels(curState.poseMeters.translation.y)
            val curVert = verticalMetersToPixels(curState.poseMeters.translation.x)
            val lastHoriz = horizontalMetersToPixels(lastState.poseMeters.translation.y)
            val lastVert = verticalMetersToPixels(lastState.poseMeters.translation.x)

            val curLeftHoriz = horizontalMetersToPixels(curStateLeft.translation.y)
            val curLeftVert = verticalMetersToPixels(curStateLeft.translation.x)
            val lastLeftHoriz = horizontalMetersToPixels(lastStateLeft.translation.y)
            val lastLeftVert = verticalMetersToPixels(lastStateLeft.translation.x)

            val curRightHoriz = horizontalMetersToPixels(curStateRight.translation.y)
            val curRightVert = verticalMetersToPixels(curStateRight.translation.x)
            val lastRightHoriz = horizontalMetersToPixels(lastStateRight.translation.y)
            val lastRightVert = verticalMetersToPixels(lastStateRight.translation.x)

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

    //Mouse state
    private var isDragging = false
    private var dragStartX = 0
    private var dragStartY = 0
    private var dragStartOffsetX = 0
    private var dragStartOffsetY = 0


    //Main rendering function
    override fun paintComponent(g: Graphics) {
        super.paintComponent(g)
        val g2d = g as Graphics2D
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON)
        drawOrigin(g2d)
        drawTrajectory(g2d)
        drawRobot(g2d)
        drawText(g2d)
        drawMarkers(g2d)
    }

    fun startSimulation() {
        activeSimulation.start()
    }

    fun resetSimulation() {
        activeSimulation.reset()
    }

    init {
        preferredSize = Dimension(ceil(fieldWidth * ppm).toInt(), ceil(fieldHeight * ppm).toInt())
        border = BorderFactory.createLineBorder(Color.BLACK, 1)

        val states = arrayListOf<Trajectory.State>()
        var timeOffset = 0.0
        //Populate states array
        trajectories.forEach {
            trajectory ->
            states.addAll(trajectory.states.map {
                Trajectory.State(
                    it.timeSeconds + timeOffset, //Offset the time by the accumulated time of the previous trajectories
                    it.velocityMetersPerSecond,
                    it.accelerationMetersPerSecondSq,
                    it.poseMeters,
                    it.curvatureRadPerMeter
                )
            })
            timeOffset += trajectory.totalTimeSeconds //Add this segment's time to the time offset
        }

        //Create a new trajectory which combines all the sub-trajectories
        trajectory = Trajectory(states)
        stats = computeStats()

        resetSimulation()
        addMouseListener(object : MouseListener {
            override fun mouseClicked(e: MouseEvent?) {

            }

            override fun mousePressed(e: MouseEvent) {
                dragStartX = e.x
                dragStartY = e.y
                dragStartOffsetX = viewOffsetX
                dragStartOffsetY = viewOffsetY
            }

            override fun mouseReleased(e: MouseEvent?) {
            }

            override fun mouseEntered(e: MouseEvent?) {
            }

            override fun mouseExited(e: MouseEvent?) {
            }

        })

        addMouseMotionListener(object : MouseMotionListener {
            override fun mouseDragged(e: MouseEvent) {
                val dx = e.x - dragStartX
                val dy = e.y - dragStartY

                viewOffsetX = dragStartOffsetX + dx
                viewOffsetY = dragStartOffsetY + dy

                repaint()
            }

            override fun mouseMoved(e: MouseEvent?) {
            }

        })
    }
}