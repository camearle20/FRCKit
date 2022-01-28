package frckit.tools.pathview

import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.util.Units
import java.awt.event.WindowAdapter
import java.awt.event.WindowEvent
import javax.swing.*
import kotlin.system.exitProcess
import frckit.util.GeomUtil
import java.awt.Color

/**
 * Creates a new trajectory visualizer, which can draw and display the ideal following of WPILib differential drive
 * motion profiles
 *
 * @param ppm The pixels per meter value to use for the simulation window
 * @param trajectories A list of one or more trajectories to display.
 * @param trackWidthMeters The track-width of the robot to display, in meters
 * @param markers A list of positions of markers to display on the field.  They appear as 7-inch diameter magenta circles (this is specific to 2021 challenges and is hard-coded).
 */
class TrajectoryVisualizer(val ppm: Double, val trajectories: List<Trajectory>, val trackWidthMeters: Double, val markers: List<TrajectoryMarker> = listOf()) {
    private val frame = JFrame("Trajectory Visualization")

    fun start() {
        frame.addWindowListener(object : WindowAdapter() {
            override fun windowClosing(e: WindowEvent?) {
                exitProcess(0) //Force the application to close (this is needed to stop the executor)
            }
        })

        val canvas = TrajectoryViewCanvas(ppm, Units.inchesToMeters(250.0), Units.inchesToMeters(250.0), trajectories, trackWidthMeters, markers)

        val buttonPanel = JPanel()
        buttonPanel.layout = BoxLayout(buttonPanel, BoxLayout.X_AXIS)

        val playButton = JButton("Simulate")
        playButton.addActionListener { canvas.startSimulation() }
        val resetButton = JButton("Reset")
        resetButton.addActionListener { canvas.resetSimulation() }
        val resetViewButton = JButton("Reset View Offset")
        resetViewButton.addActionListener { canvas.resetView() }
        val zoomInButton = JButton("Zoom In")
        zoomInButton.addActionListener { canvas.zoom(true) }
        val zoomOutButton = JButton("Zoom Out")
        zoomOutButton.addActionListener { canvas.zoom(false) }

        buttonPanel.add(playButton)
        buttonPanel.add(resetButton)
        buttonPanel.add(resetViewButton)
        buttonPanel.add(zoomInButton)
        buttonPanel.add(zoomOutButton)

        frame.layout = BoxLayout(frame.contentPane, BoxLayout.Y_AXIS)

        frame.contentPane.add(canvas)
        frame.contentPane.add(buttonPanel)

        SwingUtilities.invokeLater {
            frame.pack()
            frame.isVisible = true
        }
    }
}

//Testing main function, not for use
/*
fun main() {
    // Create config for trajectory
    // Create config for trajectory
    val config: TrajectoryConfig = TrajectoryConfig(
        Units.inchesToMeters(120.0),
        Units.inchesToMeters(50.0)
    ) // Add kinematics to ensure max speed is actually obeyed
    config.setKinematics(DifferentialDriveKinematics(Units.inchesToMeters(24.0)))
    config.addConstraint(CentripetalAccelerationConstraint(Units.inchesToMeters(150.0)))


    val exampleTrajectory =
        TrajectoryGenerator.generateTrajectory( // Start at the origin facing the +X direction
            GeomUtil.IDENTITY_POSE,  // Pass through these two interior waypoints, making an 's' curve path
            listOf(
                Translation2d(1.0, 1.0),
                Translation2d(2.0, -1.0)
            ),  // End 3 meters straight ahead of where we started, facing forward
            GeomUtil.poseFromTranslation(Translation2d(3.0, 0.0)),
            config
        )

    val otherExampleTrajectory = TrajectoryGenerator.generateTrajectory( // Start at the origin facing the +X direction
        GeomUtil.IDENTITY_POSE,  // Pass through these two interior waypoints, making an 's' curve path
        listOf(
            Translation2d(1.0, 1.0),
            Translation2d(2.0, -1.0)
        ),  // End 3 meters straight ahead of where we started, facing forward
        GeomUtil.poseFromTranslation(Translation2d(3.0, 0.0)),
        config
    ).transformBy(Transform2d(GeomUtil.IDENTITY_POSE, exampleTrajectory.states[exampleTrajectory.states.size - 1].poseMeters))


    //val exampleTrajectory = TrajectoryGenerator.generateTrajectory(start, interiorList, end, config)
    val visualizer = TrajectoryVisualizer(200.0, listOf(exampleTrajectory, otherExampleTrajectory), Units.inchesToMeters(24.0),
        listOf(TrajectoryMarker(Translation2d(1.0, 1.0), Units.inchesToMeters(7.0), Color.ORANGE)))
    visualizer.start()
} */