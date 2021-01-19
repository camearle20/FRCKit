package frckit.tools.pathview

import edu.wpi.first.wpilibj.trajectory.Trajectory
import java.awt.event.WindowAdapter
import java.awt.event.WindowEvent
import javax.swing.*
import kotlin.system.exitProcess
import edu.wpi.first.wpilibj.geometry.Rotation2d

import edu.wpi.first.wpilibj.geometry.Pose2d

import edu.wpi.first.wpilibj.geometry.Translation2d

import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator

import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig


class TrajectoryVisualizer(val trajectory: Trajectory, val trackWidthMeters: Double) {
    private val frame = JFrame("Trajectory Visualization")

    fun start() {
        frame.addWindowListener(object : WindowAdapter() {
            override fun windowClosing(e: WindowEvent?) {
                exitProcess(0) //Force the application to close (this is needed to stop the executor)
            }
        })

        val canvas = TrajectoryViewCanvas(2, 250.0, 250.0, trajectory, trackWidthMeters)

        val buttonPanel = JPanel()
        buttonPanel.layout = BoxLayout(buttonPanel, BoxLayout.X_AXIS)

        val playButton = JButton("Simulate")
        playButton.addActionListener { canvas.startSimulation() }
        val resetButton = JButton("Reset")
        resetButton.addActionListener { canvas.resetSimulation() }

        buttonPanel.add(playButton)
        buttonPanel.add(resetButton)

        frame.layout = BoxLayout(frame.contentPane, BoxLayout.Y_AXIS)

        frame.contentPane.add(canvas)
        frame.contentPane.add(buttonPanel)

        SwingUtilities.invokeLater {
            frame.pack()
            frame.isVisible = true
        }
    }
}

fun main() {
    // Create config for trajectory
    // Create config for trajectory
    val config: TrajectoryConfig = TrajectoryConfig(
        5.0,
        5.0
    ) // Add kinematics to ensure max speed is actually obeyed

    // An example trajectory to follow.  All units in meters.

    // An example trajectory to follow.  All units in meters.
    val exampleTrajectory = TrajectoryGenerator.generateTrajectory( // Start at the origin facing the +X direction
        Pose2d(0.0, 0.0, Rotation2d(0.0)),  // Pass through these two interior waypoints, making an 's' curve path
        listOf(
            Translation2d(1.0, 1.0),
            Translation2d(2.0, -1.0)
        ),  // End 3 meters straight ahead of where we started, facing forward
        Pose2d(3.0, 0.0, Rotation2d(0.0)),  // Pass config
        config
    )

    val visualizer = TrajectoryVisualizer(exampleTrajectory, 0.5)
    visualizer.start()
}