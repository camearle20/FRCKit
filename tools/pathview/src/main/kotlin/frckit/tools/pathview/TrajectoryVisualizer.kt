package frckit.tools.pathview

import edu.wpi.first.wpilibj.trajectory.Trajectory
import java.awt.event.WindowAdapter
import java.awt.event.WindowEvent
import javax.swing.*
import kotlin.system.exitProcess
import edu.wpi.first.wpilibj.geometry.Rotation2d

import edu.wpi.first.wpilibj.geometry.Pose2d

import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics

import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator

import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint
import edu.wpi.first.wpilibj.util.Units
import frckit.util.GeomUtil

/**
 * Creates a new trajectory visualizer, which can draw and display the ideal following of WPILib differential drive
 * motion profiles
 *
 * @param ppm The pixels per meter value to use for the simulation window
 * @param trajectory The trajectory object to display
 * @param trackWidthInches The track-width of the robot to display, in inches
 * @param markers A list of positions of markers to display on the field.  They appear as 7-inch diameter magenta circles (this is specific to 2021 challenges and is hard-coded).
 */
class TrajectoryVisualizer(val ppm: Double, val trajectory: Trajectory, val trackWidthMeters: Double, val markers: List<Translation2d> = listOf()) {
    private val frame = JFrame("Trajectory Visualization")

    fun start() {
        frame.addWindowListener(object : WindowAdapter() {
            override fun windowClosing(e: WindowEvent?) {
                exitProcess(0) //Force the application to close (this is needed to stop the executor)
            }
        })

        val canvas = TrajectoryViewCanvas(ppm, Units.inchesToMeters(250.0), Units.inchesToMeters(250.0), trajectory, trackWidthMeters, markers)

        val buttonPanel = JPanel()
        buttonPanel.layout = BoxLayout(buttonPanel, BoxLayout.X_AXIS)

        val playButton = JButton("Simulate")
        playButton.addActionListener { canvas.startSimulation() }
        val resetButton = JButton("Reset")
        resetButton.addActionListener { canvas.resetSimulation() }
        val resetViewButton = JButton("Reset View Offset")
        resetViewButton.addActionListener { canvas.resetView() }

        buttonPanel.add(playButton)
        buttonPanel.add(resetButton)
        buttonPanel.add(resetViewButton)

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


    //val exampleTrajectory = TrajectoryGenerator.generateTrajectory(start, interiorList, end, config)
    val visualizer = TrajectoryVisualizer(100.0, exampleTrajectory, Units.inchesToMeters(24.0), listOf(Translation2d(1.0, 1.0)))
    visualizer.start()
}

