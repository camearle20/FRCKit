package frckit.simulation.simj

import org.ode4j.math.*
import org.ode4j.ode.*
import kotlin.math.sqrt

class Simulation {
    private val world = OdeHelper.createWorld()
    private val space = OdeHelper.createSimpleSpace()
    private val contactGroup = OdeHelper.createJointGroup()

    private val plane = OdeHelper.createPlane(space, 0.0, 0.0, 1.0, 0.0)
    private val body = OdeHelper.createBody(world)
    private val sides = DVector3(1.0, 1.0, 1.0)
    private val mass = OdeHelper.createMass()
    private val box = OdeHelper.createBox(space, sides)

    data class State(val position: DVector3C, val rotation: DQuaternionC)
    var currentState = State(DVector3(), DQuaternion())
        @Synchronized get
        @Synchronized private set

    private val nearCallback = DGeom.DNearCallback { data, o1, o2 ->
        val b1 = o1.body
        val b2 = o2.body

        val contacts = DContactBuffer(20)
        val n = OdeHelper.collide(o1, o2, 20, contacts.geomBuffer)
        for (i in 0 until n) {
            val contact = contacts[i]
            contact.surface.mode = OdeConstants.dContactApprox1 or OdeConstants.dContactBounce
            contact.surface.mu = 1.0
            contact.surface.bounce = .5

            val joint = OdeHelper.createContactJoint(world, contactGroup, contact)
            joint.attach(b1, b2)
        }
    }

    init {
        world.setGravity(0.0, 0.0, -9.81)
        world.contactMaxCorrectingVel = 0.9
        world.contactSurfaceLayer = .001

        body.setPosition(0.0, 0.0, 5.0)


        mass.setBox(0.5, sides)
        mass.adjust(100.0)
        body.mass = mass
        box.body = body

        val q = DQuaternion()
        val s = sqrt(3.0)
        OdeMath.dQFromAxisAndAngle(q, s, s, s , Math.toRadians(45.0))
        body.quaternion = q
    }

    fun update() {
        for (i in 0 until 100) {
            space.collide(null, nearCallback)
            world.step(.0001)
            contactGroup.empty()
        }

        val q = DQuaternion(box.quaternion.get0(), box.quaternion.get1(), box.quaternion.get2(), box.quaternion.get3())

        currentState = State(box.position.clone(), q)
    }
}