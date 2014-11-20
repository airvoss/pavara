from pavara.constants import *
from pavara.utils.integrator import Integrator, Friction
from pavara.base_objects import *
from pavara.map_objects import Sky, Dome
from pavara.utils.geom import to_cartesian
from panda3d.core import AmbientLight, DirectionalLight, VBase4, Vec3, TransparencyAttrib, CompassEffect, NodePath
from panda3d.bullet import BulletDebugNode, BulletWorld, BulletGhostNode, BulletSphereShape, BulletRigidBodyNode
import math
import random
import string
from collections import namedtuple

ObjectFamily = namedtuple('ObjectFamily', ['np', 'update'])

def null_update(world, node, dt):
    pass

def update_goody(world, node, dt):
    if not node.getPythonTag('active'):
        self.timeout += dt
        if self.timeout > self.respawn:
            node.setPythonTag['active'] = True
            node.show()
            self.timeout = 0
        return
    spin_bone = node.getPythonTag('spin_bone')
    spin = node.getPythonTag('spin')
    if spin_bone:
        spin_bone.set_hpr(spin_bone, spin[2]*dt, spin[1]*dt, spin[0]*dt)
    else:
        node.set_hpr(node, *[x * dt for x in spin])
    result = world.physics.contact_test(node.node())
    for contact in result.getContacts():
        node_1 = contact.getNode0()
        node_2 = contact.getNode1()
        if "Walker" in node_2.get_name():
           # TODO: identify which player and credit them with the items.
           node.setPythonTag('active', False)
           node.hide()


def update_walker(world, node, dt):
    dt = min(dt, 0.2) # let's just temporarily assume that if we're getting less than 5 fps, dt must be wrong.
    yaw = self.movement['left'] + self.movement['right']
    self.rotate_by(yaw * dt * 60, 0, 0)
    walk = self.movement['forward'] + self.movement['backward']
    start = self.position()
    cur_pos_ts = TransformState.make_pos(self.position() + self.head_height)

    if self.on_ground:
        friction = DEFAULT_FRICTION
    else:
        friction = AIR_FRICTION

    #to debug walk cycle (stay in place)
    #riction = 0

    speed = walk
    pos = self.position()
    self.move_by(0, 0, speed)
    direction = self.position() - pos
    newpos, self.xz_velocity = Friction(direction, friction).integrate(pos, self.xz_velocity, dt)
    self.move(newpos)

    # Cast a ray from just above our feet to just below them, see if anything hits.
    pt_from = self.position() + Vec3(0, 1, 0)
    pt_to = pt_from + Vec3(0, -1.1, 0)
    result = self.world.physics.ray_test_closest(pt_from, pt_to, MAP_COLLIDE_BIT | SOLID_COLLIDE_BIT)

    # this should return 'on ground' information
    self.skeleton.update_legs(walk, dt, self.world.scene, self.world.physics)

    if self.y_velocity.get_y() <= 0 and result.has_hit():
        self.on_ground = True
        self.crouch_impulse = self.y_velocity.y
        self.y_velocity = Vec3(0, 0, 0)
        self.move(result.get_hit_pos())
        self.skeleton.left_leg_on_ground = True
        self.skeleton.right_leg_on_ground = True
    else:
        self.on_ground = False
        current_y = Point3(0, self.position().get_y(), 0)
        y, self.y_velocity = self.integrator.integrate(current_y, self.y_velocity, dt)
        self.move(self.position() + (y - current_y))

    if self.crouching and self.skeleton.crouch_factor < 1:
        self.skeleton.crouch_factor += (dt*60)/10
        self.skeleton.update_legs(0, dt, self.world.scene, self.world.physics)
    elif not self.crouching and self.skeleton.crouch_factor > 0:
        self.skeleton.crouch_factor -= (dt*60)/10
        self.skeleton.update_legs(0, dt, self.world.scene, self.world.physics)

    #if self.crouch_impulse < 0:

    goal = self.position()
    adj_dist = abs((start - goal).length())
    new_pos_ts = TransformState.make_pos(self.position() + self.head_height)

    sweep_result = self.st_result(cur_pos_ts, new_pos_ts)
    count = 0
    while sweep_result.has_hit() and count < 10:
        moveby = sweep_result.get_hit_normal()
        self.xz_velocity = -self.xz_velocity.cross(moveby).cross(moveby)
        moveby.normalize()
        moveby *= adj_dist * (1 - sweep_result.get_hit_fraction())
        self.move(self.position() + moveby)
        new_pos_ts = TransformState.make_pos(self.position() + self.head_height)
        sweep_result = self.st_result(cur_pos_ts, new_pos_ts)
        count += 1

    if self.energy > WALKER_MIN_CHARGE_ENERGY:
        if self.left_gun_charge < 1:
            self.energy -= WALKER_ENERGY_TO_GUN_CHARGE[0]
            self.left_gun_charge += WALKER_ENERGY_TO_GUN_CHARGE[1]
        else:
            self.left_gun_charge = math.floor(self.left_gun_charge)

        if self.right_gun_charge < 1:
            self.energy -= WALKER_ENERGY_TO_GUN_CHARGE[0]
            self.right_gun_charge += WALKER_ENERGY_TO_GUN_CHARGE[1]
        else:
            self.right_gun_charge = math.floor(self.right_gun_charge)

    if self.energy < 1:
        self.energy += WALKER_RECHARGE_FACTOR * (dt)

    if self.player:
        self.sights.update(self.left_barrel_joint, self.right_barrel_joint)

class World (object):
    """
    The World models basically everything about a map, including gravity, ambient light, the sky, and all map objects.
    """

    def __init__(self, camera, debug=False, audio3d=None, client=None, server=None):

        self.incarnators = []


        self.collidables = set()

        self.garbage = set()
        self.scene = NodePath('world')
        mapnp = NodePath('map_objects')
        mapnp.reparentTo(self.scene)
        walkers = NodePath('walkers')
        walkers.reparentTo(self.scene)
        goodies = NodePath('goodies')
        goodies.reparentTo(self.scene)
        self.families = { 'map_object': ObjectFamily(np=mapnp, update=null_update),
                          'goody': ObjectFamily(np=goodies, update=update_goody),
                          'walker': ObjectFamily(np=walkers, update=update_walker),
                        }


        # Set up the physics world. TODO: let maps set gravity.
        self.gravity = DEFAULT_GRAVITY
        self.physics = BulletWorld()
        self.physics.set_gravity(self.gravity)

        self.debug = debug

        if debug:
            debug_node = BulletDebugNode('Debug')
            debug_node.show_wireframe(True)
            debug_node.show_constraints(True)
            debug_node.show_bounding_boxes(False)
            debug_node.show_normals(False)
            np = self.scene.attach_new_node(debug_node)
            np.show()
            self.physics.set_debug_node(debug_node)


    def get_incarn(self):
        return random.choice(self.incarnators)


    def attach(self, obj):
        obj.world = self
        node = None
        solid = None
        if hasattr(obj, 'create_node'):
            node = obj.create_node()
        if hasattr(obj, 'create_solid'):
            solid = obj.create_solid()
        family = obj.get_family()
        parent = self.families[family]
        if solid:
            if isinstance(solid, BulletRigidBodyNode):
                self.physics.attach_rigid_body(solid)
            elif isinstance(solid, BulletGhostNode):
                self.physics.attach_ghost(solid)
        if node:
            if solid:
                # If this is a solid visible object, create a new physics
                # node and reparent the visual node to that.
                phys_node = parent.np.attach_new_node(solid)
                node.reparent_to(phys_node)
                node = phys_node
            else:
                # Otherwise just reparent the visual node to the root.
                node.reparent_to(parent.np)
            node.setTag('type', family)
        elif solid:
            node = parent.np.attach_new_node(solid)
        if solid and obj.collide_bits is not None:
            solid.set_into_collide_mask(obj.collide_bits)

        # Let the object know it has been attached.
        obj.attached(node)
        return obj

    def attach_incarnator(self, incarn):
        incarn.world = self
        self.incarnators.append(incarn)


    def create_hector(self, name=None):
        # TODO: get random incarn, start there
        h = self.attach(Hector(name))
        h.move((0, 15, 0))
        return h

    def register_updater(self, obj):
        #self.updatables.add(obj)
        pass

    def register_updater_later(self, obj):
        #self.updatables_to_add.add(obj)
        pass



    def update(self, task):
        """
        Called every frame to update the physics, etc.
        """
        dt = globalClock.getDt()
        self.collidables -= self.garbage
        for family in self.families.values():
            for np in family.np.getChildren():
                family.update(self, np, dt)
        while True:
            if len(self.garbage) < 1:
                break;
            trash = self.garbage.pop()
            if(isinstance(trash.solid, BulletGhostNode)):
                self.physics.remove_ghost(trash.solid)
            if(isinstance(trash.solid, BulletRigidBodyNode)):
                self.physics.remove_rigid_body(trash.solid)
            if hasattr(trash, 'dead'):
                trash.dead()
            trash.node.remove_node()
            del(trash)
        self.physics.do_physics(dt)
        for obj in self.collidables:
            result = self.physics.contact_test(obj.node.node())
            for contact in result.get_contacts():
                obj1 = self.objects.get(contact.get_node0().get_name())
                obj2 = self.objects.get(contact.get_node1().get_name())
                if obj1 and obj2:
                    # Check the collision bits to see if the two objects should collide.
                    should_collide = obj1.collide_bits & obj2.collide_bits
                    if not should_collide.is_zero():
                        pt = contact.get_manifold_point()
                        if obj1 in self.collidables:
                            obj1.collision(obj2, pt, True)
                        if obj2 in self.collidables:
                            obj2.collision(obj1, pt, False)
        return task.cont

class ServerWorld(World):
    """
    The server's view of the world, sans any purely visual information.
    """
    def __init__(self, debug=False):
        super(ServerWorld, self).__init__(debug)



    def register_collider(self, obj):
        assert isinstance(obj, PhysicalObject)
        self.collidables.add(obj)

    def do_explosion(self, node, radius, force):
        center = node.get_pos(self.scene);
        expl_body = BulletGhostNode("expl")
        expl_shape = BulletSphereShape(radius)
        expl_body.add_shape(expl_shape)
        expl_bodyNP = self.scene.attach_new_node(expl_body)
        expl_bodyNP.set_pos(center)
        self.physics.attach_ghost(expl_body)
        result = self.physics.contact_test(expl_body)
        for contact in result.getContacts():
            n0_name = contact.getNode0().get_name()
            n1_name = contact.getNode1().get_name()
            obj = None
            try:
                obj = self.objects[n1_name]
            except:
                break
            if n0_name == "expl" and n1_name not in EXPLOSIONS_DONT_PUSH and not n1_name.startswith('Walker'):
                # repeat contact test with just this pair of objects
                # otherwise all manifold point values will be the same
                # for all objects in original result
                real_c = self.physics.contact_test_pair(expl_body, obj.solid)
                mpoint = real_c.getContacts()[0].getManifoldPoint()
                distance = mpoint.getDistance()
                if distance < 0:
                    if hasattr(obj, 'decompose'):
                        obj.decompose()
                    else:
                        expl_vec = Vec3(mpoint.getPositionWorldOnA() - mpoint.getPositionWorldOnB())
                        expl_vec.normalize()
                        magnitude = force * 1.0/math.sqrt(abs(radius - abs(distance)))
                        obj.solid.set_active(True)
                        obj.solid.apply_impulse(expl_vec*magnitude, mpoint.getLocalPointB())
                    if hasattr(obj, 'damage'):
                        obj.damage(magnitude/5)
        self.physics.remove_ghost(expl_body)
        expl_bodyNP.detach_node()
        del(expl_body, expl_bodyNP)


    def do_plasma_push(self, plasma, node, energy):
        obj = None
        try:
            obj = self.objects[node]
        except:
            raise

        if node not in EXPLOSIONS_DONT_PUSH and not node.startswith('Walker'):
            if hasattr(obj, 'decompose'):
                obj.decompose()
            else:
                solid = obj.solid
                dummy_node = NodePath('tmp')
                dummy_node.set_hpr(plasma.hpr)
                dummy_node.set_pos(plasma.pos)
                f_vec = scene.get_relative_vector(dummy_node, Vec3(0,0,1))
                local_point = (obj.node.get_pos() - dummy_node.get_pos()) *-1
                f_vec.normalize()
                solid.set_active(True)
                try:
                    solid.apply_impulse(f_vec*(energy*35), Point3(local_point))
                except:
                    pass
                del(dummy_node)
        if hasattr(obj, 'damage'):
            obj.damage(energy*5)


class ClientWorld(World):
    """
    A client's view of the world, including visual information.
    Leaves most physics and game logic to the server to dictate.
    """
    def __init__(self, camera, debug=False, audio3d=False):
        super(ClientWorld, self).__init__(debug)
        self.camera = camera
        self.audio3d = audio3d
        self.ambient = self._make_ambient()
        self.celestials = CompositeObject()
        self.sky = self.attach(Sky())

    def _make_ambient(self):
        alight = AmbientLight('ambient')
        alight.set_color(VBase4(*DEFAULT_AMBIENT_COLOR))
        node = self.scene.attach_new_node(alight)
        self.scene.set_light(node)
        return node

    def set_ambient(self, color):
        """
        Sets the ambient light to the given color.
        """
        self.ambient.node().set_color(VBase4(*color))

    def create_celestial_node(self):
        bounds = self.camera.node().get_lens().make_bounds()
        self.celestials = self.celestials.create_node()
        self.celestials.set_transparency(TransparencyAttrib.MAlpha)
        self.celestials.set_light_off()
        self.celestials.set_effect(CompassEffect.make(self.camera, CompassEffect.PPos))
        self.celestials.node().set_bounds(bounds)
        self.celestials.node().set_final(True)
        self.celestials.reparent_to(self.scene)

    def register_collider(self, obj):
        pass

    def add_celestial(self, azimuth, elevation, color, intensity, radius, visible):
        """
        Adds a celestial light source to the scene. If it is a visible celestial, also add a sphere model.
        """
        if not self.camera:
            return
        location = Vec3(to_cartesian(azimuth, elevation, 1000.0 * 255.0 / 256.0))
        if intensity:
            dlight = DirectionalLight('celestial')
            dlight.set_color((color[0] * intensity, color[1] * intensity,
                color[2] * intensity, 1.0))
            node = self.scene.attach_new_node(dlight)
            node.look_at(*(location * -1))
            self.scene.set_light(node)
        if visible:
            if radius <= 2.0:
                samples = 6
            elif radius >= 36.0:
                samples = 40
            else:
                samples = int(round(((1.5 * radius) * (2 / 3.0)) + 3.75))
            celestial = Dome(radius * 1.5, samples, 2, color, 0, location,
                ((-(math.degrees(azimuth))), 90 + math.degrees(elevation), 0))
            self.celestials.attach(celestial)
