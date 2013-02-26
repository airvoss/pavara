from pandac.PandaModules import *
from panda3d.bullet import *
from pavara.utils.geom import GeomBuilder, to_cartesian
from pavara.assets import load_model
from direct.interval.LerpInterval import *
from direct.interval.IntervalGlobal import *
import math
import random

DEFAULT_AMBIENT_COLOR = (0.4, 0.4, 0.4, 1)
DEFAULT_GROUND_COLOR =  (0, 0, 0.15, 1)
DEFAULT_SKY_COLOR =     (0, 0, 0.15, 1)
DEFAULT_HORIZON_COLOR = (0, 0, 0.8, 1)
DEFAULT_HORIZON_SCALE = 0.05

NO_COLLISION_BITS = BitMask32.all_off()
MAP_COLLIDE_BIT =   BitMask32.bit(0)
SOLID_COLLIDE_BIT = BitMask32.bit(1)
GHOST_COLLIDE_BIT = BitMask32.bit(2)


class WorldObject (object):
    """
    Base class for anything attached to a World.
    """

    world = None
    last_unique_id = 0
    collide_bits = NO_COLLISION_BITS

    def __init__(self, name=None):
        self.name = name
        if not self.name:
            self.__class__.last_unique_id += 1
            self.name = '%s:%d' % (self.__class__.__name__, self.__class__.last_unique_id)

    def __repr__(self):
        return self.name

    def attached(self):
        """
        Called when this object is actually attached to a World. By this time, self.world will have been set.
        """
        pass

    def detached(self):
        """
        """
        pass

    def update(self, dt):
        """
        Called each frame if the WorldObject has registered itself as updatable.
        """
        pass

class PhysicalObject (WorldObject):
    """
    A WorldObject subclass that represents a physical object; i.e. one that is visible and may have an associated
    solid for physics collisions.
    """

    node = None
    solid = None
    collide_bits = MAP_COLLIDE_BIT
    moved = False

    def create_node(self):
        """
        Called by World.attach to create a NodePath that will be re-parented to the World's render.
        """
        pass

    def create_solid(self):
        """
        Called by World.attach to create any necessary physics geometry.
        """
        pass

    def collision(self, other, manifold, first):
        """
        Called when this object is collidable, and collides with another collidable object.
        :param manifold: The "line" between the colliding objects. See
                         https://www.panda3d.org/reference/devel/python/classpanda3d.bullet.BulletManifoldPoint.php
        :param first: Whether this was the "first" (node0) object in the collision.
        """
        pass

    def rotate(self, yaw, pitch, roll):
        """
        Programmatically set the yaw, pitch, and roll of this object.
        """
        self.node.set_hpr(yaw, pitch, roll)
#        self.moved = True
        
    def rotate_by(self, yaw, pitch, roll):
        """
        Programmatically rotate this object by the given yaw, pitch, and roll.
        """
        self.node.set_hpr(self.node, yaw, pitch, roll)
#        self.moved = True
        
    def move(self, center):
        """
        Programmatically move this object to be centered at the given coordinates.
        """
        self.node.set_pos(*center)
        self.moved = True

    def move_by(self, x, y, z):
        """
        Programmatically move this object by the given distances in each direction.
        """
        self.node.set_fluid_pos(self.node, x, y, z)
        if x and y and z:
            self.moved = True

    def position(self):
        return self.node.get_pos()

    def add_update(self, datagram):
        if not self.moved:
            return
        pos = self.position()
        hpr = self.node.get_hpr()
        datagram.addString(self.name)
        datagram.addFloat32(pos.x)
        datagram.addFloat32(pos.y)
        datagram.addFloat32(pos.z)
        datagram.addFloat32(hpr.x)
        datagram.addFloat32(hpr.y)
        datagram.addFloat32(hpr.z)
        self.moved = False

class CompositeObject (PhysicalObject):

    def __init__(self, name=None):
        super(CompositeObject, self).__init__(name)
        self.objects = []

    def create_node(self):
        composite_geom = GeomBuilder('composite')
        for obj in self.objects:
            obj.add_to(composite_geom)
        return NodePath(composite_geom.get_geom_node())

    def create_solid(self):
        node = BulletRigidBodyNode(self.name)
        for obj in self.objects:
            obj.add_solid(node)
        return node

    def attach(self, obj):
        self.objects.append(obj)

class Effect (object):
    """Effects wrap objects like boxes and ramps and other effects and change
       behavior of the wrapped object. Subclasses of this class automatically
       delegate anything not implemented in the effect to the object being
       wrapped."""
    def __init__(self, effected):
        object.__setattr__(self, 'effected', effected)

    def __setattr__(self, name, value):
        """Attributes should be assigned as deeply in the effect chain as
           possible, so go all the way down the effect chain, and start coming
           back until the first object to have the attribute is found. Set it
           and return. If nothing is found, set it in this object."""
        objs = []
        obj = self

        while hasattr(obj, 'effected'):
            obj = obj.effected
            objs.append(obj)

        while objs:
            obj = objs.pop()
            if hasattr(obj, name):
                object.__setattr__(obj, name, value)
                return
        object.__setattr__(self, name, value)

    def __getattr__(self, name):
        """Python only calls this function if the attribute is not defined on
           the object. This makes it so method calls are automatically passed
           down the effect chain if they are not defined in the effect object"""
        return getattr(self.effected, name)


class Hologram (Effect):
    collide_bits = NO_COLLISION_BITS

class FreeSolid (Effect):

    def __init__(self, effected, mass):
        if mass > 0:
            self.mass = mass
        Effect.__init__(self, effected)
        self.collide_bits = MAP_COLLIDE_BIT | SOLID_COLLIDE_BIT

    def create_solid(self):
        node = self.effected.create_solid()
        node.set_mass(self.mass if self.mass > 0 else 1)
        return node


class Transparent (Effect):

    def __init__(self, effected, alpha):
        Effect.__init__(self, effected)
        self.alpha = alpha

    def create_node(self):
        node = self.effected.create_node()
        node.set_transparency(TransparencyAttrib.MAlpha)
        node.setAlphaScale(self.alpha)
        return node

class Hector (PhysicalObject):

    collide_bits = SOLID_COLLIDE_BIT

    def __init__(self, incarnator, name=None):
        super(Hector, self).__init__(name=name)

        self.spawn_point = incarnator
        self.on_ground = False
        self.mass = 150.0 # 220.0 for heavy
        self.xz_velocity = Vec3(0, 0, 0)
        self.y_velocity = 0
        self.factors = {
            'forward': 0.1,
            'backward': -0.1,
            'left': 2.0,
            'right': -2.0,
            'crouch': 0.0,
        }
        self.movement = {
            'forward': 0.0,
            'backward': 0.0,
            'left': 0.0,
            'right': 0.0,
            'crouch': 0.0,
        }
        self.walk_phase = 0
        self.placing = False
        self.head_height = Vec3(0, 1.5, 0)
        self.collides_with = MAP_COLLIDE_BIT | SOLID_COLLIDE_BIT

    def create_node(self):
        from direct.actor.Actor import Actor
        self.actor = Actor('hector.egg')
        #self.actor.listJoints()
        self.barrels = self.actor.find("**/barrels")
        self.barrel_trim = self.actor.find("**/barrelTrim")
        self.visor = self.actor.find("**/visor")
        self.hull = self.actor.find("**/hull")
        self.crotch = self.actor.find("**/crotch")
        self.head = self.actor.attachNewNode("hector_head_node")
        self.barrels.reparentTo(self.head)
        self.barrel_trim.reparentTo(self.head)
        self.visor.reparentTo(self.head)
        self.hull.reparentTo(self.head)
        self.crotch.reparentTo(self.head)
        self.left_top = self.actor.find("**/leftTop")
        self.right_top = self.actor.find("**/rightTop")
        self.left_middle = self.actor.find("**/leftMiddle")
        self.left_middle.reparentTo(self.left_top)
        self.right_middle = self.actor.find("**/rightMiddle")
        self.right_middle.reparentTo(self.right_top)
        self.left_bottom = self.actor.find("**/leftBottom")
        self.left_bottom.reparentTo(self.left_middle)
        self.right_bottom = self.actor.find("**/rightBottom")
        self.right_bottom.reparentTo(self.right_middle)

        self.walk_playing = False

        def get_joint_control(name):
            return self.actor.controlJoint(None, "modelRoot", name)
        self.right_top_bone = get_joint_control("rightTopBone")
        self.left_top_bone = get_joint_control("leftTopBone")
        self.right_middle_bone = get_joint_control("rightMidBone")
        self.left_middle_bone = get_joint_control("leftMidBone")
        self.right_bottom_bone = get_joint_control("rightBottomBone")
        self.left_bottom_bone = get_joint_control("leftBottomBone")
        self.head_bone = get_joint_control("headBone")

        def get_joint_expose(name):
            return self.actor.exposeJoint(None, "modelRoot", name)
        self.right_top_bone_joint = get_joint_expose("rightTopBone")
        self.left_top_bone_joint = get_joint_expose("leftTopBone")
        self.right_middle_bone_joint = get_joint_expose("rightMidBone")
        self.left_middle_bone_joint = get_joint_expose("leftMidBone")
        self.right_bottom_bone_joint = get_joint_expose("rightBottomBone")
        self.left_bottom_bone_joint = get_joint_expose("leftBottomBone")
        self.head_bone_joint = get_joint_expose("headBone")

        self.torso_rest_y = [self.head_bone.get_pos(), self.left_top_bone.get_pos(), self.right_top_bone.get_pos()]
        self.legs_rest_mat = [ [self.right_top_bone.get_hpr(), self.right_middle_bone.get_hpr(), self.right_bottom_bone.get_hpr()],
                               [self.left_top_bone.get_hpr(), self.left_middle_bone.get_hpr(), self.left_bottom_bone.get_hpr()] ]

        """the below two functions will get called when the interval starts, allowing us to set the base
            positions for crouching/leg extension etc. currently they return the values without modification."""
        def get_base_leg_rotation(address1, address2, motion = 0):
            rest_rot = self.legs_rest_mat[address1][address2]
            return rest_rot + motion

        def get_base_torso_y(address, motion = 0):
            rest_y = self.torso_rest_y[address]
            return rest_y + motion

        def make_return_sequence():
            return_speed = .1
            y_return_head_int = LerpPosInterval(self.head_bone, return_speed, get_base_torso_y(0))
            y_return_left_int = LerpPosInterval(self.left_top_bone, return_speed, get_base_torso_y(1))
            y_return_right_int = LerpPosInterval(self.right_top_bone, return_speed, get_base_torso_y(2))

            r_top_return_int = LerpHprInterval(self.right_top_bone, return_speed, get_base_leg_rotation(0,0))
            r_mid_return_int = LerpHprInterval(self.right_middle_bone, return_speed, get_base_leg_rotation(0,1))
            r_bot_return_int = LerpHprInterval(self.right_bottom_bone, return_speed, get_base_leg_rotation(0,2))

            l_top_return_int = LerpHprInterval(self.left_top_bone, return_speed, get_base_leg_rotation(1,0))
            l_mid_return_int = LerpHprInterval(self.left_middle_bone, return_speed, get_base_leg_rotation(1,1))
            l_bot_return_int = LerpHprInterval(self.left_bottom_bone, return_speed, get_base_leg_rotation(1,2))

            return Parallel(r_top_return_int, r_mid_return_int, r_bot_return_int,
                            l_top_return_int, l_mid_return_int, l_bot_return_int,
                            y_return_head_int, y_return_left_int, y_return_right_int)

        self.return_seq = make_return_sequence()


        def make_walk_sequence():
            walk_cycle_speed = .8

            upbob = Vec3(0, 0.03, 0)
            downbob = upbob * -1
            bob_parts = (self.head_bone, self.left_top_bone, self.right_top_bone)

            down_interval = [LerpPosInterval(bone, walk_cycle_speed/4.0, get_base_torso_y(idx, downbob)) for (idx, bone) in enumerate(bob_parts)]
            up_interval = [LerpPosInterval(bone, walk_cycle_speed/4.0, get_base_torso_y(idx, upbob)) for (idx, bone) in enumerate(bob_parts)]

            top_motion = [Vec3(0, p, 0) for p in    [ 30, -5, -40,  28]]
            mid_motion = [Vec3(0, p, 0) for p in    [ 20,  0,  -4, -10]]
            bottom_motion = [Vec3(0, p, 0) for p in [-50,  5,  44, -18]]

            right_bones = [self.right_top_bone, self.right_middle_bone, self.right_bottom_bone]
            left_bones = [self.left_top_bone, self.left_middle_bone, self.left_bottom_bone]

            right_top_forward = [LerpHprInterval(right_bones[0], walk_cycle_speed/4.0, get_base_leg_rotation(0, 0, motion)) for motion in top_motion]
            right_mid_forward = [LerpHprInterval(right_bones[1], walk_cycle_speed/4.0, get_base_leg_rotation(0, 1, motion)) for motion in mid_motion]
            right_bottom_forward = [LerpHprInterval(right_bones[2], walk_cycle_speed/4.0, get_base_leg_rotation(0, 2, motion)) for motion in bottom_motion]

            left_top_forward = [LerpHprInterval(left_bones[0], walk_cycle_speed/4.0, get_base_leg_rotation(1, 0, motion)) for motion in top_motion]
            left_mid_forward = [LerpHprInterval(left_bones[1], walk_cycle_speed/4.0, get_base_leg_rotation(1, 1, motion)) for motion in mid_motion]
            left_bottom_forward = [LerpHprInterval(left_bones[2], walk_cycle_speed/4.0, get_base_leg_rotation(1, 2, motion)) for motion in bottom_motion]

            return Sequence(
                            Parallel(right_top_forward[0], right_mid_forward[0], right_bottom_forward[0],
                                    left_top_forward[2], left_mid_forward[2], left_bottom_forward[2],
                                    down_interval[0], down_interval[1], down_interval[2]),
                            Parallel(right_top_forward[1], right_mid_forward[1], right_bottom_forward[1],
                                    left_top_forward[3], left_mid_forward[3], left_bottom_forward[3],
                                    up_interval[0], up_interval[1], up_interval[2]),
                            Parallel(right_top_forward[2], right_mid_forward[2], right_bottom_forward[2],
                                    left_top_forward[0], left_mid_forward[0], left_bottom_forward[0],
                                    down_interval[0], down_interval[1], down_interval[2]),
                            Parallel(right_top_forward[3], right_mid_forward[3], right_bottom_forward[3],
                                    left_top_forward[1], left_mid_forward[1], left_bottom_forward[1],
                                    up_interval[0], up_interval[1], up_interval[2]),
                           )

        self.walk_forward_seq = make_walk_sequence()
        self.actor.set_pos(*self.spawn_point.pos)
        self.actor.look_at(*self.spawn_point.heading)
        return self.actor

    def create_solid(self):
        self.hector_capsule = BulletGhostNode(self.name + "_hect_cap")
        self.hector_capsule_shape = BulletCylinderShape(.7, .2, YUp)
        self.hector_bullet_np = self.actor.attach_new_node(self.hector_capsule)
        self.hector_bullet_np.node().add_shape(self.hector_capsule_shape)
        self.hector_bullet_np.node().set_kinematic(True)
        self.hector_bullet_np.set_pos(0,1.5,0)
        self.hector_bullet_np.wrt_reparent_to(self.actor)
        self.world.physics.attach_ghost(self.hector_capsule)
        self.hector_bullet_np.node().setIntoCollideMask(GHOST_COLLIDE_BIT)
        return None

    def setup_shape(self, gnodepath, bone, pname):
        shape = BulletConvexHullShape()

        gnode = gnodepath.node()
        geom = gnode.get_geom(0)
        shape.add_geom(geom)

        node = BulletRigidBodyNode(self.name + pname)
        np = self.actor.attach_new_node(node)
        np.node().add_shape(shape)
        np.node().set_kinematic(True)
        np.wrt_reparent_to(bone)
        self.world.physics.attach_rigid_body(node)
        return np

    def setupColor(self, colordict):
        if colordict.has_key("barrel_color"):
            self.barrels.setColor(*colordict.get("barrel_color"))
        if colordict.has_key("barrel_trim_color"):
            self.barrel_trim.setColor(*colordict.get("barrel_trim_color"))
        if colordict.has_key("visor_color"):
            self.visor.setColor(*colordict.get("visor_color"))
        if colordict.has_key("body_color"):
            color = colordict.get("body_color")
            self.hull.setColor(*color)
            self.crotch.setColor(*color)
            self.left_top.setColor(*color)
            self.right_top.setColor(*color)
            self.left_middle.setColor(*color)
            self.right_middle.setColor(*color)
            self.left_bottom.setColor(*color)
            self.right_bottom.setColor(*color)
        if colordict.has_key("hull_color"):
            self.hull.setColor(*colordict.get("hull_color"))
            self.crotch.setColor(*colordict.get("hull_color"))
        if colordict.has_key("top_leg_color"):
            color = colordict.get("top_leg_color")
            self.left_top.setColor(*color)
            self.right_top.setColor(*color)
        if colordict.has_key("middle_leg_color"):
            color = colordict.get("middle_leg_color")
            self.left_middle.setColor(*color)
            self.right_middle.setColor(*color)
        if colordict.has_key("bottom_leg_color"):
            color = colordict.get("bottom_leg_color")
            self.left_bottom.setColor(*color)
            self.right_bottom.setColor(*color)
        return

    def attached(self):
        #self.node.set_scale(3.0)
        self.world.register_collider(self)
        self.world.register_updater(self)

    def collision(self, other, manifold, first):
        world_pt = manifold.get_position_world_on_a() if first else manifold.get_position_world_on_b()
        print self, 'HIT BY', other, 'AT', world_pt

    def handle_command(self, cmd, pressed):
        if cmd is 'crouch' and not pressed and self.on_ground:
            self.y_velocity = 0.1
        self.movement[cmd] = self.factors[cmd] if pressed else 0.0

    def update(self, dt):
        dt = min(dt, 0.2) # let's just temporarily assume that if we're getting less than 5 fps, dt must be wrong.
        yaw = self.movement['left'] + self.movement['right']
        self.rotate_by(yaw * dt * 60, 0, 0)
        walk = self.movement['forward'] + self.movement['backward']
        start = self.position()
        cur_pos_ts = TransformState.make_pos(self.position() + self.head_height)
        if self.on_ground:
            speed = walk
            self.xz_velocity = self.position()
            self.move_by(0, 0, speed * dt * 60)
            self.xz_velocity -= self.position()
            self.xz_velocity *= -1
            self.xz_velocity /= (dt * 60)
        else:
            self.move(self.position() + self.xz_velocity * dt * 60)

        # Cast a ray from just above our feet to just below them, see if anything hits.
        pt_from = self.position() + Vec3(0, 1, 0)
        pt_to = pt_from + Vec3(0, -1.1, 0)
        result = self.world.physics.ray_test_closest(pt_from, pt_to, MAP_COLLIDE_BIT | SOLID_COLLIDE_BIT)
        self.update_legs(walk,dt)
        if self.y_velocity <= 0 and result.has_hit():
            self.on_ground = True
            self.y_velocity = 0
            self.move(result.get_hit_pos())
        else:
            self.on_ground = False
            self.y_velocity -= 0.20 * dt
            self.move_by(0, self.y_velocity * dt * 60, 0)
        goal = self.position()
        adj_dist = abs((start - goal).length())
        new_pos_ts = TransformState.make_pos(self.position() + self.head_height)

        sweep_result = self.world.physics.sweepTestClosest(self.hector_capsule_shape, cur_pos_ts, new_pos_ts, self.collides_with, 0)
        while sweep_result.has_hit():
            moveby = sweep_result.get_hit_normal()
            moveby.normalize()
            moveby *= adj_dist * (1 - sweep_result.get_hit_fraction())
            self.move(self.position() + moveby)
            new_pos_ts = TransformState.make_pos(self.position() + self.head_height)
            sweep_result = self.world.physics.sweepTestClosest(self.hector_capsule_shape, cur_pos_ts, new_pos_ts, self.collides_with, 0)


    def update_legs(self, walk, dt):
        if walk != 0:
            if not self.walk_playing:
                self.walk_playing = True
                self.walk_forward_seq.loop()
        else:
            if self.walk_playing:
                self.walk_playing = False
                self.walk_forward_seq.pause()
                self.return_seq.start()
    #def crouch(self):

    #def uncrouch(self):


class Block (PhysicalObject):
    """
    A block. Blocks with non-zero mass will be treated as freesolids.
    """

    def __init__(self, size, color, mass, center, hpr, name=None):
        super(Block, self).__init__(name)
        self.size = size
        self.color = color
        self.mass = mass
        self.center = center
        self.hpr = hpr

    def create_node(self):
        return NodePath(GeomBuilder('block').add_block(self.color, (0, 0, 0), self.size).get_geom_node())

    def create_solid(self):
        node = BulletRigidBodyNode(self.name)
        node.add_shape(BulletBoxShape(Vec3(self.size[0] / 2.0, self.size[1] / 2.0, self.size[2] / 2.0)))
        return node

    def add_solid(self, node):
        node.add_shape(BulletBoxShape(Vec3(self.size[0] / 2.0, self.size[1] / 2.0, self.size[2] / 2.0)), TransformState.make_pos_hpr(Point3(*self.center), self.hpr))

    def add_to(self, geom_builder):
        rot = LRotationf()
        rot.set_hpr(self.hpr)
        geom_builder.add_block(self.color, self.center, self.size, rot)

    def attached(self):
        self.move(self.center)
        self.rotate(*self.hpr)

class Dome (PhysicalObject):
    """
    A dome.
    """

    def __init__(self, radius, color, mass, center, hpr, name=None):
        super(Dome, self).__init__(name)
        self.radius = radius
        self.color = color
        self.mass = mass
        self.center = center
        self.hpr = hpr

    def create_node(self):
        self.geom = GeomBuilder().add_dome(self.color, (0, 0, 0), self.radius, 8, 5).get_geom_node()
        return NodePath(self.geom)

    def create_solid(self):
        node = BulletRigidBodyNode(self.name)
        mesh = BulletConvexHullShape()
        mesh.add_geom(self.geom.get_geom(0))
        node.add_shape(mesh)
        return node

    def add_solid(self, node):
        mesh = BulletConvexHullShape()
        mesh.add_geom(GeomBuilder().add_dome(self.color, self.center, self.radius, 8, 5).get_geom())
        node.add_shape(mesh)
        return node

    def add_to(self, geom_builder):
        rot = LRotationf(*self.hpr)
        geom_builder.add_dome(self.color, self.center, self.radius, 8, 5, rot)

    def attached(self):
        self.move(self.center)
        self.rotate_by(*self.hpr)

class Ground (PhysicalObject):
    """
    The ground. This is not a visible object, but does create a physical solid.
    """

    def __init__(self, radius, color, name=None):
        super(Ground, self).__init__(name)
        self.color = color

    def create_solid(self):
        node = BulletRigidBodyNode(self.name)
        node.add_shape(BulletPlaneShape(Vec3(0, 1, 0), 1))
        return node

    def attached(self):
        self.move((0, -1.0, 0))
        # We need to tell the sky shader what color we are.
        self.world.sky.set_ground(self.color)

class Ramp (PhysicalObject):
    """
    A ramp. Basically a block that is rotated, and specified differently in XML. Should maybe be a Block subclass?
    """

    def __init__(self, base, top, width, thickness, color, mass, hpr, name=None):
        super(Ramp, self).__init__(name)
        self.base = Point3(*base)
        self.top = Point3(*top)
        self.width = width
        self.thickness = thickness
        self.color = color
        self.length = (self.top - self.base).length()
        self.mass = mass
        self.hpr = hpr
        v1 = self.top - self.base
        if self.base.get_x() != self.top.get_x():
            p3 = Point3(self.top.get_x()+100, self.top.get_y(), self.top.get_z())
        else:
            p3 = Point3(self.top.get_x(), self.top.get_y(), self.top.get_z() + 100)
        v2 = self.top - p3
        self.up = v1.cross(v2)
        self.up.normalize()
        self.midpoint = Point3((self.base + self.top) / 2.0)

    def create_node(self):
        return NodePath(GeomBuilder('ramp').add_block(self.color, (0, 0, 0), (self.thickness, self.width, self.length)).get_geom_node())

    def create_solid(self):
        node = BulletRigidBodyNode(self.name)
        node.add_shape(BulletBoxShape(Vec3(self.thickness / 2.0, self.width / 2.0, self.length / 2.0)))
        return node

    def add_solid(self, node):
        node.add_shape(BulletBoxShape(Vec3(self.thickness / 2.0, self.width / 2.0, self.length / 2.0)), TransformState.make_pos_hpr(Point3(*self.midpoint), self.hpr))

    def add_to(self, geom_builder):
        # honestly i don't understand this at all
        diff = self.top - self.base
        if diff.get_z() == 0:
            vec2 = diff.get_xy()
            vec2 = Vec2(abs(diff.get_x()), diff.get_y())
            vec1 = Vec2(1, 0)
        else:
            vec1 = diff.get_yz()
            vec1 = Vec2(diff.get_y(), abs(diff.get_z()))
            vec2 = Vec2(0, -1)
        rot = LRotation(diff.get_xz().signedAngleDeg(Vec2(0, -1)), (vec1.signedAngleDeg(vec2) ), 90)
        rot = rot * LRotation(*self.hpr)
        self.hpr = rot.get_hpr()
        geom_builder.add_block(self.color, self.midpoint, (self.thickness, self.width, self.length), rot)

    def attached(self):
        # Do the block rotation after we've been attached (i.e. have a NodePath), so we can use node.look_at.
        self.move(self.midpoint)
        self.node.look_at(self.top, self.up)
        self.rotate_by(*self.hpr)

class Sky (WorldObject):
    """
    The sky is actually just a square re-parented onto the camera, with a shader to handle the coloring and gradient.
    """

    def __init__(self, ground=DEFAULT_GROUND_COLOR, color=DEFAULT_SKY_COLOR, horizon=DEFAULT_HORIZON_COLOR, scale=DEFAULT_HORIZON_SCALE):
        super(Sky, self).__init__('sky')
        self.ground = ground
        self.color = color
        self.horizon = horizon
        self.scale = scale

    def attached(self):
        if not self.world.camera:
            self.node = self.world.render.attach_new_node('sky')
            return
        geom = GeomNode('sky')
        bounds = self.world.camera.node().get_lens().make_bounds()
        dl = bounds.getMin()
        ur = bounds.getMax()
        z = dl.getZ() * 0.99

        geom.add_geom(GeomBuilder('sky').add_rect((1, 1, 1, 1), dl.getX(), dl.getY(), 0, ur.getX(), ur.getY(), 0).get_geom())
        self.node = self.world.render.attach_new_node(geom)
        self.node.set_shader(Shader.load('Shaders/Sky.sha'))
        self.node.set_shader_input('camera', self.world.camera)
        self.node.set_shader_input('sky', self.node)
        self.node.set_shader_input('groundColor', *self.ground)
        self.node.set_shader_input('skyColor', *self.color)
        self.node.set_shader_input('horizonColor', *self.horizon)
        self.node.set_shader_input('gradientHeight', self.scale, 0, 0, 0)
        self.node.reparent_to(self.world.camera)
        self.node.set_pos(self.world.camera, 0, 0, z)

    def set_ground(self, color):
        self.ground = color
        self.node.set_shader_input('groundColor', *self.ground)

    def set_color(self, color):
        self.color = color
        self.node.set_shader_input('skyColor', *self.color)

    def set_horizon(self, color):
        self.horizon = color
        self.node.set_shader_input('horizonColor', *self.horizon)

    def set_scale(self, height):
        self.scale = height
        self.node.set_shader_input('gradientHeight', self.scale, 0, 0, 0)

class Incarnator (WorldObject):
    def __init__(self, pos, heading, name=None):
        super(Incarnator, self).__init__(name)
        self.pos = Vec3(*pos)
        self.heading = Vec3(to_cartesian(math.radians(heading), 0, 1000.0 * 255.0 / 256.0)) * -1

class Goody (PhysicalObject):
    def __init__(self, pos, model, items, respawn, spin, name=None):
        super(Goody, self).__init__(name)
        self.pos = Vec3(*pos)
        self.grenades = items[0]
        self.missles = items[1]
        self.boosters = items[2]
        self.model = model
        self.respawn = float(respawn)
        self.spin = spin
        self.geom = None
        self.active = True
        self.timeout = 0

    def create_node(self):
        m = load_model('misc/rgbCube')
        m.set_scale(.5)
        m.set_hpr(45,45,45)
        return m

    def create_solid(self):
        node = BulletGhostNode(self.name)
        node_shape = BulletSphereShape(.5)
        node.add_shape(node_shape)
        node.set_kinematic(True)
        return node

    def attached(self):
        self.node.set_pos(self.pos)
        self.world.register_updater(self)
        self.world.register_collider(self)
        self.solid.setIntoCollideMask(GHOST_COLLIDE_BIT)

    def update(self, dt):
        if not self.active:
            self.timeout += dt
            if self.timeout > self.respawn:
                self.active = True
                self.node.show()
                self.timeout = 0
            return

        self.rotate_by(*[x * dt for x in self.spin])
        result = self.world.physics.contact_test(self.solid)
        for contact in result.getContacts():
            node_1 = contact.getNode0()
            node_2 = contact.getNode1()
            if "Hector" in node_2.get_name():
               # TODO: identify which player and credit them with the items.
               self.active = False
               self.node.hide()

class World (object):
    """
    The World models basically everything about a map, including gravity, ambient light, the sky, and all map objects.
    """

    def __init__(self, camera, debug=False, client=None, server=None):
        self.objects = {}
        self.incarnators = []
        self.collidables = set()
        self.updatables = set()
        self.render = NodePath('world')
        self.camera = camera
        self.ambient = self._make_ambient()
        self.sky = self.attach(Sky())
        # Set up the physics world. TODO: let maps set gravity.
        self.physics = BulletWorld()
        self.physics.set_gravity(Vec3(0, -9.81, 0))
        self.debug = debug
        self.client = client
        self.server = server
        if debug:
            debug_node = BulletDebugNode('Debug')
            debug_node.show_wireframe(True)
            debug_node.show_constraints(True)
            debug_node.show_bounding_boxes(False)
            debug_node.show_normals(False)
            np = self.render.attach_new_node(debug_node)
            np.show()
            self.physics.set_debug_node(debug_node)

    def _make_ambient(self):
        alight = AmbientLight('ambient')
        alight.set_color(VBase4(*DEFAULT_AMBIENT_COLOR))
        node = self.render.attach_new_node(alight)
        self.render.set_light(node)
        return node

    def attach(self, obj):
        assert hasattr(obj, 'world') and hasattr(obj, 'name')
        assert obj.name not in self.objects
        obj.world = self
        if obj.name.startswith('Incarnator'):
            self.incarnators.append(obj)
        if hasattr(obj, 'create_node') and hasattr(obj, 'create_solid'):
            # Let each object define it's own NodePath, then reparent them.
            obj.node = obj.create_node()
            obj.solid = obj.create_solid()
            if obj.solid:
                if isinstance(obj.solid, BulletRigidBodyNode):
                    self.physics.attach_rigid_body(obj.solid)
                elif isinstance(obj.solid, BulletGhostNode):
                    self.physics.attach_ghost(obj.solid)
            if obj.node:
                if obj.solid:
                    # If this is a solid visible object, create a new physics node and reparent the visual node to that.
                    phys_node = self.render.attach_new_node(obj.solid)
                    obj.node.reparent_to(phys_node)
                    obj.node = phys_node
                else:
                    # Otherwise just reparent the visual node to the root.
                    obj.node.reparent_to(self.render)
            elif obj.solid:
                obj.node = self.render.attach_new_node(obj.solid)
            if obj.solid and obj.collide_bits is not None:
                obj.solid.set_into_collide_mask(obj.collide_bits)
        self.objects[obj.name] = obj
        # Let the object know it has been attached.
        obj.attached()
        return obj

    def get_incarn(self):
        return random.choice(self.incarnators)

    def create_hector(self, name=None):
        return self.attach(Hector(self.get_incarn(), name=name))

    def set_ambient(self, color):
        """
        Sets the ambient light to the given color.
        """
        self.ambient.node().set_color(VBase4(*color))

    def add_celestial(self, azimuth, elevation, color, intensity, radius, visible):
        """
        Adds a celestial light source to the scene. If it is a visible celestial, also add a sphere model.
        """
        if not self.camera:
            return
        location = Vec3(to_cartesian(azimuth, elevation, 1000.0 * 255.0 / 256.0))
        if intensity:
            dlight = DirectionalLight('celestial')
            dlight.set_color((color[0]*intensity, color[1]*intensity, color[2]*intensity, 1.0))
            node = self.render.attach_new_node(dlight)
            node.look_at(*(location * -1))
            self.render.set_light(node)
        if visible:
            sphere = load_model('misc/sphere')
            sphere.set_transparency(TransparencyAttrib.MAlpha)
            sphere.reparent_to(self.render)
            sphere.set_light_off()
            sphere.set_effect(CompassEffect.make(self.camera, CompassEffect.PPos))
            sphere.set_scale(45*radius)
            sphere.set_color(*color)
            sphere.set_pos(location)

    def register_collider(self, obj):
        assert isinstance(obj, PhysicalObject)
        self.collidables.add(obj)

    def register_updater(self, obj):
        assert isinstance(obj, WorldObject)
        self.updatables.add(obj)

    def update(self, task):
        """
        Called every frame to update the physics, etc.
        """
        dt = globalClock.getDt()
        for obj in self.updatables:
            obj.update(dt)
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
