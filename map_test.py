import sys, random
from panda3d.core import *
from pandac.PandaModules import WindowProperties
from direct.gui.DirectGui import *
from direct.showbase.ShowBase import ShowBase

from pavara.maps import load_maps
from pavara.network import Server, Client
from pavara.constants import TCP_PORT
from pavara.world import Block, FreeSolid, Hector

class Pavara (ShowBase):
    def __init__(self, *args):
        ShowBase.__init__(self)

        print args

        self.x = None
        self.y = None

        # Init Panda3D crap.
        self.initP3D()
        maps = load_maps('Maps/bodhi.xml', self.cam)
        for map in maps:
            print map.name, '--', map.author
        self.map = maps[0]
        print render.analyze()

        # Testing physical hector.
        incarn = self.map.world.get_incarn()
        self.hector = self.map.world.attach(Hector(incarn))
        self.hector.setupColor({"barrel_color": Vec3(.7,.7,.7),
            "barrel_trim_color": Vec3(.2,.2,.2), "visor_color": Vec3(.3,.6,1),
            "body_color":Vec3(.6,.2,.2)})

        # Put the hector in the World's render so the lighting applies correctly
        # self.h = HectorActor(self.map.world.render, 0, 13, 14, 90)

        self.map.show(self.render)
        taskMgr.add(self.map.world.update, 'worldUpdateTask')

        # axes = loader.loadModel('models/yup-axis')
        # axes.setScale(10)
        # axes.reparentTo(render)

        host = args[0] if args else '127.0.0.1'
        print 'CONNECTING TO', host
        self.client = Client(self.map.world, host, TCP_PORT)

        self.setupInput()

    def initP3D(self):
        self.setBackgroundColor(0, 0, 0)
        self.enableParticles()
        self.disableMouse()
        render.setAntialias(AntialiasAttrib.MAuto)
        props = WindowProperties()
        props.setCursorHidden(True)
        self.win.requestProperties(props)
        self.camera.setPos(0, 20, 40)
        self.camera.setHpr(0, 0, 0)
        self.floater = NodePath(PandaNode("floater"))
        self.floater.reparentTo(render)
        self.up = Vec3(0, 1, 0)
        taskMgr.add(self.move, 'move')

    def setKey(self, key, value):
        self.keyMap[key] = value

    def drop_blocks(self):
        block = self.map.world.attach(FreeSolid(Block((1, 1, 1), (1, 0, 0, 1), 0.01, (0, 40, 0), (0, 0, 0)), 0.01))
        for i in range(10):
            rand_pos = (random.randint(-25, 25), 40, random.randint(-25, 25))
            block = self.map.world.attach(FreeSolid(Block((1, 1, 1), (1, 0, 0, 1), 0.01, rand_pos, (0, 0, 0)), 0.01))

    def setupInput(self):
        self.keyMap = { 'left': 0
                      , 'right': 0
                      , 'forward': 0
                      , 'backward': 0
                      , 'rotateLeft': 0
                      , 'rotateRight': 0
                      , 'walkForward': 0
                      , 'crouch': 0
                      }
        self.accept('escape', sys.exit)
        self.accept('p', self.drop_blocks)
        self.accept('w', self.setKey, ['forward', 1])
        self.accept('w-up', self.setKey, ['forward', 0])
        self.accept('a', self.setKey, ['left', 1])
        self.accept('a-up', self.setKey, ['left', 0])
        self.accept('s', self.setKey, ['backward', 1])
        self.accept('s-up', self.setKey, ['backward', 0])
        self.accept('d', self.setKey, ['right', 1])
        self.accept('d-up', self.setKey, ['right', 0])

        # Hector movement.
        self.accept('i',        self.client.send, ['forward', True])
        self.accept('i-up',     self.client.send, ['forward', False])
        self.accept('j',        self.client.send, ['left', True])
        self.accept('j-up',     self.client.send, ['left', False])
        self.accept('k',        self.client.send, ['backward', True])
        self.accept('k-up',     self.client.send, ['backward', False])
        self.accept('l',        self.client.send, ['right', True])
        self.accept('l-up',     self.client.send, ['right', False])
        self.accept('shift',    self.client.send, ['crouch', True])
        self.accept('shift-up', self.client.send, ['crouch', False])

    def move(self, task):
        dt = globalClock.getDt()
        if self.mouseWatcherNode.hasMouse():
            oldx = self.x
            oldy = self.y
            md = self.win.getPointer(0)
            self.x = md.getX()
            self.y = md.getY()
            centerx = self.win.getProperties().getXSize()/2
            centery = self.win.getProperties().getYSize()/2
            self.win.movePointer(0, centerx, centery)

            if (oldx is not None):
                self.floater.setPos(self.camera, 0, 0, 0)
                self.floater.setHpr(self.camera, 0, 0, 0)
                self.floater.setH(self.floater, (centerx-self.x) * 10 * dt)
                p = self.floater.getP()
                self.floater.setP(self.floater, (centery-self.y) * 10 * dt)
                self.floater.setZ(self.floater, -1)
                angle = self.up.angleDeg(self.floater.getPos() - self.camera.getPos())
                if 10 > angle or angle > 170:
                    self.floater.setPos(self.camera, 0, 0, 0)
                    self.floater.setP(p)
                    self.floater.setZ(self.floater, -1)
                self.camera.lookAt(self.floater.getPos(), self.up)
        else:
            self.x = None
            self.y = None
        if (self.keyMap['forward']):
            self.camera.setZ(self.camera, -25 * dt)
        if (self.keyMap['backward']):
            self.camera.setZ(self.camera, 25 * dt)
        if (self.keyMap['left']):
            self.camera.setX(self.camera, -25 * dt)
        if (self.keyMap['right']):
            self.camera.setX(base.camera, 25 * dt)

        return task.cont

if __name__ == '__main__':
    loadPrcFile('pavara.prc')
    p = Pavara(*sys.argv[1:])
    p.run()
