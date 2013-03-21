import sys, random
from panda3d.core import *
from pandac.PandaModules import WindowProperties
from direct.gui.DirectGui import *
from direct.showbase.ShowBase import ShowBase
from direct.showbase import Audio3DManager
from direct.filter.CommonFilters import CommonFilters

from pavara.maps import load_maps
from pavara.world import Block, FreeSolid
from pavara.hector import Hector


class Pavara (ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        self.x = None
        self.y = None

        self.render.setShaderAuto()
        self.initP3D()
        self.audio3d = Audio3DManager.Audio3DManager(self.sfxManagerList[0], self.cam)
        maps = load_maps('Maps/bodhi.xml', self.cam, audio3d=self.audio3d)

        for map in maps:
            print map.name, '--', map.author
        self.map = maps[0]

        # Testing physical hector.
        incarn = self.map.world.get_incarn()
        self.hector = self.map.world.attach(Hector(incarn))

        self.hector.setup_color({
            "barrel_outer_color": [.7,.7,.7],
            "barrel_inner_color": [.2,.2,.2],
            "visor_color": [2.0/255, 94.0/255, 115.0/255],
            "body_primary_color": [3.0/255, 127.0/255, 140.0/255],
            "body_secondary_color": [217.0/255, 213.0/255, 154.0/255],
            "engines": [89.0/255, 2.0/255, 2.0/255]
        })

        self.setupInput()

        self.map.show(self.render)
        black_shader=loader.loadShader("Shaders/blackShader.sha")
        glow_buffer=self.win.make_texture_buffer("Glow scene", 512, 512)
        glow_buffer.set_sort(-3)
        glow_buffer.set_clear_color(Vec4(0,0,0,1))

        # We have to attach a camera to the glow buffer. The glow camera
        # must have the same frustum as the main camera. As long as the aspect
        # ratios match, the rest will take care of itself.
        glow_camera = self.makeCamera(glow_buffer, lens=self.cam.node().get_lens())

        # Tell the glow camera to use the glow shader
        tempnode = NodePath(PandaNode("temp node"))
        tempnode.set_shader(black_shader, 100)
        glow_camera.node().set_initial_state(tempnode.get_state())


        # set up the pipeline: from glow scene to blur x to blur y to main window.
        blur_xbuffer=self.make_filter_buffer(glow_buffer,  "Blur X", -2, "Shaders/XBlurShader.sha")
        blur_ybuffer=self.make_filter_buffer(blur_xbuffer, "Blur Y", -1, "Shaders/YBlurShader.sha")
        self.finalcard = blur_ybuffer.get_texture_card()
        self.finalcard.reparent_to(render2d)
        self.finalcard.set_attrib(ColorBlendAttrib.make(ColorBlendAttrib.MAdd))
        taskMgr.add(self.map.world.update, 'worldUpdateTask')
        base.bufferViewer.setPosition("llcorner")
        base.bufferViewer.setLayout("hline")
        base.bufferViewer.setCardSize(0.652,0)
        #base.bufferViewer.toggleEnable()

        # axes = loader.loadModel('models/yup-axis')
        # axes.setScale(10)
        # axes.reparentTo(render)

    def initP3D(self):
        self.setBackgroundColor(0, 0, 0)
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
                      , 'fire': 0
                      , 'missile': 0
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
        self.accept('i',        self.hector.handle_command, ['forward', True])
        self.accept('i-up',     self.hector.handle_command, ['forward', False])
        self.accept('j',        self.hector.handle_command, ['left', True])
        self.accept('j-up',     self.hector.handle_command, ['left', False])
        self.accept('k',        self.hector.handle_command, ['backward', True])
        self.accept('k-up',     self.hector.handle_command, ['backward', False])
        self.accept('l',        self.hector.handle_command, ['right', True])
        self.accept('l-up',     self.hector.handle_command, ['right', False])
        self.accept('shift',    self.hector.handle_command, ['crouch', True])
        self.accept('shift-up', self.hector.handle_command, ['crouch', False])
        self.accept('mouse1',   self.hector.handle_command, ['fire', True])
        self.accept('mouse1-up',self.hector.handle_command, ['fire', False])
        self.accept('u',        self.hector.handle_command, ['missile', True])
        self.accept('u-up',     self.hector.handle_command, ['missile', False])

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

    def make_filter_buffer(self, srcbuffer, name, sort, prog):
        blur_buffer = self.win.make_texture_buffer(name, 512, 512)
        blur_buffer.set_sort(sort)
        blur_buffer.set_clear_color(Vec4(1,0,0,1))
        blur_camera = self.makeCamera2d(blur_buffer)
        blur_scene = NodePath("new Scene")
        blur_camera.node().set_scene(blur_scene)
        shader = loader.loadShader(prog)
        card = srcbuffer.get_texture_card()
        card.reparent_to(blur_scene)
        card.set_shader(shader)
        return blur_buffer

if __name__ == '__main__':
    loadPrcFile('pavara.prc')
    p = Pavara()
    p.run()
