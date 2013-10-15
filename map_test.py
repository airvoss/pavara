import sys, os, random
from panda3d.core import *
from panda3d.rocket import *
from pandac.PandaModules import WindowProperties
from direct.gui.DirectGui import *
from direct.showbase.ShowBase import ShowBase
from direct.showbase import Audio3DManager
from direct.filter.CommonFilters import CommonFilters
from direct.interval.LerpInterval import *
from direct.interval.IntervalGlobal import *

from pavara.maps import load_maps
from pavara.world import Block, FreeSolid, MODEL_CAM_BITS, LIGHT_CAM_BITS, PLAIN_CAM_BITS, BLOOM_CAM_BITS
from pavara.utils.geom import GeomBuilder
from pavara.walker import Walker


class MapTest (ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        self.light_shaders = None
        self.x = None
        self.y = None

        #self.task_mgr = taskMgr
        self.initP3D()
        self.audio3d = Audio3DManager.Audio3DManager(self.sfxManagerList[0], self.cam)
        self.audio3d.setDopplerFactor(.7)

        self.doc = None
        self.map = None

        self.modelbuffer = self.makeFBO("model buffer",1)
        self.lightbuffer = self.makeFBO("light buffer",0)
        self.bloombuffer = self.makeFBO("bloom buffer",0)

        self.texDepth = Texture()
        self.texDepth.setFormat(Texture.FDepthStencil)
        self.texAlbedo = Texture()
        self.texNormal = Texture()
        self.texFinal = Texture()
        self.texBloom = Texture()
        self.modelbuffer.addRenderTexture(self.texDepth,  GraphicsOutput.RTMBindOrCopy, GraphicsOutput.RTPDepthStencil)
        self.modelbuffer.addRenderTexture(self.texAlbedo, GraphicsOutput.RTMBindOrCopy, GraphicsOutput.RTPColor)
        self.modelbuffer.addRenderTexture(self.texNormal, GraphicsOutput.RTMBindOrCopy, GraphicsOutput.RTPAuxRgba0)

        self.lightbuffer.addRenderTexture(self.texFinal,  GraphicsOutput.RTMBindOrCopy, GraphicsOutput.RTPColor)
        self.lightbuffer.addRenderTexture(self.texDepth,  GraphicsOutput.RTMBindOrCopy, GraphicsOutput.RTPDepthStencil)
        self.bloombuffer.addRenderTexture(self.texBloom,  GraphicsOutput.RTMBindOrCopy, GraphicsOutput.RTPColor)
        self.bloombuffer.addRenderTexture(self.texDepth,  GraphicsOutput.RTMBindOrCopy, GraphicsOutput.RTPDepthStencil)
        lens = self.cam.node().get_lens()
        self.modelcam = base.makeCamera(self.modelbuffer, lens=lens, scene=render, mask=MODEL_CAM_BITS)
        self.lightcam = base.makeCamera(self.lightbuffer, lens=lens, scene=render, mask=LIGHT_CAM_BITS)
        self.plaincam = base.makeCamera(self.lightbuffer, lens=lens, scene=render, mask=PLAIN_CAM_BITS)
        self.bloomcam = base.makeCamera(self.bloombuffer, lens=lens, scene=render, mask=BLOOM_CAM_BITS)
        self.cam.node().set_active(0)

        self.modelbuffer.setSort(1)
        self.lightbuffer.setSort(2)
        self.bloombuffer.setSort(3)
        self.win.setSort(4)

        self.modelcam.node().getDisplayRegion(0).setSort(0)
        self.lightcam.node().getDisplayRegion(0).setSort(1)
        self.plaincam.node().getDisplayRegion(0).setSort(2)

        self.modelcam.node().getDisplayRegion(0).disableClears()
        self.lightcam.node().getDisplayRegion(0).disableClears()
        self.plaincam.node().getDisplayRegion(0).disableClears()
        self.bloomcam.node().getDisplayRegion(0).disableClears()
        base.cam.node().getDisplayRegion(0).disableClears()
        base.cam2d.node().getDisplayRegion(0).disableClears()
        self.modelbuffer.disableClears()
        #self.bloombuffer.disableClears()
        base.win.disableClears()
        self.bloombuffer.setClearDepthActive(0)
        self.modelbuffer.setClearColorActive(1)
        self.modelbuffer.setClearDepthActive(1)
        self.lightbuffer.setClearColorActive(1)
        self.lightbuffer.setClearDepthActive(0)
        self.lightbuffer.setClearColor(Vec4(0,0,0,1))
        self.bloombuffer.setClearColor(Vec4(0,0,0,1))

        proj = base.cam.node().getLens().getProjectionMat()
        print proj
        # vvvv probably not right vvvv TODO: understand projection matrices.
        proj_x = -0.5 * proj.getCell(3,2) / proj.getCell(0,0)
        proj_y = -0.5 * proj.getCell(3,2) / proj.getCell(1,1)
        proj_z = 0.5 * proj.getCell(3,2)
        proj_w = 0.5 - 0.5*proj.getCell(2,2)

        tempnode = NodePath(PandaNode("temp node"))
        tempnode.setAttrib(AlphaTestAttrib.make(RenderAttrib.MGreaterEqual, 0.5))
        tempnode.setShader(loader.loadShader("Shaders/model.sha"))
        tempnode.setShaderInput('usevertex', 1,1,1,1)
        tempnode.setAttrib(DepthTestAttrib.make(RenderAttrib.MLessEqual))
        self.modelcam.node().setInitialState(tempnode.getState())

        tempnode = NodePath(PandaNode("temp node"))
        tempnode.setShader(loader.loadShader("Shaders/light.sha"))
        tempnode.setShaderInput("texnormal",self.texNormal)
        tempnode.setShaderInput("texalbedo",self.texAlbedo)
        tempnode.setShaderInput("texdepth",self.texDepth)
        tempnode.setShaderInput("camera",self.lightcam)
        tempnode.setShaderInput("proj",Vec4(proj_x,proj_y,proj_z,proj_w))
        tempnode.setAttrib(ColorBlendAttrib.make(ColorBlendAttrib.MAdd, ColorBlendAttrib.OOne, ColorBlendAttrib.OOne))
        tempnode.setAttrib(CullFaceAttrib.make(CullFaceAttrib.MCullCounterClockwise))
        tempnode.setAttrib(DepthTestAttrib.make(RenderAttrib.MGreaterEqual))
        tempnode.setAttrib(DepthWriteAttrib.make(DepthWriteAttrib.MOff))
        self.lightcam.node().setInitialState(tempnode.getState())

        tempnode = NodePath(PandaNode('temp node'))
        tempnode.setAttrib(DepthTestAttrib.make(RenderAttrib.MLess))
        tempnode.setAttrib(DepthWriteAttrib.make(DepthWriteAttrib.MOn))
        self.plaincam.node().setInitialState(tempnode.getState())

        tempnode = NodePath(PandaNode('temp node'))
        tempnode.setAttrib(DepthTestAttrib.make(RenderAttrib.MLessEqual))
        tempnode.setAttrib(DepthWriteAttrib.make(DepthWriteAttrib.MOff))
        self.bloomcam.node().setInitialState(tempnode.getState())

        render.setState(RenderState.makeEmpty())

        self.lightroot = NodePath(PandaNode("lightroot"))
        self.lightroot.reparentTo(render)
        self.modelroot = NodePath(PandaNode("modelroot"))
        self.modelroot.reparentTo(render)
        self.lightroot.hide(BitMask32(MODEL_CAM_BITS))
        self.modelroot.hide(BitMask32(LIGHT_CAM_BITS))
        self.modelroot.hide(BitMask32(PLAIN_CAM_BITS))
        self.modelroot.hide(BitMask32(BLOOM_CAM_BITS))

        # set up the pipeline: from glow scene to blur x to blur y to main window.
        blur_xbuffer=self.make_filter_buffer(self.bloombuffer,  "Blur X", -2, "Shaders/XBlurShader.sha")
        blur_ybuffer=self.make_filter_buffer(blur_xbuffer, "Blur Y", -1, "Shaders/YBlurShader.sha")
        #fxaa_shader = loader.loadShader('Shaders/fxaa.cg')
        #fxaa_buffer = base.win.make_texture_buffer('fxaa', 1024, 1024)
        #fxaa_buffer.set_sort(5) # ???
        #fxaa_buffer.set_clear_color(Vec4(1,0,0,1))
        #fxaa_cam = base.makeCamera2d(fxaa_buffer)
        #fxaa_scene = NodePath('fxaa')
        #fxaa_cam.node().set_scene(fxaa_scene)
        #card = self.lightbuffer.get_texture_card()
        #card.reparent_to(fxaa_scene)
        #card.set_shader(fxaa_shader)
        #card.set_shader_input('color', self.texFinal)
        self.finalcard = self.lightbuffer.get_texture_card()
        self.finalcard.set_texture(self.texFinal)
        self.finalcard.reparent_to(render2d)
        self.blurcard = blur_ybuffer.get_texture_card()
        #self.finalcard.set_texture(self.texFinal)
        self.blurcard.reparent_to(render2d)
        self.blurcard.set_attrib(ColorBlendAttrib.make(ColorBlendAttrib.MAdd))
        base.bufferViewer.setPosition("llcorner")
        base.bufferViewer.setLayout("hline")
        base.bufferViewer.setCardSize(0.652,0)
        #self.modelbuffer.setClearColorActive(1)
        #base.bufferViewer.toggleEnable()
        if len(sys.argv) > 1:
            self.switch_map(sys.argv[1])
            self.start_map()
        else:
            self.switch_map("../Ui/scenes/splash.xml", audio=False)
            self.fade_in()
            incarn = self.map.world.get_incarn()
            self.map.world.attach(Walker(incarn))
            self.show_selection_screen()

        # axes = loader.loadModel('models/yup-axis')
        # axes.setScale(10)
        # axes.reparentTo(render)

    def makeFBO(self, name, auxrgba):
        # This routine creates an offscreen buffer.  All the complicated
        # parameters are basically demanding capabilities from the offscreen
        # buffer - we demand that it be able to render to texture on every
        # bitplane, that it can support aux bitplanes, that it track
        # the size of the host window, that it can render to texture
        # cumulatively, and so forth.
        winprops = WindowProperties()
        print winprops
        props = FrameBufferProperties()
        props.setRgbColor(1)
        props.setAlphaBits(1)
        props.setDepthBits(1)
        props.setAuxRgba(auxrgba)
        return base.graphicsEngine.makeOutput(
             base.pipe, "model buffer", -2,
             props, winprops,
             GraphicsPipe.BFSizeTrackHost | GraphicsPipe.BFCanBindEvery |
             GraphicsPipe.BFRttCumulative | GraphicsPipe.BFRefuseWindow,
             base.win.getGsg(), base.win)

    def initP3D(self):
        self.disableMouse()
        self.setBackgroundColor(0, 0, 0)
        #render.setAntialias(AntialiasAttrib.MAuto)
        self.floater = NodePath(PandaNode("floater"))
        self.floater.reparentTo(render)
        self.up = Vec3(0, 1, 0)

    def fade_in(self):
        #blackout card
        bgeom = GeomBuilder().add_rect([0,0,0,1],-5,-5,0,5,5,0).get_geom_node()
        b = render.attach_new_node(bgeom)
        b.hide(LIGHT_CAM_BITS | MODEL_CAM_BITS)
        b.show(PLAIN_CAM_BITS | BLOOM_CAM_BITS)
        b.set_depth_write(False)
        b.set_pos(self.cam.get_pos(render))
        b.set_hpr(self.cam.get_hpr(render))
        b_move_by = render.get_relative_vector(self.cam, Vec3(0,0,-2))
        b.set_pos(b, b_move_by)
        b.set_color(0,0,0)
        b.set_transparency(TransparencyAttrib.MAlpha)
        #fade from full opacity to no opacity
        cintv = LerpColorScaleInterval(b, 1.5, (1,1,1,0), (1,1,1,1))
        def _unhide_ui():
            self.doc.GetElementById('content').style.display = 'block'
            b.detach_node()
        #show ui after lerp is finished
        showui = Func(_unhide_ui)
        Sequence(cintv,showui).start()



    def show_selection_screen(self):
        LoadFontFace("Ui/assets/MunroSmall.otf")
        LoadFontFace("Ui/assets/Munro.otf")
        self.r_region = RocketRegion.make('pandaRocket', self.lightbuffer)
        self.r_region.setActive(1)
        self.r_region.setSort(6)
        context = self.r_region.getContext()
        self.doc = context.LoadDocument('Ui/rml/map_test.rml')

        mlist = self.doc.GetElementById('map_select')
        for idx,item in enumerate(os.listdir('Maps')):
            fn_split = item.split('.')
            if len(fn_split) < 2 or fn_split[0] == "" or fn_split[1] != "xml":
                continue
            item_div = self.doc.CreateElement("div")
            item_div.SetAttribute("map", item)
            item_div.AddEventListener('click', self.map_selected, True)
            item_div.AppendChild(self.doc.CreateTextNode(item))
            mlist.AppendChild(item_div)

        self.doc.GetElementById('go').AddEventListener('click', self.start_map, True)
        self.doc.GetElementById('quit').AddEventListener('click', self.quit_clicked, True)
        self.doc.GetElementById('info_box').style.display = 'none'
        self.doc.GetElementById('content').style.display = 'none'
        self.doc.GetElementById('go').style.display = 'none'
        self.doc.Show()


        self.ih = RocketInputHandler()
        self.ih_node = base.mouseWatcher.attachNewNode(self.ih)
        self.r_region.setInputHandler(self.ih)

    def map_selected(self):
        in_map = event.current_element.GetAttribute("map")
        self.switch_map(in_map, show_info=True)
        return

    def switch_map(self, mapname, show_info=False, audio=True):
        if self.map:
            self.map.remove(self.render)
            del(self.map)
        if self.light_shaders:
            self.light_shaders.removeNode()
            del self.light_shaders
        if audio:
            maps = load_maps('Maps/%s' % mapname, self.cam, audio3d=self.audio3d)
        else:
            maps = load_maps('Maps/%s' % mapname, self.cam)
        near = self.cam.node().get_lens().get_near()
        self.map = maps[0]
        self.light_shaders = render.attach_new_node(PandaNode('lighting_shaders'))
        amb = self.light_shaders.attach_new_node(GeomBuilder('ambient_lights').add_rect([0,0,0,1],-5,-5,0,5,5,0).get_geom_node())
        self.light_shaders.hide(MODEL_CAM_BITS | BLOOM_CAM_BITS | PLAIN_CAM_BITS)
        self.light_shaders.show(LIGHT_CAM_BITS)
        amb.setShader(loader.loadShader("Shaders/ambient.sha"))
        amb.set_shader_input('amb', self.map.world.ambient)
        self.light_shaders.setAttrib(DepthTestAttrib.make(RenderAttrib.MLessEqual))
        self.light_shaders.setAttrib(CullFaceAttrib.make(CullFaceAttrib.MCullClockwise))
        self.light_shaders.set_pos(Vec3(0, 0, -near-near*0.01))
        self.light_shaders.reparent_to(self.cam)
        for dlight in self.map.world.dlights:
            np = self.light_shaders.attach_new_node(GeomBuilder('dir_lights').add_rect([0,0,0,1],-5,-5,0,5,5,0).get_geom_node())
            np.setShader(loader.loadShader('Shaders/directional.sha'))
            np.set_shader_input('dl', dlight)

        self.map.show(self.render)
        self.camera.setPos(*self.map.preview_cam[0])
        self.camera.setH(self.map.preview_cam[1][0])
        self.camera.setP(self.map.preview_cam[1][1])
        if self.doc and show_info:
            title_e = self.doc.GetElementById('info_title')
            title_txt = title_e.first_child
            title_e.RemoveChild(title_txt)
            new_txt = self.map.name + " -- " + self.map.author
            new_txt = new_txt.encode('latin2', 'ignore')
            title_e.AppendChild(self.doc.CreateTextNode(new_txt))
            desc_e = self.doc.GetElementById('info_content')
            desc_txt = desc_e.first_child
            desc_e.RemoveChild(desc_txt)
            desc_e.AppendChild(self.doc.CreateTextNode(self.map.description.encode('latin2', 'ignore')))
            self.doc.GetElementById('info_box').style.display = 'block'
            self.doc.GetElementById('go').style.display = 'inline'
            self.doc.GetElementById('gametitle').style.display = 'none'

    def start_map(self):
        try:
            self.r_region.setActive(0)
            self.ih_node.detach_node()
        except:
            pass
        incarn = self.map.world.get_incarn()
        walker_color_dict = {
            "barrel_color": [.7,.7,.7],
            "visor_color": [2.0/255, 94.0/255, 115.0/255],
            "body_primary_color": [3.0/255, 127.0/255, 140.0/255],
            "body_secondary_color": [217.0/255, 213.0/255, 154.0/255]
        }
        self.walker = self.map.world.attach(Walker(incarn, colordict=walker_color_dict, player=True))
        taskMgr.add(self.move, 'move')
        taskMgr.add(self.map.world.update, 'worldUpdateTask')
        self.setup_input()
        props = WindowProperties()
        props.setCursorHidden(True)
        self.win.requestProperties(props)
        print self.render.analyze()

    def quit_clicked(self):
        exit()

    def set_key(self, key, value):
        self.key_map[key] = value

    def drop_blocks(self):
        block = self.map.world.attach(FreeSolid(Block((1, 1, 1), (1, 0, 0, 1), 0.01, (0, 40, 0), (0, 0, 0)), 0.01))
        for i in range(10):
            rand_pos = (random.randint(-25, 25), 40, random.randint(-25, 25))
            block = self.map.world.attach(FreeSolid(Block((1, 1, 1), (1, 0, 0, 1), 0.01, rand_pos, (0, 0, 0)), 0.01))

    def setup_input(self):
        self.key_map = {'cam_forward': 0
                      , 'cam_left': 0
                      , 'cam_backward': 0
                      , 'cam_right': 0
                      , 'left': 0
                      , 'right': 0
                      , 'up': 0
                      , 'down': 0
                      , 'forward': 0
                      , 'backward': 0
                      , 'rotateLeft': 0
                      , 'rotateRight': 0
                      , 'walkForward': 0
                      , 'crouch': 0
                      , 'fire': 0
                      , 'missile': 0
                      , 'grenade_fire': 0
                      , 'grenade': 0
                      , 'print_cam': 0
                      }
        self.accept('escape', sys.exit)
        self.accept('p', self.drop_blocks)
        self.accept('w', self.set_key, ['cam_forward', 1])
        self.accept('w-up', self.set_key, ['cam_forward', 0])
        self.accept('a', self.set_key, ['cam_left', 1])
        self.accept('a-up', self.set_key, ['cam_left', 0])
        self.accept('s', self.set_key, ['cam_backward', 1])
        self.accept('s-up', self.set_key, ['cam_backward', 0])
        self.accept('d', self.set_key, ['cam_right', 1])
        self.accept('d-up', self.set_key, ['cam_right', 0])
        self.accept('/', self.set_key, ['print_cam', 1])
        self.accept('/-up', self.set_key, ['print_cam', 0])
        # Test walker movement
        self.accept('i',         self.set_key, ['forward', 1])
        self.accept('i-up',      self.set_key, ['forward', 0])
        self.accept('shift-i',   self.set_key, ['forward', 1])
        self.accept('shift-i-up',self.set_key, ['forward', 0])
        self.accept('j',         self.set_key, ['left', 1])
        self.accept('j-up',      self.set_key, ['left', 0])
        self.accept('shift-j',   self.set_key, ['left', 1])
        self.accept('shift-j-up',self.set_key, ['left', 0])
        self.accept('k',         self.set_key, ['backward', 1])
        self.accept('k-up',      self.set_key, ['backward', 0])
        self.accept('shift-k',   self.set_key, ['backward', 1])
        self.accept('shift-k-up',self.set_key, ['backward', 0])
        self.accept('l',         self.set_key, ['right', 1])
        self.accept('l-up',      self.set_key, ['right', 0])
        self.accept('shift-l',   self.set_key, ['right', 1])
        self.accept('shift-l-up',self.set_key, ['right', 0])
        self.accept('shift',     self.set_key, ['crouch', 1])
        self.accept('shift-up',  self.set_key, ['crouch', 0])
        self.accept('mouse1',    self.set_key, ['fire', 1])
        self.accept('mouse1-up', self.set_key, ['fire', 0])
        self.accept('shift-mouse1',self.set_key, ['fire', 1])
        self.accept('shift-mouse1-up',self.set_key, ['fire', 0])
        self.accept('u',         self.set_key, ['missile', 1])
        self.accept('u-up',      self.set_key, ['missile', 0])
        self.accept('shift-u',   self.set_key, ['missile', 1])
        self.accept('shift-u-up',self.set_key, ['missile', 0])
        self.accept('o',         self.set_key, ['grenade_fire', 1])
        self.accept('o-up',      self.set_key, ['grenade_fire', 0])
        self.accept('shift-o',   self.set_key, ['grenade_fire', 1])
        self.accept('shift-o-up',self.set_key, ['grenade_fire', 0])
        self.accept('m',         self.set_key, ['grenade', 1])
        self.accept('m-up',      self.set_key, ['grenade', 0])
        self.accept('shift-m',   self.set_key, ['grenade', 1])
        self.accept('shift-m-up',self.set_key, ['grenade', 0])

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

        if (self.key_map['cam_forward']):
            self.camera.setZ(self.camera, -25 * dt)
        if (self.key_map['cam_backward']):
            self.camera.setZ(self.camera, 25 * dt)
        if (self.key_map['cam_left']):
            self.camera.setX(self.camera, -25 * dt)
        if (self.key_map['cam_right']):
            self.camera.setX(self.camera, 25 * dt)
        if (self.key_map['print_cam']):
            print "CAMERA: Pos - %s, Hpr - %s" % (self.camera.get_pos(), self.camera.get_hpr())
            self.key_map['print_cam'] = 0

        self.walker.handle_command('forward', self.key_map['forward'])
        self.walker.handle_command('left', self.key_map['left'])
        self.walker.handle_command('backward', self.key_map['backward'])
        self.walker.handle_command('right', self.key_map['right'])
        self.walker.handle_command('crouch', self.key_map['crouch'])

        self.walker.handle_command('fire', self.key_map['fire'])
        if self.key_map['fire']: self.key_map['fire'] = 0
        self.walker.handle_command('missile', self.key_map['missile'])
        if self.key_map['missile']: self.key_map['missile'] = 0
        self.walker.handle_command('grenade_fire', self.key_map['grenade_fire'])
        if self.key_map['grenade_fire']: self.key_map['grenade_fire'] = 0
        self.walker.handle_command('grenade', self.key_map['grenade'])
        if self.key_map['grenade']: self.key_map['grenade'] = 0

        return task.cont

    def make_filter_buffer(self, srcbuffer, name, sort, prog):
        blur_buffer = self.win.make_texture_buffer(name, 1280, 720)
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
    m = MapTest()
    m.run()
