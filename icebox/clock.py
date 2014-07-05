from panda3d.core import *
from icebox.world import World
from icebox.network import Server, Client
from icebox.local_player import LocalPlayer
from icebox.input_manager import InputManager

class Clock (object):
    def __init__(self, showbase, is_client):
        render = showbase.render
        self.time = 0
        self.world = World(showbase, is_client)
        if is_client:
            self.local_player = LocalPlayer()
            self.client = Client(self.world, 'localhost', 23000)
            self.input_manager = InputManager(showbase, self.local_player, self.client)
        else:
            self.server = Server(self.world, 23000)
            taskMgr.doMethodLater(5, self.drop_block, 'drop_block_task')

    def update(self, task):
        dt = globalClock.getDt()
        self.time += dt
        self.world.update(dt)
        return task.cont

    def drop_block(self, task):
        self.world.add_block([0,25,0])
        return task.again