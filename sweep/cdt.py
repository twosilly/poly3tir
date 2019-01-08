from sweep_context import kAlpha,SweepContext
from advancing_front import Node,AdvancingFront
from sweep import Sweep
class CDT(object):
    def __init__(self,polyline):
        self.sweep_context_ = SweepContext(polyline)
        self.sweep_ = Sweep()
    def __del__(self):
        self.sweep_context_ = None
        self.sweep_ = None
    #TODO:添加洞
    def AddHole(self,polyline):
        self.sweep_context_.AddHole(polyline)

    #TODO:添加一个点
    def AddPoint(self,point):
        self.sweep_context_.AddPoint(point);
    #TODO:三角形点
    def Triangulate(self):
        self.sweep_.Triangulate(self.sweep_context_);
    #TODO:得到Delaunay 三角形
    def GetTriangles(self):
        return self.sweep_context_.GetTriangles();
    #TODO:得到三角形map
    def GetMap(self):
        return  self.sweep_context_.GetMap();

