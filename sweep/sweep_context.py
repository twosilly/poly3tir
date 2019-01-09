import sys
sys.path.append(r"../")
from common import  *
#from sweep_context import kAlpha,SweepContext
from advancing_front import Node,AdvancingFront
#初始三角形因子，种子三角形将扩展30%
#左侧和右侧的点集宽度
kAlpha = 0.3
class SweepContext(object):
    class Basin():
        def __init__(self):
            self.left_node = None
            self.bottom_node = None
            self.right_node = None
            self.width = 0.0
            self.left_highest = False
        def Clear(self):
            self.left_node = None
            self.bottom_node = None
            self.right_node = None
            self.width = 0.0
            self.left_highest = False
    class EdgeEvent():
        def __init__(self):
            self.constrained_edge = None
            self.right = None
    def __init__(self, polyline):
        self.front_ = None
        self.head_ = None
        self.tail_ = None
        self.af_head_ = None
        self.af_middle_ = None
        self.af_tail_ = None

        self.basin = self.Basin();
        self.edge_event = self.EdgeEvent();

        self.points_ = polyline;
        self.edge_list = []
        self.map_ = []

        self.InitEdges(self.points_);
    #FIXME:有点问题的
    def __del__(self):
        self.front_ = None
        self.head_ = None
        self.tail_ = None
        self.af_head_ = None
        self.af_middle_ = None
        self.af_tail_ = None
        self.triangles_ = []
        self.map_ = []
        self.points_ = []

    def set_head(self,p1):
        self.head_ = p1
    def head(self):
        return self.head_
    def set_tail(self,p1):
        self.tail_ = p1
    def tail(self):
        return self.tail_
    def point_count(self):
        return len(self.points_)
    def LocateNode(self,point):
        #TODO:实现搜索树
        return self.front_.LocateNode(point.X)

    #FIXME:这里绝对有问题
    def RemoveNode(self,node):
        del node
    def CreateAdvancingFront(self,nodes):
        #// 初始三角形
        triangle = Triangle3(self.points_[0], self.tail_, self.head_);

        self.map_.append(triangle);
        node1 = Node(triangle.GetPoint(1))
        node0 = Node(triangle.GetPoint(0))
        self.af_head_   = node1.Node2(triangle.GetPoint(1), triangle);
        self.af_middle_ = node0.Node2(triangle.GetPoint(0), triangle);
        self.af_tail_   = Node(triangle.GetPoint(2));
        self.front_     = AdvancingFront(self.af_head_, self.af_tail_);

        # TODO: 如果头是中间的下一个而不是前一个，会更直观吗?
        # 所以交换头和尾
        self.af_head_.next = self.af_middle_;
        self.af_middle_.next = self.af_tail_;
        self.af_middle_.prev = self.af_head_;
        self.af_tail_.prev = self.af_middle_;
        self.edge_list = []
    
    #试着将一个节点映射到这个三角形中没有邻居的所有边
    def MapTriangleToNodes(self,t):
        for i in range(3):
            if not t.GetNeighbor(i):
                n = self.front_.LocatePoint(t.PointCW(t.GetPoint(i)))
                if(n):
                    n.triangle = t

    def AddToMap(self,triangle):
        self.map_.append(triangle)

    def GetPoint(self,index):
        return self.points_[index]

    def GetPoints(self):
        return self.points_
    #FIXME:可能有问题
    def RemoveFromMap(self,triangle):
        self.map_.remove(triangle)

    def AddHole(self,polyline):
        self.InitEdges(polyline);
        for i in range(0,len(polyline)):
            self.points_.append(polyline[i])
    def AddPoint(self,point):
        self.points_.append(point)
    def front(self):
        return self.front_
    def MeshClean(self,triangle):
        if (triangle != None and not triangle.IsInterior()):
            triangle.IsInterior(True);
            self.triangles_.append(triangle);
            for i in range(3):
                if (not triangle.constrained_edge[i]):
                    self.MeshClean(triangle.GetNeighbor(i));

    def GetTriangles(self):
        return self.triangles_
    def GetMap(self):
        return self.map_
    def InitTriangulation(self):
        xmax = (self.points_[0].X)
        xmin = (self.points_[0].X);
        ymax = (self.points_[0].Y)
        ymin = (self.points_[0].Y);
        zmax = (self.points_[0].Z)
        zmin = (self.points_[0].Z);

        # 计算范围。

        for i in range(len(self.points_)):
            p = self.points_[i];
            if (p.X > xmax):
                xmax = p.X;
            if (p.X < xmin):
                xmin = p.X;
            if (p.Y > ymax):
                ymax = p.Y;
            if (p.Y < ymin):
                ymin = p.Y;
            if (p.Z > zmax):
                zmax = p.Y;
            if (p.Z < zmin):
                zmin = p.Y;


        dx = kAlpha * (xmax - xmin);
        dy = kAlpha * (ymax - ymin);
        zy = kAlpha * (zmax - zmin);
        self.head_ = Point3(xmax + dx, ymin - dy, zmin - zy);
        self.tail_ = Point3(xmin - dx, ymin - dy, zmin - zy);

        def cmp(a,b):
            if a.Y > b.Y:
                return True
            elif (a.Y == b.Y):
                # 确保q是x值更大的点
                if (a.X < b.X):
                    return True;
            return False

        # 沿y轴排序
        self.points_.sort(key = (lambda x:x.Y))
        #std::sort(points_.begin(), points_.end(), cmp);
    def InitEdges(self,polyline):
        num_points = len(polyline) - 1
        for i in range(num_points):
            j =  i+1 if i < num_points else 0
            self.edge_list.append(Edge3(polyline[i],polyline[j]))


