#import Node
#import Edge3
import math

def cmp(a,b):
    if a.Y > b.Y:
        return True
    elif (a.Y == b.Y):
        # 确保q是x值更大的点
        if (a.X < b.X):
            return True;
    return False
#在两个向量上做点积。
def Dot(a,b):
    return  a.X * b.X + a.Y * b.Y + a.Z * b.Z;
#对两个向量做叉乘。在二维中这产生一个标量。 在三维中这产生一个向量点
#(ax,ay,az)x(bx,by,bz)=(aybz-azby,azbx-axbz,axby-aybx)
def Cross(a,b):
    if isinstance(b,Point3) and isinstance(a,Point3):
        return Point3(a.Y * b.Z - a.Z * b.Y, a.Z*b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);
    if not isinstance(a,Point3):#对一个标量和一个点做叉乘。在2D中，这会产生一个点。
        return Point3(-b * a.Y, b * a.X, a.Z);
    return Point3(b * a.Y, -b * a.X, a.Z);

class Point3(object):
    def __init__(self,x,y,z):
        self.X = x
        self.Y = y
        self.Z = z
        #这一点构成了一个上端点
        self.edge_list = []

    #将这个点设置为所有的0
    def set_zero(self):
        self.X = 0.0
        self.Y = 0.0
        self.Z = 0.0

    #将这个点设置为某个指定的坐标。
    def set(self,x,y,z):
        self.X = x
        self.Y = y
        self.Z = z

    def __str__(self):
        return "Point3("+str(self.X) +","+str(self.Y)+"," +str(self.Z) +")"
    #减法
    def __sub__(self, other):
        self.X -= other.X
        self.Y -= other.Y
        self.Z -= other.Z
        return self
    #加法
    def __add__(self, other):
        self.X += other.X
        self.Y += other.Y
        self.Z += other.Z
        return self
    #用标量乘以这个点
    def __mul__(self, other):
        self.X *= other
        self.Y *= other
        self.Z *= other
        return self

    def __eq__(self,other):
        return  self.X == other.X and self.Y == other.Y and self.Z == other.Z

    #得到这个点的长度
    def Length(self):
        return math.sqrt(math.pow(self.X,2) + math.pow(self.Y,2) + math.pow(self.Z,2) )

    #把这个点转换成一个单位点。返回长度
    def Normalize(self):
        len = self.Length()
        self.X /= len
        self.Y /= len
        self.Z /= len
        return len

#表示简单多边形的边缘
#TODO:虽然标了一个3但是其实还是二维运行，只是会带着一个z的坐标值，因为我们的是等高线，所以需要这个
class Edge3():
    def __init__(self,p = Point3(0,0,0),q= Point3(0,0,0)):
        self._p = p
        self._q = q
        self.compute_init(self._p,self._q)
        
    def compute_init(self,p1,p2):
        if p1.Y > p2.Y:
            self._p = p1
            self._q = p2
        elif p1.Y == p2.Y:
            if p1.X > p2.X:
                self._q = p1
                self._p = p2
            elif p1.X == p2 .X:
                #重复点
                raise RuntimeError('Repeat points')
        self._q.edge_list.append(self)

#基于三角形的数据结构比基于四边的数据结构具有更好的性能
#参见:J. Shewchuk， "Triangle3:工程2D质量网格发生器和Delaunay三角仪"
#"Triangulations in CGAL"
class Triangle3(object):
    def __init__(self,a,b,c):
        #Triangle3 points
        self.points_ =  [0 for i in range(3)] #这里只是为了占位
        self.points_[0] = a
        self.points_[1] = b
        self.points_[2] = c
        #邻区列表#Neighbor list 邻区列表
        self.neighbors_ = [None for i in range(3)]
        #这个三角形标记为内部三角形了吗?
        self.interior_ = False
        #标记以确定边缘是否是受约束的边缘
        self.constrained_edge = [False for i in range(3)]
        #标记以确定边缘是否为Delauney边缘
        self.delaunay_edge = [False for i in range(3)]

    def GetPoint(self,index_):
        return self.points_[index_]
    #逆时针指向给定点的点
    def PointCW(self, point):
        if self.points_[0] == point:
            return self.points_[2]
        elif self.points_[1] == point:
            return self.points_[0]
        elif self.points_[2] == point:
            return self.points_[1]
        raise RuntimeError('PointCW Error')
    #逆时针指向给定点的点
    def PointCCW(self, point):
        if self.points_[0] == point:
            return self.points_[1]
        elif self.points_[1] == point:
            return self.points_[2]
        elif self.points_[2] == point:
            return self.points_[0]
        raise RuntimeError('PointCCW Error')

    def OppositePoint(self , t,  p):
        cw = t.PointCW(p)
        return self.PointCW(cw)

    def GetNeighbor(self,index):
        return self.neighbors_[index]

    # 更新邻居指针
    def MarkNeighbor(self,p1,p2,t):
        if (p1 == self.points_[2] and p2 == self.points_[1]) or (p1 == self.points_[1] and p2 == self.points_[2]):
            self.neighbors_[0] = t
        elif (p1 == self.points_[0] and p2 == self.points_[2]) or (p1 == self.points_[2] and p2 == self.points_[0]):
            self.neighbors_[1] = t
        elif (p1 == self.points_[0] and p2 == self.points_[1]) or (p1 == self.points_[1] and p2 == self.points_[0]):
            self.neighbors_[2] = t
        else:
            raise RuntimeError('MarkNeighbor Error')
        #穷举搜索更新邻居指针
    def MarkNeighbor(self, t):
        if t.Contains2(self.points_[1], self.points_[2]):
            self.neighbors_[0] =  t;
            t.MarkNeighbor(self.points_[1], self.points_[2], self);

        elif t.Contains2(self.points_[0], self.points_[2]):
            self.neighbors_[1] =  t;
            t.MarkNeighbor(self.points_[0], self.points_[2], self);
        elif t.Contains2(self.points_[0], self.points_[1]):
            self.neighbors_[2] = t;
            t.MarkNeighbor(self.points_[0], self.points_[1], self);


    def Contains(self, p):
        return p == self.points_[0] or p == self.points_[1] or p == self.points_[2]

    def ContainsEdge(self, e):
        return self.Contains(e.p) and self.Contains(e.q)

    def Contains2(self , p,  q):
        return self.Contains(p) and self.Contains(q)

    def MarkConstrainedEdge(self,index):
        self.constrained_edge[index] = True;

    def MarkConstrainedEdge(self,edge):
        self.MarkConstrainedEdge(edge.p, edge.q)

    def MarkConstrainedEdge2(self,p,q):
        if ((q == self.points_[0] and p == self.points_[1]) or (q == self.points_[1] and p == self.points_[0])) :
            self.constrained_edge[2] = True;
        elif ((q == self.points_[0] and p == self.points_[2]) or (q == self.points_[2] and p == self.points_[0])) :
            self.constrained_edge[1] = True;
        elif ((q == self.points_[1] and p == self.points_[2]) or (q == self.points_[2] and p == self.points_[1])) :
            self.constrained_edge[0] = True;

    def Index(self, p):
        if p == self.points_[0]:
            return 0;
        elif (p == self.points_[1]):
            return 1;
        elif (p == self.points_[2]):
            return 2;
        raise RuntimeError('Index Error')


    def EdgeIndex(self,p1,p2):
        if (self.points_[0] == p1):
            if (self.points_[1] == p2):
                return 2;
            elif (self.points_[2] == p2):
                 return 1;
            
        elif (self.points_[1] == p1):
            if (self.points_[2] == p2):
                return 0;
            elif (self.points_[0] == p2):
                return 2;

        elif (self.points_[2] == p1):
            if (self.points_[0] == p2):
                return 1;
            elif (self.points_[1] == p2):
                return 0;

        return -1;
    #邻边顺时针指向给定的点
    def NeighborCW(self,point):
        if (point == self.points_[0]):
            return self.neighbors_[1];
        elif (point == self.points_[1]):
            return self.neighbors_[2];
        return self.neighbors_[0];
    
     #对给定的点逆时针旋转
    def NeighborCCW(self,point):
        if (point == self.points_[0]):
            return self.neighbors_[2];
        elif (point == self.points_[1]):
            return self.neighbors_[0];
        return self.neighbors_[1];

    def GetConstrainedEdgeCCW(self,p):
        if ( p == self.points_[0]):
            return self.constrained_edge[2];
        elif (p == self.points_[1]):
            return self.constrained_edge[0];
        return self.constrained_edge[1];

    def GetConstrainedEdgeCW(self,p):
        if ( p == self.points_[0]):
            return self.constrained_edge[1];
        elif ( p == self.points_[1]):
            return self.constrained_edge[2];
        return self.constrained_edge[0];

    def SetConstrainedEdgeCCW(self,p,ce):
        if ( p == self.points_[0]):
            self.constrained_edge[2] = ce;
        elif (  p == self.points_[1]):
            self.constrained_edge[0] = ce;
        else:
            self.constrained_edge[1] = ce;

    def SetConstrainedEdgeCW(self,p,ce):
        if (p == self.points_[0]):
            self.constrained_edge[1] = ce;
        elif ( p == self.points_[1]):
            self.constrained_edge[2] = ce;
        else:
            self.constrained_edge[0] = ce;

    def GetDelunayEdgeCCW(self,p):
        if (p == self.points_[0]):
            return self.delaunay_edge[2];
        elif (p == self.points_[1]):
            return self.delaunay_edge[0];
        return self.delaunay_edge[1];
    def GetDelunayEdgeCW(self,p):
        if (p == self.points_[0]):
            return self.delaunay_edge[1];
        elif (p == self.points_[1]):
            return self.delaunay_edge[2];
        return self.delaunay_edge[0];

    def SetDelunayEdgeCCW(self,p,e):
        if (p == self.points_[0]):
            self.delaunay_edge[2] = e;
        elif (  p == self.points_[1]):
            self.delaunay_edge[0] = e;
        else:
            self.delaunay_edge[1] = e;

    def SetDelunayEdgeCCW(self,p,e):
        if (p == self.points_[0]):
            self.delaunay_edge[2] = e;
        elif (  p == self.points_[1]):
            self.delaunay_edge[0] = e;
        else:
            self.delaunay_edge[1] = e;

    #到给定点的邻居
    def SetDelunayEdgeCW(self,p,e):
        if (p == self.points_[0]):
            self.delaunay_edge[1] = e;
        elif (  p == self.points_[1]):
            self.delaunay_edge[2] = e;
        else:
            self.delaunay_edge[0] = e;

    #绕点(0)顺时针旋转合法化三角形
    def Legalize(self,point):
        self.points_[1] = self.points_[0];
        self.points_[0] = self.points_[2];
        self.points_[2] = point;

    #顺时针旋转使triagnle合法化
    def Legalize(self,opoint ,npoint):
        if (opoint == self.points_[0]):
            self.points_[1] = self.points_[0];
            self.points_[0] = self.points_[2];
            self.points_[2] = npoint;
        elif ( opoint == self.points_[1]):
            self.points_[2] = self.points_[1];
            self.points_[1] = self.points_[0];
            self.points_[0] = npoint;
        elif ( opoint == self.points_[2]):
            self.points_[0] = self.points_[2];
            self.points_[2] = self.points_[1];
            self.points_[1] = npoint;
        else:
            raise RuntimeError('Legalize Error')

    def ClearNeighbors(self):
        self.neighbors_[0] = None;
        self.neighbors_[1] = None;
        self.neighbors_[2] = None;

    def ClearDelunayEdges(self):
        self.delaunay_edge[0] = False;
        self.delaunay_edge[1] = False;
        self.delaunay_edge[2] = False;

    def IsInterior(self):
        return self.interior_

    def IsInterior(self, b):
        self.interior_ = b;

    #到给定点的邻居
    def NeighborAcross(self,opoint):
        if (opoint == self.points_[0]):
              return self.neighbors_[0];
        elif (opoint == self.points_[1]):
            return self.neighbors_[1]
        return self.neighbors_[2];

    def __str__(self):
        str = ''
        str += str(self.points_[0].X)+","+str(self.points_[0].Y) +"," +str(self.points_[0].Z)+"\n";
        str += str(self.points_[1].X) + "," + str(self.points_[1].Y) + "," + str(self.points_[1].Z) + "\n";
        str += str(self.points_[2].X) + "," + str(self.points_[2].Y) + "," + str(self.points_[2].Z) + "\n";
        return str

