import sys
sys.path.append(r"../")
from common import  *

import math

class Sweep(object):
    def __init__(self):
        self.nodes_ = None
    #三角形简单多边形与孔
    def Triangulate(self, tcx):
        tcx.InitTriangulation();
        tcx.CreateAdvancingFront(self.nodes_);
        # 扫描点;建立网
        self.SweepPoints(tcx);
        # 清除
        self.FinalizationPolygon(tcx);
    def SweepPoints(self,tcx):
        for i in range(len(tcx)):
            point = tcx.GetPoint(i);
            node = self.PointEvent(tcx, point);
            for i in range(len(point.edge_list)):
                self.EdgeEvent(tcx, point.edge_list[i], node);
    """
    /**
     *查找新点左侧的close节点并创建一个新三角形。如果需要，新的孔和盆地将填补。
     *
     * @param tcx
     * @param point
     * @return
     */
    """
    def PointEvent(self, tcx,point):
        node = tcx.LocateNode(point);
        new_node = self.NewFrontTriangle(tcx, point, node);
        #只需要检查+，因为点的x值不会小于节点，这是由于我们从前面获取节点的方式
        if point.X <= node.point.X + EPSILON:
            self.Fill(tcx,node)
        #tcx.AddNode(new_node);
        self.FillAdvancingFront(tcx, new_node);
        return new_node;
    
    def EdgeEvent(self,tcx,edge,node):
        tcx.edge_event.constrained_edge = edge;
        tcx.edge_event.right = (edge.p.X > edge.q.X);

        if (self.IsEdgeSideOfTriangle(node.triangle, edge.p, edge.q)):
            return;


        # 现在我们将做所有需要的填充
        # TODO: 与flip process集成可能会带来更好的性能，但目前这避免了同时需要flip和fill的情况
        self.FillEdgeEvent(tcx, edge, node);
        self.EdgeEvent(tcx, edge.p, edge.q, node.triangle, edge.q);
    def EdgeEvent(self,tcx,ep,eq,triangle,point):
        if (self.IsEdgeSideOfTriangle(*triangle, ep, eq)):
            return;

        p1 = triangle.PointCCW(point);
        o1 = Orient2d(eq, *p1, ep);
        if (o1 == Orientation.COLLINEAR):
            print("A %f %f   %f %f    %f %f \n"%(eq.X, eq.y, p1.X, p1.Y, ep.X, ep.Y));
            raise RuntimeError("EdgeEvent - Collinear not supported")

        p2 = triangle.PointCW(point);
        o2 = Orient2d(eq, *p2, ep);
        if (o2 == Orientation.COLLINEAR):
            print("A %f %f   %f %f    %f %f \n" % (eq.X, eq.y, p1.X, p1.Y, ep.X, ep.Y));
            raise RuntimeError("EdgeEvent - Collinear not supported")



        if (o1 == o2):
            # 需要决定我们是旋转CW还是CCW来得到一个三角形的交叉边
            if (o1 == Orientation.CW):
                triangle = triangle.NeighborCCW(point);
            else:
                triangle = triangle.NeighborCW(point);
            self.EdgeEvent(tcx, ep, eq, triangle, point);
        else:
            # 这个三角形交叉约束，让我们开始吧!
            self.FlipEdgeEvent(tcx, ep, eq, triangle, point);
    #FIXME:貌似有点问题
    def NewFrontTriangle(self,tcx,point,node):
        triangle = Triangle3(point, *node.point, *node.next.point);

        triangle.MarkNeighbor(*node.triangle);
        tcx.AddToMap(triangle);
        new_node = Node(point);
        self.nodes_.append(new_node);

        new_node.next = node.next;
        new_node.prev = node;
        node.next.prev = new_node;
        node.next = new_node;

        if (not self.Legalize(tcx, * triangle)):
            tcx.MapTriangleToNodes( * triangle);

        return new_node;

    """
    /**
     * 在前进的前方添加一个三角形来填充一个洞。
     * @param tcx
     * @param 节点-中间的节点，这是洞的底部
     */
    """
    def Fill(self,tcx,node):
        triangle = Triangle3(node.prev.point, node.point, node.next.point);

        # TODO: 应该从邻居三角形复制constrained_edge值吗
        #       现在，constrained_edge值将在合法化过程中复制
        triangle.MarkNeighbor(node.prev.triangle);
        triangle.MarkNeighbor(node.triangle);

        tcx.AddToMap(triangle);

        # 更新前沿
        node.prev.next = node.next;
        node.next.prev = node.prev;

        # 如果它是合法的，三角形已经被映射
        if (not self.Legalize(tcx, triangle)):
            tcx.MapTriangleToNodes(triangle)
    #如果三角形合法，返回true
    def Legalize(self,tcx,t):
        # 为了使三角形合法化，我们首先要找出三条边中是否有一条违反了德劳内条件
        for i in range(3):
            if t.delaunay_edge[i]:
                continue
            ot = t.GetNeighbor(i);
            if ot:
                p = t.GetPoint(i);
                op = ot.OppositePoint(t, *p);
                oi = ot.Index(op);
                # 如果这是一个受限的Edge3或Delaunay Edge3(仅在递归合法期间)
                # 那么我们就不应该试图合法化
                if (ot.constrained_edge[oi] or ot.delaunay_edge[oi]):
                    t.constrained_edge[i] = ot.constrained_edge[oi];
                    continue;
                inside = self.Incircle(*p, *t.PointCCW(*p), *t.PointCW(*p), *op);
                if inside:
                    # 让我们将这条共享边标记为Delaunay
                    t.delaunay_edge[i] = True;
                    ot.delaunay_edge[oi] = True;

                    # 让我们旋转共享边一个顶点CW使其合法化
                    self.RotateTrianglePair(t, *p, *ot, *op);

                    # 我们现在得到了一个有效的Delaunay边3由两个三角形共享这给了我们4条新边来检查Delaunay

                    # 确保对特定的三角形只进行一次三角形到节点的映射
                    not_legalized = not self.Legalize(tcx, t);
                    if (not_legalized):
                        tcx.MapTriangleToNodes(t);
                        # 重置Delaunay边，因为它们只在我们添加新的三角形或点之前是有效的。
                        # XXX: 需要考虑这个问题。在我们返回到之前的递归级别之后，可以尝试这些边吗
                        t.delaunay_edge[i] = False;
                        ot.delaunay_edge[oi] = False;
                        # 如果三角形已经合法化了，就不需要检查其他边，因为递归合法化将处理这些边，所以我们可以在这里结束。
                        return True;
        return False;


    """
    /**
     * <b>Requirement</b>:<br>
     * 1. a、b和c组成一个三角形.<br>
     * 2. 已知a和d在bc的对边<br>
     * <pre>
     *                a
     *                +
     *               / \
     *              /   \
     *            b/     \c
     *            +-------+
     *           /    d    \
     *          /           \
     * </pre>
     * <b>Fact</b>: d必须在B区域才有机会进入由a B c组成的圆内<br>
     * 如果orient2d(a, B,d)或orient2d(c,a,d)为CW，则d在B之外<br>
     *  这一先验知识为我们提供了一种优化内圆检验的方法
     * @param a - 三角形对边d
     * @param b - triangle point
     * @param c - triangle point
     * @param d - point opposite a
     * @return 如果d在圆内，则为真;如果在圆边，则为假
     */
    """
    def Incircle(self,pa,pb,pc,pd):
        adx = pa.X - pd.X
        ady = pa.Y - pd.Y;
        bdx = pb.X - pd.X;
        bdy = pb.Y - pd.Y;

        adxbdy = adx * bdy;
        bdxady = bdx * ady;
        oabd = adxbdy - bdxady;

        if (oabd <= 0):
            return False;

        cdx = pc.X - pd.X;
        cdy = pc.Y - pd.Y;

        cdxady = cdx * ady;
        adxcdy = adx * cdy;
        ocad = cdxady - adxcdy;

        if (ocad <= 0):
            return False;

        bdxcdy = bdx * cdy;
        cdxbdy = cdx * bdy;

        alift = adx * adx + ady * ady;
        blift = bdx * bdx + bdy * bdy;
        clift = cdx * cdx + cdy * cdy;

        det = alift * (bdxcdy - cdxbdy) + blift * ocad + clift * oabd;

        return det > 0;

    """
    /**
     * 旋转三角形对一个顶点CW
     *<pre>
     *       n2                    n2
     *  P +-----+             P +-----+
     *    | t  /|               |\  t |
     *    |   / |               | \   |
     *  n1|  /  |n3           n1|  \  |n3
     *    | /   |    after CW   |   \ |
     *    |/ oT |               | oT \|
     *    +-----+ oP            +-----+
     *       n4                    n4
     * </pre>
     */
    """
    def RotateTrianglePair(self,t,p,ot,op):
        #Triangle3 * n1, *n2, *n3, *n4;
        n1 = t.NeighborCCW(p);
        n2 = t.NeighborCW(p);
        n3 = ot.NeighborCCW(op);
        n4 = ot.NeighborCW(op);

        ce1 = t.GetConstrainedEdgeCCW(p);
        ce2 = t.GetConstrainedEdgeCW(p);
        ce3 = ot.GetConstrainedEdgeCCW(op);
        ce4 = ot.GetConstrainedEdgeCW(op);

        de1 = t.GetDelunayEdgeCCW(p);
        de2 = t.GetDelunayEdgeCW(p);
        de3 = ot.GetDelunayEdgeCCW(op);
        de4 = ot.GetDelunayEdgeCW(op);

        t.Legalize(p, op);
        ot.Legalize(op, p);

        # 重新映射delaunay_edge
        ot.SetDelunayEdgeCCW(p, de1);
        t.SetDelunayEdgeCW(p, de2);
        t.SetDelunayEdgeCCW(op, de3);
        ot.SetDelunayEdgeCW(op, de4);

        # ：重新映射constrained_edge
        ot.SetConstrainedEdgeCCW(p, ce1);
        t.SetConstrainedEdgeCW(p, ce2);
        t.SetConstrainedEdgeCCW(op, ce3);
        ot.SetConstrainedEdgeCW(op, ce4);

        # 重新映射的邻居
        # XXX: 可以通过跟踪旋转后应该分配给哪个邻居的边来优化标记邻居。
        # #现在mark邻居做了很多测试来找到正确的边。
        t.ClearNeighbors();
        ot.ClearNeighbors();
        if (n1): ot.MarkNeighbor( * n1);
        if (n2): t.MarkNeighbor( * n2);
        if (n3): t.MarkNeighbor( * n3);
        if (n4): ot.MarkNeighbor( * n4);
        t.MarkNeighbor(ot);
        #填补前进前线的漏洞
    def FillAdvancingFront(self,tcx,n):
        # 填写正确的洞
        node = n.next;

        while (node.next):
            angle = HoleAngle( * node);
            if (angle > M_PI_2 or angle < -M_PI_2):
                break;
                self.Fill(tcx, *node);
                node = node.next;
        # 填满了洞
        node = n.prev;

        while (node.prev):
            angle = HoleAngle( * node);
            if (angle > M_PI_2 or angle < -M_PI_2):
                break;
                self.Fill(tcx, *node);
                node = node.prev;

        # 填写正确的盆地
        if (n.next and n.next.next):
            angle = BasinAngle(n);
            if (angle < PI_3div4):
                self.FillBasin(tcx, n);

    """
    /**
     *
     * @param node - 中间节点
     * @return 3个前结点之间的夹角
     */
    """
    def HoleAngle(self,node):
        """
          /* Complex plane
           * ab = cosA +i*sinA
           * ab = (ax + ay*i)(bx + by*i) = (ax*bx + ay*by) + i(ax*by-ay*bx)
           * atan2(y,x) computes the principal value of the argument function
           * applied to the complex number x+iy
           * Where x = ax*bx + ay*by
           *       y = ax*by - ay*bx
           */
        """
        ax = node.next.point.X - node.point.X;
        ay = node.next.point.Y - node.point.Y;
        bx = node.prev.point.X - node.point.X;
        by = node.prev.point.Y - node.point.Y;

        return math.atan2(ax * by - ay * bx, ax * bx + ay * by);


    def BasinAngle(self,node):
        ax = node.point.X - node.next.next.point.X;
        ay = node.point.Y - node.next.next.point.Y;
        return math.atan2(ay, ax);

    """
    /**
     * 填充在给定节点右侧前进前沿形成的盆地。
     * 首先，我们确定一个左、下、右节点
    *盆地边界。然后我们做一个定量填充。
     *
     * @param tcx
     * @param node - 开始节点，这个或下一个节点将是左节点
     */
    """
    def FillBasin(self,tcx,node):
        if (Orient2d(node.point, node.next.point, node.next.next.point) == Orientation.CCW):
            tcx.basin.left_node = node.next.next;
        else:
            tcx.basin.left_node = node.next;
        # 找到底部和右侧节点
        tcx.basin.bottom_node = tcx.basin.left_node;
        while (tcx.basin.bottom_node.next
        and tcx.basin.bottom_node.point.Y >= tcx.basin.bottom_node.next.point.Y):
            tcx.basin.bottom_node = tcx.basin.bottom_node.next;

        if (tcx.basin.bottom_node == tcx.basin.left_node):
        # 没有有效的盆地
            return;


        tcx.basin.right_node = tcx.basin.bottom_node;
        while (tcx.basin.right_node.next
        and tcx.basin.right_node.point.Y < tcx.basin.right_node.next.point.Y):
            tcx.basin.right_node = tcx.basin.right_node.next;
        if (tcx.basin.right_node == tcx.basin.bottom_node):
            # No valid basins
            return;


        tcx.basin.width = tcx.basin.right_node.point.X - tcx.basin.left_node.point.X;
        tcx.basin.left_highest = tcx.basin.left_node.point.Y > tcx.basin.right_node.point.Y;

        self.FillBasinReq(tcx, tcx.basin.bottom_node);
    """
    /**
     * 用三角形填充盆地的递归算法
     *
     * @param tcx
     * @param node - bottom_node
     * @param cnt - 用于交换偶数和奇数的计数器
     */
    """
    def FillBasinReq(self,tcx,node):
        # 如果浅层停止充填
        if (self.IsShallow(tcx, node)):
            return;


        self.Fill(tcx, *node);

        if (node.prev == tcx.basin.left_node and node.next == tcx.basin.right_node):
            return;
        elif (node.prev == tcx.basin.left_node):
            o = Orient2d(node.point, node.next.point, node.next.next.point)
            if (o == Orientation.CW):
                return;
            node = node.next;
        elif (node.next == tcx.basin.right_node):
            o = Orient2d( node.point, node.prev.point, node.prev.prev.point);
            if (o == Orientation.CCW):
                return;
            node = node.prev;
        else:
            # 继续使用Y值最小的邻居节点
            if (node.prev.point.Y < node.next.point.Y):

                node = node.prev;
            else:
                node = node.next;  
        self.FillBasinReq(tcx, node);
    
    def IsShallow(self,tcx, node):
        height = 0.0

        if (tcx.basin.left_highest):
            height = tcx.basin.left_node.point.Y - node.point.Y;
        else :
            height = tcx.basin.right_node.point.Y - node.point.Y;

        # 如果浅层停止充填
        if (tcx.basin.width > height):
            return True;

        return False;

    def IsEdgeSideOfTriangle(self,triangle,ep,eq):
        index = triangle.EdgeIndex( ep, eq);

        if (index != -1):
            triangle.MarkConstrainedEdge(index);
            t = triangle.GetNeighbor(index);
            if (t) :
                t.MarkConstrainedEdge( ep, eq);

            return True;

        return False;

    def FillEdgeEvent(self,tcx,edge,node):
        if (tcx.edge_event.right):
            self.FillRightAboveEdgeEvent(tcx, edge, node);
        else:
            self.FillLeftAboveEdgeEvent(tcx, edge, node);


    def FillRightAboveEdgeEvent(self,tcx,edge,node):
        while (node.next.point.X < edge.p.X):
            # 检查下一个节点是否在边缘以下
            if (Orient2d(  edge.q, node.next.point, * edge.p) == Orientation.CCW):
                    self.FillRightBelowEdgeEvent(tcx, edge, node);
            else:
                node = node.next;
                
    def FillRightBelowEdgeEvent(self,tcx,edge,node):
        if (node.point.X < edge.p.X):
            if (Orient2d(node.point, node.next.point, node.next.next.point) == Orientation.CCW):
                # 凹面
                self.FillRightConcaveEdgeEvent(tcx, edge, node);
            else:
                # 凸面体
                self.FillRightConvexEdgeEvent(tcx, edge, node);
                # 重试这个
                self.FillRightBelowEdgeEvent(tcx, edge, node);

    def FillRightConcaveEdgeEvent(self,tcx,edge,node):
        self.Fill(tcx, node.next);
        if (node.next.point != edge.p):
            # 下一个在边的上面还是下面?
            if (Orient2d(  edge.q, *node.next.point,  edge.p) == Orientation.CCW):
                #下边
                if (Orient2d(  node.point,  node.next.point,  node.next.next.point) == Orientation.CCW):
                    # 下一个是凹的
                    self.FillRightConcaveEdgeEvent(tcx, edge, node);
                else:
                    pass
                # 下一个是凸

    def FillRightConvexEdgeEvent(self,tcx,edge,node):
        # 下一个是凹的还是凸的?
        if (Orient2d(*node.next.point, * node.next.next.point, * node.next.next.next.point) == Orientation.CCW):
            # 凹面
            self.FillRightConcaveEdgeEvent(tcx, edge, * node.next);
        else:
            # 凸面体
            # 下一个在边的上面还是下面?
            if (Orient2d( * edge.q, * node.next.next.point, * edge.p) == Orientation.CCW):
                # 下边
                self.FillRightConvexEdgeEvent(tcx, edge, * node.next);
            else:
                pass
            # 上面
            
    def FillLeftAboveEdgeEvent(self,tcx,edge,node):
        while (node.prev.point.x > edge.p.x):
            # 检查下一个节点是否在边缘以下
            if (Orient2d( edge.q, node.prev.point, edge.p) == Orientation.CW):
                self.FillLeftBelowEdgeEvent(tcx, edge, node)
            else:
                node = node.prev;
    def FillLeftBelowEdgeEvent(self,tcx,edge,node):
        if (node.point.X > edge.p.X):
            if (Orient2d(  node.point,  node.prev.point,  node.prev.prev.point) == Orientation.CW):
                # 凹面
                self.FillLeftConcaveEdgeEvent(tcx, edge, node);
            else:
                # 凸面体
                self.FillLeftConvexEdgeEvent(tcx, edge, node);
                # 重试这个
                self.FillLeftBelowEdgeEvent(tcx, edge, node);
    def FillLeftConcaveEdgeEvent(self,tcx,edge,node):
        self.Fill(tcx, *node.prev);
        if (node.prev.point != edge.p):
            # 下一个在边的上面还是下面?
            if (Orient2d( * edge.q, * node.prev.point, * edge.p) == Orientation.CW):
                # 下边
                if (Orient2d( * node.point, * node.prev.point, * node.prev.prev.point) == Orientation.CW):
                    # 下一个是凹的
                    self.FillLeftConcaveEdgeEvent(tcx, edge, node);
                else:
                    pass
                    # Next is convex
    
    def FillLeftConvexEdgeEvent(self,tcx,edge,node):
        # 下一个是凹的还是凸的?
        if (Orient2d(*node.prev.point, node.prev.prev.point, node.prev.prev.prev.point) == Orientation.CW) :
            # 凹面
            self.FillLeftConcaveEdgeEvent(tcx, edge, node.prev);
        else :
            # 凸面体
            # 下一个在边的上面还是下面?
            if (Orient2d( edge.q, node.prev.prev.point, edge.p) == Orientation.CW) :
            # 下边
                self.FillLeftConvexEdgeEvent(tcx, edge, node.prev);
            else :
                pass
            # Above
    def FlipEdgeEvent(self,tcx,ep,eq,t,p):
        ot = t.NeighborAcross(p);
        op = ot.OppositePoint(t, p);

        if (ot == None) :
            # 如果我们想对fillEdgeEvent进行积分就在这里
            # 以目前的实现方式，我们永远不可能达到这个目标
            raise RuntimeError("[BUG:FIXME] FLIP failed due to missing triangle")#翻转失败，因为缺少三角形

        if (self.InScanArea(p, t.PointCCW(p),  t.PointCW(p), op)) :
            # 让我们旋转共享边一个顶点CW
            self.RotateTrianglePair( * t, p, ot, op);
            tcx.MapTriangleToNodes( * t);
            tcx.MapTriangleToNodes(ot);

            if (p == eq and op == ep) :
                if (eq == tcx.edge_event.constrained_edge.q and ep == tcx.edge_event.constrained_edge.p) :
                    t.MarkConstrainedEdge(  ep,  eq);
                    ot.MarkConstrainedEdge(  ep,  eq);
                    self.Legalize(tcx,  t);
                    self.Legalize(tcx, ot);
                else :
                     pass
                # TODO: 我认为其中一个三角形应该在这里合法化?

            else :
                o = Orient2d(eq, op, ep);
                t = self.NextFlipTriangle(tcx, int(o), t, ot, p, op);
                self.FlipEdgeEvent(tcx, ep, eq, t, p);

        else :
            newP = self.NextFlipPoint(ep, eq, ot, op);
            self.FlipScanEdgeEvent(tcx, ep, eq, * t, ot, newP);
            self.EdgeEvent(tcx, ep, eq, t, p);
            
    def NextFlipTriangle(self,tcx,o,t,ot,p,op):
        if (o == Orientation.CCW):
            # 翻转后ot不相交
            edge_index = ot.EdgeIndex( p, op);
            ot.delaunay_edge[edge_index] = True;
            self.Legalize(tcx, ot);
            ot.ClearDelunayEdges();
            return t;

        # t不是翻转后的过边
        edge_index = t.EdgeIndex(  p,  op);

        t.delaunay_edge[edge_index] = True;
        self.Legalize(tcx, t);
        t.ClearDelunayEdges();
        return ot;
    
    def NextFlipPoint(self,ep,eq,ot,op):

        o2d = Orient2d(eq, op, ep);
        if (o2d == Orientation.CW) :
            # Right
            return ot.PointCCW(op)
        elif (o2d == Orientation.CCW) :
            # Left
            return ot.PointCW(op);

        else :
            raise RuntimeError("[Unsupported] Opposing point on constrained edge")
    
    def FlipScanEdgeEvent(self,tcx,ep,eq,flip_triangle,t,p):
        ot = t.NeighborAcross(p);
        op = ot.OppositePoint(t, p);

        if (t.NeighborAcross(p) == None) :
            # 如果我们想对fillEdgeEvent进行积分就在这里
            # 以目前的实现方式，我们永远不可能达到这个目标
            raise RuntimeError("[BUG:FIXME] FLIP failed due to missing triangle")
        if (InScanArea(eq, *flip_triangle.PointCCW(eq), flip_triangle.PointCW(eq), op)) :
            # 翻转与新的边缘op->eq
            self.FlipEdgeEvent(tcx, eq, op, ot, op);
            # TODO: 实际上，我只是想通过在上面的flipScanEdgeEvent之前获得下一个ot和op来改进它，
            # 并在这里继续flipScanEdgeEvent
            # 在这里设置新的ot和op，并循环回inScanArea测试也需要先设置一个新的flip_triangle
            # 乍一看这有点复杂，所以还得等。
        else :
            newP = self.NextFlipPoint(ep, eq, ot, op);
            self.FlipScanEdgeEvent(tcx, ep, eq, flip_triangle, ot, newP);
            
    def FinalizationPolygon(self,tcx):
        # 从内部三角形开始
        t = tcx.front().head().next.triangle;
        p = tcx.front().head().next.point;
        while (not t.GetConstrainedEdgeCW( p)):
            t = t.NeighborCCW( * p);
        # 收集受边约束的内部三角形
        tcx.MeshClean(*t);












            
        
    

    





            
        


        


