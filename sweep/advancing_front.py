
import sys
sys.path.append(r"../")
from common import  *
#进阶节点
class Node(object):
    def __init__(self,p):
        self.point = p
        self.triangle = Triangle3(self.point,self.point,self.point)
        self.next = Node
        self.prev = Node
        self.value = p.X
    #@classmethod
    def Node2(self,p,t):
        self.point = p
        self.triangle = t
        self.next = None
        self.prev = None
        self.value = p.X
        return self

#进阶算法
class AdvancingFront(object):
    def __init__(self,head,tail):
        self.head = head
        self.tail = tail
        self.search_node_ = head
    def __del__(self):
        pass

    def head(self):
        return self.head
    def set_head(self,node):
        self.head = node
    def tail(self):
        return self.tail
    def set_tail(self,node):
        self.tail = node
    def search(self):
        return self.search_node_
    def set_search(self,node):
        self.search_node_ = node
    #沿前进方向定位插入点
    def LocateNode(self,x):
        node = self.search_node_
        if (x < node.value):
            node = node.prev
            while ((node) != None):
                if (x >= node.value):
                    self.search_node_ = node;
                    return node;
                node = node.prev
        else:
            node = node.next
            while ((node) != None):
                if (x < node.value):
                    self.search_node_ = node.prev;
                    return node.prev;
                node = node.next

        return None;
    def FindSearchNode(self,x):
        #TODO: implement BST index 实现BST指数
        return self.search_node_
    #定位点
    def LocatePoint(self,point):
        px = point.X
        node = self.FindSearchNode(px)
        nx = node.point.X
        if px == nx:
            if point != node.point:
                #我们可能有两个节点在短时间内具有相同的x值
                if point == node.prev.point:
                    node = node.prev
                elif point == node.next.point:
                    node = node.next
                else:
                    raise RuntimeError("LocatePoint Error")
        elif px < nx:
            node  = node.prev
            while (node != None):
                if point == node.point:
                    break
                node = node.prev
        else:
            node = node.next
            while ((node) != None):
                if (point == node.point):
                    break;
        if node:
            self.search_node_ = node
        return node
