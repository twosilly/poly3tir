from Shapes3 import Point3
from utils import Orient2d,Orientation,InScanArea,EPSILON,PI_3div4
#import Shapes3 as Point3
print("ss:",Point3(1,2,3))
p = Point3(3,4,1)
p2 = Point3(3,4,1)
print("==:",p != p2)
print("CW:",Orientation.CW)
print("CCW:",Orientation.CCW)
print("COLLINEAR:",Orientation.COLLINEAR)