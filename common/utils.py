import math
from enum import Enum
PI_3div4 = 3 * math.pi /4
EPSILON = 1e-12

#enum Orientation { CW, CCW, COLLINEAR };
class Orientation(Enum):
    CW = 0
    CCW = 1
    COLLINEAR = 2


"""
/**
 * Forumla to calculate signed area<br>
 * Positive if CCW<br>
 * Negative if CW<br>
 * 0 if collinear<br>
 * <pre>
 * A[P1,P2,P3]  =  (x1*y2 - y1*x2) + (x2*y3 - y2*x3) + (x3*y1 - y3*x1)
 *              =  (x1-x3)*(y2-y3) - (y1-y3)*(x2-x3)
 * </pre>
 */
"""
def Orient2d(pa, pb, pc):
      detleft = (pa.X - pc.X) * (pb.Y - pc.Y);
      detright = (pa.Y - pc.Y) * (pb.X - pc.X);
      val = detleft - detright;
      if (val > -EPSILON and val < EPSILON):
        return Orientation.COLLINEAR;
      elif (val > 0):
          return Orientation.CCW;
      return Orientation.CW;

def InScanArea(pa, pb, pc, pd):

      pdx = pd.X
      pdy = pd.Y
      adx = pa.X - pdx;
      ady = pa.Y - pdy;
      bdx = pb.X - pdx;
      bdy = pb.Y - pdy;

      adxbdy = adx * bdy;
      bdxady = bdx * ady;
      oabd = adxbdy - bdxady;

      if (oabd <= EPSILON):
         return False;
      cdx = pc.X - pdx;
      cdy = pc.Y - pdy;

      cdxady = cdx * ady;
      adxcdy = adx * cdy;
      ocad = cdxady - adxcdy;

      if (ocad <= EPSILON):
        return False;
      return True;
