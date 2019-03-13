# AUXILIARY TOOLS FOR UAV GEOMETRY GENERATION ==================================
# This is a dependency of appendix.py.

from __future__ import division
import math, rhinoscriptsyntax as rs

def domain_gen(surface_id):
    if surface_id:
        domainU = rs.SurfaceDomain( surface_id, 0)
        print domainU
        u0 = domainU[0]
        u1 = domainU[1]

        domainV = rs.SurfaceDomain( surface_id, 1)
        print domainV
        v0 = domainV[0]
        v1 = domainV[1]
        return u0, u1, v0, v1
    else:
        print ("domain generation error")

def ExtractTrailingEdge(surface_id):
    u0, u1, v0, v1 = domain_gen(surface_id)
    TE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u0,v0],[u1,v0]])
    return TE_Curve

def BooleanSplitY(yposn,cuttersize,solid):
    # Take the incoming solid and split it into two new solids using a Yplane
    # cutting surface at yposn. Deletes the original solid.
    pt1 = rs.AddPoint((-cuttersize,yposn,cuttersize))
    pt2 = rs.AddPoint((-cuttersize,yposn,-cuttersize))
    pt3 = rs.AddPoint((-cuttersize,yposn+cuttersize,-cuttersize))
    pt4 = rs.AddPoint((-cuttersize,yposn+cuttersize,cuttersize))
    pt5 = rs.AddPoint((cuttersize,yposn,cuttersize))
    pt6 = rs.AddPoint((cuttersize,yposn,-cuttersize))
    pt7 = rs.AddPoint((cuttersize,yposn+cuttersize,-cuttersize))
    pt8 = rs.AddPoint((cuttersize,yposn+cuttersize,cuttersize))
    pt9 = rs.AddPoint((-cuttersize,yposn-cuttersize,cuttersize))
    pt10 = rs.AddPoint((-cuttersize,yposn-cuttersize,-cuttersize))
    pt11 = rs.AddPoint((cuttersize,yposn-cuttersize,cuttersize))
    pt12 = rs.AddPoint((cuttersize,yposn-cuttersize,-cuttersize))
    Cutter1 = rs.AddBox([pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8])
    #Cutter2 = rs.AddBox([pt1, pt2, pt9, pt10, pt5, pt6, pt11, pt12])
    
    Cutter2 = rs.AddBox([pt1, pt2, pt10, pt9, pt5, pt6, pt12, pt11])
    
    solid1 = rs.CopyObject(solid)
    solid2 = rs.CopyObject(solid)
    solid1 = rs.BooleanDifference(solid1,Cutter1)
    solid2 = rs.BooleanDifference(solid2,Cutter2)
    
    rs.DeleteObjects([pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8, pt9, pt10,\
    pt11, pt12, solid])
    
    return solid1, solid2

def BooleanSplitX(xposn,cuttersize,solid):
    # take the incoming solid and split it into two new solids using a Xplane
    # cutting surface at xposn
    # deletes the original solid
    pt1 = rs.AddPoint((xposn,-cuttersize,-cuttersize))
    pt2 = rs.AddPoint((xposn+cuttersize,-cuttersize,-cuttersize))
    pt3 = rs.AddPoint((xposn+cuttersize,-cuttersize,cuttersize))
    pt4 = rs.AddPoint((xposn,-cuttersize,cuttersize))
    pt5 = rs.AddPoint((xposn,cuttersize,-cuttersize))
    pt6 = rs.AddPoint((xposn+cuttersize,cuttersize,-cuttersize))
    pt7 = rs.AddPoint((xposn+cuttersize,cuttersize,cuttersize))
    pt8 = rs.AddPoint((xposn,cuttersize,cuttersize))
    pt9 = rs.AddPoint((xposn-cuttersize,-cuttersize,-cuttersize))
    pt10 = rs.AddPoint((xposn-cuttersize,-cuttersize,cuttersize))
    pt11 = rs.AddPoint((xposn-cuttersize,cuttersize,-cuttersize))
    pt12 = rs.AddPoint((xposn-cuttersize,cuttersize,cuttersize))
    Cutter1 = rs.AddBox([pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8])
    Cutter2 = rs.AddBox([pt1, pt9, pt10, pt4, pt5, pt11, pt12, pt8])
    
    solid1 = rs.CopyObject(solid)
    solid2 = rs.CopyObject(solid)
    solid1 = rs.BooleanDifference(solid1,Cutter1)
    solid2 = rs.BooleanDifference(solid2,Cutter2)
    
    rs.DeleteObjects([pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8, pt9, pt10,\
    pt11, pt12, solid])
    
    return solid1, solid2
    
def BooleanSplitZ(zposn,cuttersize,solid):
    # take the incoming solid and split it into two new solids using a Zplane
    # cutting surface at zposn
    # deletes the original solid
    pt1 = rs.AddPoint((-cuttersize,-cuttersize,zposn))
    pt2 = rs.AddPoint((-cuttersize,-cuttersize,zposn+cuttersize))
    pt3 = rs.AddPoint((-cuttersize,cuttersize,zposn+cuttersize))
    pt4 = rs.AddPoint((-cuttersize,cuttersize,zposn))
    pt5 = rs.AddPoint((cuttersize,-cuttersize,zposn))
    pt6 = rs.AddPoint((cuttersize,-cuttersize,zposn+cuttersize))
    pt7 = rs.AddPoint((cuttersize,cuttersize,zposn+cuttersize))
    pt8 = rs.AddPoint((cuttersize,cuttersize,zposn))
    pt9 = rs.AddPoint((-cuttersize,-cuttersize,zposn-cuttersize))
    pt10 = rs.AddPoint((-cuttersize,cuttersize,zposn-cuttersize))
    pt11 = rs.AddPoint((cuttersize,-cuttersize,zposn-cuttersize))
    pt12 = rs.AddPoint((cuttersize,cuttersize,zposn-cuttersize))
    Cutter1 = rs.AddBox([pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8])
    Cutter2 = rs.AddBox([pt1, pt9, pt10, pt4, pt5, pt11, pt12, pt8])
    
    solid1 = rs.CopyObject(solid)
    solid2 = rs.CopyObject(solid)
    solid1 = rs.BooleanDifference(solid1,Cutter1)
    solid2 = rs.BooleanDifference(solid2,Cutter2)
    
    rs.DeleteObjects([pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8, pt9, pt10,\
    pt11, pt12, solid])
    
    return solid1, solid2

def CutCylinder(x1,y1,z1,x2,y2,z2,diameter,Solid):
    # take the incoming solid and cut a cylinder in it between the two points
    # of given diameter
    # preserves the original solid
    pt1 = rs.AddPoint((x1,y1,z1))
    pt2 = rs.AddPoint((x2,y2,z2))
    Cutter = rs.AddCylinder(pt1, pt2, diameter/2)
    rs.DeleteObjects([pt1, pt2])
    Solid = rs.BooleanDifference(Solid,Cutter)
    return Solid

def CreateSpar(x1,y1,z1,x2,y2,z2,Odiameter, Idiameter):
    # take the incoming solid and cut a cylinder in it between the two points
    # of given diameter
    # preserves the original solid
    pt1 = rs.AddPoint((x1,y1,z1))
    pt2 = rs.AddPoint((x2,y2,z2))
    Cutter = rs.AddCylinder(pt1, pt2, Idiameter/2)
    Solid = rs.AddCylinder(pt1, pt2, Odiameter/2)
    rs.DeleteObjects([pt1, pt2])
    Solid = rs.BooleanDifference(Solid,Cutter)
    return Solid

#========= end of auxiliary tools ==============================================