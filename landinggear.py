#========= LANDING GEAR DEFINITION =============================================
# This is a dependency of appendix.py.


from __future__ import division
import math, rhinoscriptsyntax as rs

# Generating a simple toroidal tyre
def AddTorusXY(CentreP, MajorRadius, MinorRadius):
    CpX = CentreP[0]
    CpY = CentreP[1]
    CpZ = CentreP[2]
    
    C1 = [CpX+MajorRadius-MinorRadius, CpY, CpZ]
    C2 = [CpX+MajorRadius+MinorRadius, CpY, CpZ]
    C3 = [CpX+MajorRadius            , CpY, CpZ+MinorRadius]
    
    C = rs.AddCircle3Pt(C1, C2, C3)
    
    RevAx = rs.AddLine(CentreP, [CpX, CpY, CpZ + 1])
    
    Torus = rs.AddRevSrf(C, RevAx)
    
    rs.DeleteObjects([C, RevAx])
    
    return Torus


def LandingGear(WheelCentre, WheelRadius, StrutLength):

    TyreDepth = 0.5*WheelRadius

    Tyre = AddTorusXY(WheelCentre, WheelRadius-0.5*TyreDepth, TyreDepth*0.5) 

    DiskThickness = 0.05*WheelRadius
    DiskBaseCentre = [WheelCentre[0], WheelCentre[1],\
    WheelCentre[2]+DiskThickness*0.5]
    DiskTopCentre =  [WheelCentre[0], WheelCentre[1],\
    WheelCentre[2]-DiskThickness*0.5]

    AxleTopCentre = [WheelCentre[0], WheelCentre[1],\
    WheelCentre[2]+DiskThickness*0.5+TyreDepth]

    Disk = rs.AddCylinder(DiskBaseCentre, DiskTopCentre,\
    WheelRadius-0.5*TyreDepth, True)

    Axle = rs.AddCylinder(DiskTopCentre, AxleTopCentre, WheelRadius*0.1, True)

    StrutEndTopCentre = [WheelCentre[0], WheelCentre[1],\
    WheelCentre[2]+DiskThickness*0.5+TyreDepth+DiskThickness]

    StrutWidth = WheelRadius*0.5
    StrutThickness = DiskThickness

    StrutEnd = rs.AddCylinder(AxleTopCentre, StrutEndTopCentre, StrutWidth,True)

    vB = []

    list.append(vB, [ AxleTopCentre[0]-StrutWidth,\
    AxleTopCentre[1], AxleTopCentre[2]])
    list.append(vB, [ AxleTopCentre[0]+StrutWidth,\
    AxleTopCentre[1], AxleTopCentre[2]])
    list.append(vB, [ AxleTopCentre[0]+StrutWidth,\
    AxleTopCentre[1]+StrutLength, AxleTopCentre[2]])
    list.append(vB, [ AxleTopCentre[0]-StrutWidth,\
    AxleTopCentre[1]+StrutLength, AxleTopCentre[2]])

    list.append(vB, [ AxleTopCentre[0]-StrutWidth,\
    AxleTopCentre[1], AxleTopCentre[2]+StrutThickness])
    list.append(vB, [ AxleTopCentre[0]+StrutWidth,\
    AxleTopCentre[1], AxleTopCentre[2]+StrutThickness])
    list.append(vB, [ AxleTopCentre[0]+StrutWidth,\
    AxleTopCentre[1]+StrutLength, AxleTopCentre[2]+StrutThickness])
    list.append(vB, [ AxleTopCentre[0]-StrutWidth,\
    AxleTopCentre[1]+StrutLength, AxleTopCentre[2]+StrutThickness])

    Strut = rs.AddBox(vB)

    LG = rs.BooleanUnion([Strut, StrutEnd, Axle, Disk, Tyre])
    
    RVec = rs.VectorCreate(WheelCentre, [WheelCentre[0]+1, WheelCentre[1],\
    WheelCentre[2]])
    LG = rs.RotateObject(LG, WheelCentre, -90, axis = RVec)

    return LG

#========= END OF LANDING GEAR