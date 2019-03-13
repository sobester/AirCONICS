# Example script for building a twin boom UAV geometry.
# ==============================================================================
# AirCONICS
# Aircraft CONfiguration through Integrated Cross-disciplinary Scripting 
# version 0.1.1
# Andras Sobester, 2014.
# Bug reports to a.sobester@soton.ac.uk or @ASobester please.
# ==============================================================================

from __future__ import division
import math, rhinoscriptsyntax as rs
import primitives 
import airconics_setup
import liftingsurface
#import liftingsurface_old as liftingsurface
import AirCONICStools as act
import HighLiftDevices as HLD

from harpoon_cfg import harpoonData
from harpoon_cfg import harpoonRefZone
from harpoon_cfg import harpoonClose
from XFLR5XML import XFLR5Data
from XFLR5XML import XFLR5MainWing
from XFLR5XML import XFLR5Elevator
from XFLR5XML import XFLR5Fin
from XFLR5XML import XFLR5Close
from fluent_jou import fluentData
from fluent_jou import fluentBCs
from fluent_jou import fluentModel
from fluent_jou import fluentMethod
from fluent_jou import fluentInitc
from fluent_jou import fluentCalc
from fluent_jou import fluentAdapt
from fluent_jou import fluentRunOn
from fluent_jou import fluentReport
from datfilegen import curve2dat

global BoomInner, BoomOuter
global SettingAngle, TaperRatio
global fo
rs.UnitAbsoluteTolerance( 0.0001 )
rs.UnitAngleTolerance(0.1)
rs.UnitRelativeTolerance(0.1)

#===============================================================================
# EXAMPLE - Decode1 pusher UAV
#===============================================================================

# Name	=	Value	#	Long Name / Definition
Awing	=	965830.1	#	total wing area
AR	=	9.0	#	aspect  ratio (span^2 / area)
Span	=	2948.3	#	total wing span (rect wing)
Chord	=	327.6	#	aerodynamic mean chord
Dprop	=	435.0	#	propellor diameter
Span_tail	=	797.4	#	tailplane span
Chord_tail	=	199.3	#	tailplane mean chord
Atail	=	158945.7	#	tailplane area
ARtail	=	4.0	#	tailplane aspect ratio (span^2 / area)
Height_fin	=	293.0	#	fin height (or semi-span) for two fins
Chord_fin	=	195.3	#	fin mean chord
Depth_Fuse	=	200.0	#	fuselage depth
Width_Fuse	=	150.0	#	fuselage width
Len_Nose	=	150.0	#	nose length (forward of front bulkhead)
Dia_Wheels	=	100.0	#	diam of main undercariage wheels
Len_Engine	=	125.0	#	length of engine
x_fnt_bkhd	=	640.3	#	long position of front bulkhead
x_tail_spar	=	-995.3	#	long position of tailplane spar
x_rear_bkhd	=	-200.0	#	long position of rear bulkhead
x_mid_bkhd	=	220.2	#	long position of middle bulkhead
z_fuse_base	=	-110.0	#	vert position of base of fuselage
z_tail_boom	=	0.0	#	vert position of tailboom
z_engine	=	60.0	#	vert position of engine
z_uncarriage	=	-300.0	#	vert position of centre of main undercarriage wheels
y_tail_boom	=	239.2	#	horizontal position of tail booms
x_main_spar	=	0.0	#	long position of main spar

Vmax_C	=	30.00	#	maximum cruise speed
V_L	=	15.00	#	landing speed
V_T	=	16.00	#	take-off speed
Pr_C	=	998.68	#	Pressure at cruise height
rho_C	=	1.2107	#	Density at cruise height
Vsound_C	=	339.71	#	Speed of Sound at crusie height
Re_C	=	666527.4	#	Reynolds No at crusie height and speed
Visc_C	=	1.785E-05	#	Visc. at cruise height

FinAngle = 90
TaperRatio = 0.667
VTail = False
SWEEP_QTR_deg = 0

Pod = False
NoseGear = True

Ailerons = False
ASpanStart = 0.6
ASpanEnd = 0.98
AChord = 0.3
ATaper = 1.0
ADeflectionAngle = 0.0
Elevators = False
ESpanStart = 0.01
ESpanEnd = 0.98
EChord = 0.3
ETaper = 1.0
EDeflectionAngle = 0.0
Rudder = False
RSpanStart = 0.0555
RSpanEnd = 0.98
RChord = 0.3
RTaper = 1.0
RDeflectionAngle = 0.0
# turn on the control surfaces
Ailerons = True
Elevators = True
Rudder = True

# cut up the structure if needed
CutStructure = True
RibThick = 0.005
CutterSize = 5
MainSparODia = 0.02
MainSparIDia = 0.016
BoomODia = 0.02
BoomIDia = 0.016
TailSparODia = 0.016
TailSparIDia = 0.013
FinSparODia = 0.01
FinSparIDia = 0.008
HingeSparODia = 0.005
HingeSparIDia = 0.003
SparClearance = 0.0

# meshing details for Harpoon
DesiredyPlus = 75.0 # use 75 for coarse mesh and 1 for fine one
RefLev = 4 # use 4 for coarse mesh and 5 for fine one 
#                           (nb tailplane outer zones are one level coarser)
FirstCellHtMult = 60 # multiply by 60 for coarse mesh and 2500 for fine one

# meshing details for Harpoon (if using symmetry edit farfield ymin to be -0.5
# in body units) note that Fluent scripts should include y+ adaptation as well
# also turn of BL mesh for coarse model as Fluent will adapt back what is reqd
DesiredyPlus = 0.8 # use 30 for coarse mesh and 1.7 for fine one
RefLev = 3 # use 2 for coarse mesh and 3 for fine one
#                           (nb tailplane outer zones are one level coarser)
FirstCellHtMult = 2500 # multiply by 250 for coarse mesh and 5000 for fine one

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

def myDihedralFunctionTractor(Epsilon):
    # User-defined function describing the variation of dihedral as a function
    # of the leading edge coordinate
    return 0

def myTwistFunctionTractor(Epsilon):
    # User-defined function describing the variation of twist as a function
    # of the leading edge coordinate
    RootTwist = 0
    TipTwist = 2 # typical reduction in twist to give washout
    TipTwist = 0
    return RootTwist + Epsilon*(TipTwist-RootTwist)

def myChordFunctionTractor(Epsilon):
    # User-defined function describing the variation of chord as a function of 
    # the leading edge coordinate
    
    if Epsilon < 0.2:
        return 1
    else:
        ChordLengths = [1, TaperRatio]
        EpsArray = [0.2, 1]
        f = act.linear_interpolation(EpsArray, ChordLengths)
        return f(Epsilon)


def myAirfoilFunctionTractor(Epsilon, LEPoint, ChordFunct, ChordFactor, \
    DihedralFunct, TwistFunct):
    # Defines the variation of cross section as a function of Epsilon
    
    AirfoilChordLength = (ChordFactor*ChordFunct(Epsilon))/math.cos \
    (math.radians(TwistFunct(Epsilon)))

    # Instantiate class to set up a generic airfoil with these basic parameters
    Af = primitives.Airfoil(LEPoint, AirfoilChordLength, DihedralFunct(Epsilon)\
    , TwistFunct(Epsilon),
    EnforceSharpTE = True)

    SmoothingPasses = 1

    # Add NACA2212 airfoil curve to document and retrieve handles to it and
    # its chord
    # MaxCamberPercChord = 2, MaxCamberLocTenthChord = 2,
    # MaxThicknessPercChord = 12
    # Airf,Chrd = primitives.Airfoil.AddNACA4(Af, 0, 0, 12, SmoothingPasses)
    
    # Add airfoil curve to document and retrieve handles to it and its chord
    # - in this case NACA23012, with DesignLiftCoefficient = 0.3,
    # MaxCamberLocFracChord = 0.15 and MaxThicknessPercChord = 15
    Airf,Chrd = primitives.Airfoil.AddNACA5(Af, 0.3, 0.15, 15, SmoothingPasses)
    
    return Airf, Chrd
    
    
def mySweepAngleFunctionTractor(Epsilon):
    # User-defined function describing the variation of sweep angle as a
    # function of the leading edge coordinate
    return 0


#======== TAILPLANE ============================================================

def myTwistFunctionTailplane(Epsilon):
    # User-defined function describing the variation of twist as a function
    # of the leading edge coordinate
    # 2.85 gives the correct trim at 30 m/s with zero main wing setting angle
    # according to XFLR5
    #RootTwist = 2.85    
    #TipTwist  = 2.85
    # 0.34 gives the correct trim at 30 m/s with 2.53 main wing setting angle
    # according to XFLR5
    RootTwist = 0.34    
    TipTwist  = 0.34
#    RootTwist = 8.0  
#    TipTwist  = 8.0
    #RootTwist = 0.0    
    #TipTwist  = 0.0
    return RootTwist + Epsilon*(TipTwist-RootTwist)

def myChordFunctionTailplane(Epsilon):
    # User-defined function describing the variation of chord as a function of 
    # the leading edge coordinate
    
    return 1

def myAirfoilFunctionTailplane(Epsilon, LEPoint, ChordFunct, ChordFactor,\
    DihedralFunct, TwistFunct):
    # Defines the variation of cross section as a function of Epsilon
    
    AirfoilChordLength = (ChordFactor*ChordFunct(Epsilon))/math.cos\
    (math.radians(TwistFunct(Epsilon)))

    # Instantiate class to set up a generic airfoil with these basic parameters
    Af = primitives.Airfoil(LEPoint, AirfoilChordLength, DihedralFunct(Epsilon)\
    , TwistFunct(Epsilon),
    EnforceSharpTE = True)

    SmoothingPasses = 1

    # Add NACA2212 airfoil curve to document and retrieve handles to it and
    # its chord
    # MaxCamberPercChord = 2, MaxCamberLocTenthChord = 2,
    # MaxThicknessPercChord = 12
    # Airf,Chrd = primitives.Airfoil.AddNACA4(Af, 2*(1-Epsilon), 2, 10,
    # SmoothingPasses)
    Airf,Chrd = primitives.Airfoil.AddNACA4(Af, 0, 2, 12, SmoothingPasses)
    # Airf,Chrd = primitives.Airfoil.AddNACA4(Af, 0, 0, 12, SmoothingPasses)

    return Airf, Chrd
    
    
def mySweepAngleFunctionTailplane(Epsilon):
    # User-defined function describing the variation of sweep angle as a fn
    # of the leading edge coordinate
    return 0

def myDihedralFunctionTailplane(Epsilon):
    # User-defined function describing the variation of dihedral as a function
    # of the leading edge coordinate
    return 0


#========= END OF TAILPLANE ==================================================


#======== TAILFIN ============================================================

def myTwistFunctionFin(Epsilon):
    # User-defined function describing the variation of twist as a function
    # of the leading edge coordinate
    return 0

def myChordFunctionFin(Epsilon):
    # User-defined function describing the variation of chord as a function of 
    # the leading edge coordinate
    
    return 1


def myAirfoilFunctionFin(Epsilon, LEPoint, ChordFunct, ChordFactor,\
    DihedralFunct, TwistFunct):
    # Defines the variation of cross section as a function of Epsilon
    
    AirfoilChordLength = (ChordFactor*ChordFunct(Epsilon))/math.cos(\
    math.radians(TwistFunct(Epsilon)))

    # Instantiate class to set up a generic airfoil with these basic parameters
    Af = primitives.Airfoil(LEPoint, AirfoilChordLength, DihedralFunct(Epsilon)\
    , TwistFunct(Epsilon),
    EnforceSharpTE = True)

    SmoothingPasses = 1

    # Add NACA2212 airfoil curve to document and retrieve handles to it and
    # its chord
    # MaxCamberPercChord = 2, MaxCamberLocTenthChord = 2,
    # MaxThicknessPercChord = 12
    Airf,Chrd = primitives.Airfoil.AddNACA4(Af, 0, 2, 12, SmoothingPasses)
    # Airf,Chrd = primitives.Airfoil.AddNACA4(Af, 0, 0, 12, SmoothingPasses)

    return Airf, Chrd
    
    
def mySweepAngleFunctionFin(Epsilon):
    # User-defined function describing the variation of sweep angle as a fn
    # of the leading edge coordinate
    return 0

def myDihedralFunctionFin(Epsilon):
    # User-defined function describing the variation of dihedral as a function
    # of the leading edge coordinate
    return 0


#========= END OF TAILFIN ====================================================

#========= START OF POD ======================================================


def pod(Location, Length, Fineness, CylFraction, Axis = None, Twist = 0):

    CylLength = Length*CylFraction    
    AirfoilLength = Length*(1-CylFraction)
    
    Af = primitives.Airfoil(Location, AirfoilLength, 0, Twist,\
    EnforceSharpTE = True)
    SmoothingPasses = 2
    Airf,Chrd = primitives.Airfoil.AddNACA4(Af, 0, 0,\
    Fineness*(1/(1-CylFraction)), SmoothingPasses)

    LEPoint = rs.CurveStartPoint(Chrd)
    LEPointParameter = rs.CurveClosestPoint(Airf, LEPoint)
    [Airf1,Airf2] = rs.SplitCurve(Airf, LEPointParameter)
    rs.DeleteObject(Airf2)
    Airf = Airf1

    if CylFraction>0:
        AP1 = rs.AddPoint([Location[0]+0.3*AirfoilLength, Location[1],\
        Location[2]+AirfoilLength])

        SplitPointParameter = rs.CurveClosestPoint(Airf1, AP1)

        [AirfAFT, AirfFWD] = rs.SplitCurve(Airf1, SplitPointParameter)

        rs.MoveObject(AirfAFT, [CylLength, 0,0])

        FWDPoint = rs.CurveStartPoint(AirfFWD)
        AFTPoint = rs.CurveEndPoint(AirfAFT)

        Connector = rs.AddLine(FWDPoint, AFTPoint)

        Airf = rs.JoinCurves([AirfFWD,Connector,AirfAFT], True)

        rs.DeleteObjects([Connector, AP1])

    if Axis:
        PodSurf = rs.AddRevSrf(Airf, Axis, 0, 360)
    else:
        PodSurf = rs.AddRevSrf(Airf, Chrd, 0, 360)
    
    rs.DeleteObjects([Airf2, Chrd, Airf])
    
    return [PodSurf]

#========= END OF POD ======================================================



#========= LANDING GEAR

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

#========= BOOLEAN SPLIT TOOLS

def BooleanSplitY(yposn,cuttersize,solid):
    # take the incoming solid and split it into two new solids using a Yplane
    # cutting surface at yposn
    # deletes the original solid
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

#========= END OF BOOLEAN SPLIT TOOLS =========================================

#========= DRAW GEOMETRY ======================================================
# first open file for writing harpoon meshing instructions
#
fo = open("C:\\Users\\Andy\\Documents\\airconicsv021\\harpoon_meshing_file.cfg", "w")
harpoonData(fo, "C:\\Users\\Andy\\Documents\\ajk\\UAV design\\Decode1.stl",\
DesiredyPlus, Chord/1000.0, rho_C, V_L, Visc_C, RefLev, FirstCellHtMult)
rs.EnableRedraw(False)

SWEEP_QTR_rad = math.radians(SWEEP_QTR_deg)
SWEEP_LE_rad = math.atan(math.tan(SWEEP_QTR_rad)+(1/AR)*(1-TaperRatio)/(1+TaperRatio))
SWEEP_LE_deg = math.degrees(SWEEP_LE_rad)

# Wing apex location
P = (0,0,0)

LooseSurf = 3
SegmentNo = 100

SettingAngle = 2.53 # as computed with XFLR5 to give Cl of 0.28 at 30 m/s

TipRequired = 1
SectionsRequired = True # when optimizing this currently needs to be False but
#                           when creating XFLR5 files it must be True
#SegmentNo = 10 # reduce segment number when optimizing to speed up process

Wing = liftingsurface.LiftingSurface(P, mySweepAngleFunctionTractor,\
myDihedralFunctionTractor, myTwistFunctionTractor, myChordFunctionTractor,\
myAirfoilFunctionTractor, LooseSurf, SegmentNo, TipRequired, SectionsRequired)

#Wing = liftingsurface.LiftingSurface(P, mySweepAngleFunctionTractor,\
#myDihedralFunctionTractor, myTwistFunctionTractor, myChordFunctionTractor,\
#myAirfoilFunctionTractor, LooseSurf, SegmentNo)

# Specify the desired aspect ratio, span, etc
Wing.TargetAspectRatio = AR
Wing.TargetArea = Awing/1000000.0
Wing.wTargetAspectRatio = 1
Wing.wTargetArea = 1
ChordFactor = 0.26
ScaleFactor = 1.47
OptimizeChordScale=0

#SLS, ActualSemiSpan, LSP_area,  RootChord, AR, SWingTip =\
#Wing.GenerateLiftingSurface(ChordFactor, ScaleFactor, OptimizeChordScale)
SLS, ActualSemiSpan, LSP_area,  RootChord, AR, SWingTip, Sections =\
Wing.GenerateLiftingSurface(ChordFactor, ScaleFactor, OptimizeChordScale)
print "Wing.Scalefactor, ScaleFactor =",Wing.ScaleFactor,ScaleFactor
print "Wing.Chordfactor, ChordFactor =",Wing.ChordFactor,ChordFactor
TipX = math.tan( SWEEP_LE_rad ) * ActualSemiSpan

RotVec = rs.VectorCreate((0,0,0),(0,1,0))
SLS = rs.RotateObject(SLS, (0,0,0), -SettingAngle, axis = RotVec)
SWingTip = rs.RotateObject(SWingTip, (0,0,0), -SettingAngle, axis = RotVec)
SLS_TE = ExtractTrailingEdge(SLS)

# shift wing up to allow for setting angle about LE and not 1/4 chord point where we attach it
SLS = rs.MoveObject(SLS,(0.0, 0.0, 0.25*RootChord*math.sin(math.radians(SettingAngle))))
SWingTip = rs.MoveObject(SWingTip,(0.0, 0.0, 0.25*RootChord*math.sin(math.radians(SettingAngle))))
SLS_TE = rs.MoveObject(SLS_TE,(0.0, 0.0, 0.25*RootChord*math.sin(math.radians(SettingAngle))))

if Ailerons:
    SLScopy=rs.CopyObject(SLS)
    Aflapconfig1 = ['simpleflap', ASpanStart, ASpanEnd, AChord, ATaper, ADeflectionAngle] 
    Devices = [Aflapconfig1]
    SAileron, Area, Cutter, Cutter1, Cutter2, CutBrick = HLD.AddHighLiftDevices(SLS, Devices)
    rs.DeleteObjects([Cutter, Cutter1, Cutter2])
    SLS = rs.BooleanDifference(SLS,CutBrick)
    PLS = act.MirrorObjectXZ(SLS)
    PAileron = act.MirrorObjectXZ(SAileron)
    rs.DeleteObjects([SLS, SAileron])
    Aflapconfig1 = ['simpleflap', ASpanStart, ASpanEnd, AChord, ATaper, -ADeflectionAngle] 
    Devices = [Aflapconfig1]
    SAileron, Area, Cutter, Cutter1, Cutter2, CutBrick = HLD.AddHighLiftDevices(SLScopy, Devices)
    rs.DeleteObjects([Cutter, Cutter1, Cutter2])
    SLS = rs.BooleanDifference(SLScopy,CutBrick)
    rs.DeleteObjects([SLScopy])
else:
    PLS = act.MirrorObjectXZ(SLS)

PWingTip = act.MirrorObjectXZ(SWingTip)

SLS_TE_Wake_Curve = rs.OffsetCurve( SLS_TE, [1.0,0.0,0.0],Chord/6000)
SLS_TE_Surf = rs.AddEdgeSrf([SLS_TE, SLS_TE_Wake_Curve])
PLS_TE_Surf = act.MirrorObjectXZ(SLS_TE_Surf)
TE_Surf= rs.BooleanUnion([SLS_TE_Surf, PLS_TE_Surf])
DivPts = rs.DivideCurve(SLS_TE,SegmentNo)
rs.DeleteObjects([SLS_TE, SLS_TE_Wake_Curve])


#for ii in range(0, SegmentNo):
#    harpoonRefZone(fo, [DivPts[ii].X, DivPts[ii].Y, DivPts[ii].Z], [DivPts[ii+1].X,\
#    DivPts[ii+1].Y, DivPts[ii+1].Z], RefLev, 0.2, 0.0075, 14.0, 4.0)
#    harpoonRefZone(fo, [DivPts[ii].X, -DivPts[ii].Y, DivPts[ii].Z], [DivPts[ii+1].X,\
#    -DivPts[ii+1].Y, DivPts[ii+1].Z], RefLev, 0.2, 0.0075, 14.0, 4.0)

if Ailerons:
    Zset = Chord / 1000.0 * AChord * math.sin(math.radians(SettingAngle))
    for ii in range(int(ASpanStart*SegmentNo)-1, int(ASpanEnd*SegmentNo)+1):
        harpoonRefZone(fo, [DivPts[ii].X-(AChord+0.01)*Chord*\
        myChordFunctionTractor(ii/SegmentNo)/1000.0,\
        DivPts[ii].Y, DivPts[ii].Z+Zset], [DivPts[ii+1].X-(AChord+0.01)*Chord*\
        myChordFunctionTractor(ii/SegmentNo)/1000.0,\
        DivPts[ii+1].Y, DivPts[ii+1].Z+Zset], RefLev+4, 0.04*Chord/1000.0, 0.03, 0.0, 0.0)
        harpoonRefZone(fo, [DivPts[ii].X-(AChord+0.01)*Chord*\
        myChordFunctionTractor(ii/SegmentNo)/1000.0,\
        -DivPts[ii].Y, DivPts[ii].Z+Zset], [DivPts[ii+1].X-(AChord+0.01)*Chord*\
        myChordFunctionTractor(ii/SegmentNo)/1000.0,\
        -DivPts[ii+1].Y, DivPts[ii+1].Z+Zset], RefLev+4, 0.04*Chord/1000.0, 0.03, 0.0, 0.0)

harpoonRefZone(fo, [TipX, DivPts[SegmentNo].Y*0.985, DivPts[SegmentNo].Z],\
[TipX, DivPts[SegmentNo].Y*1.015, DivPts[SegmentNo].Z], RefLev,\
RootChord*TaperRatio*3.0, 0.05, 14.0, 4.0)
harpoonRefZone(fo, [TipX, -DivPts[SegmentNo].Y*0.985, DivPts[SegmentNo].Z],\
[TipX, -DivPts[SegmentNo].Y*1.015, DivPts[SegmentNo].Z], RefLev,\
RootChord*TaperRatio*3.0, 0.05, 14.0, 4.0)

# =====  Booms =============================================================

PodLength = 1.5*Chord/1000.0
BoomLength = -x_tail_spar/1000.0
BoomY = y_tail_boom/1000.0
BoomZ = z_tail_boom/1000.0
[SPodSurf] = pod([0.025*PodLength,BoomY,BoomZ],PodLength,11, 0.1)
if CutStructure:
    SPodSurf = CutCylinder(RootChord/4.0,BoomY,BoomZ, BoomLength,BoomY,BoomZ,BoomODia,SPodSurf)
    SBoom = CreateSpar(RootChord/4.0+MainSparODia/2.0,BoomY,BoomZ,\
    BoomLength-TailSparODia/2.0,BoomY,BoomZ, BoomODia-SparClearance, BoomIDia)
else:
    SBoom = rs.AddCylinder([RootChord/4.0,BoomY,BoomZ], [BoomLength,BoomY,BoomZ], 0.01)
PPodSurf = act.MirrorObjectXZ(SPodSurf)
PBoom = act.MirrorObjectXZ(SBoom)
if CutStructure: # cut booms to enable biased meshing along their lengths away
#                   from where the pod ends
    SBoom, SBoom1 = BooleanSplitX(0.875*PodLength,CutterSize,SBoom)
    PBoom, PBoom1 = BooleanSplitX(0.875*PodLength,CutterSize,PBoom)

# =====  Tailplane =============================================================
TailChord = Chord_tail/1000.0
FinX = -x_tail_spar/1000.0-TailChord/4.0
P_TP = [FinX,0,0]
LooseSurf_TP = 1
SegmentNo_TP = 100

TipRequired_TP = 1
SectionsRequired_TP = True # when optimizing this currently needs to be False
#                           but when creating XFLR5 files it must be True
#SegmentNo = 10 # reduce segment number when optimizing to speed up process

Tailplane = liftingsurface.LiftingSurface(P_TP, mySweepAngleFunctionTailplane,\
myDihedralFunctionTailplane, myTwistFunctionTailplane, myChordFunctionTailplane,\
myAirfoilFunctionTailplane, LooseSurf_TP, SegmentNo_TP, TipRequired_TP, SectionsRequired_TP)
#Tailplane = liftingsurface.LiftingSurface(P_TP, mySweepAngleFunctionTailplane,\
#myDihedralFunctionTailplane, myTwistFunctionTailplane, myChordFunctionTailplane,\
#myAirfoilFunctionTailplane, LooseSurf_TP, SegmentNo_TP)

if VTail: # we will use a pair of tails rotated to make the V tail so
#               allow for this in area and aspect ratio
    CantAngle = math.atan((2*Height_fin)/Span_tail)
    TailSpan = Span_tail/1000.0/math.cos(CantAngle)/2.0
    Tailplane.TargetAspectRatio = ARtail/math.cos(CantAngle)/2.0
    Tailplane.TargetArea = Atail/1000000.0/math.cos(CantAngle)/2.0
else:
    CantAngle=0.0
    TailSpan = Span_tail/1000.0
    Tailplane.TargetAspectRatio = ARtail
    Tailplane.TargetArea = Atail/1000000.0

Tailplane.wTargetAspectRatio = 1
Tailplane.wTargetArea = 1
ChordFactor_TP = 0.5
ScaleFactor_TP = 0.4
OptimizeChordScale_TP=0

#SLS_TP, ActualSemiSpan_TP, LSP_area_TP,  RootChord_TP, AR_TP, SWingTip_TP =\
#Tailplane.GenerateLiftingSurface(ChordFactor_TP, ScaleFactor_TP, OptimizeChordScale_TP)
SLS_TP, ActualSemiSpan_TP, LSP_area_TP,  RootChord_TP, AR_TP, SWingTip_TP, Sections_TP =\
Tailplane.GenerateLiftingSurface(ChordFactor_TP, ScaleFactor_TP, OptimizeChordScale_TP)
print "Tailplane.Scalefactor, ScaleFactor =",Tailplane.ScaleFactor,ScaleFactor_TP
print "Tailplane.Chordfactor, ChordFactor =",Tailplane.ChordFactor,ChordFactor_TP

SLS_TP_TE = ExtractTrailingEdge(SLS_TP)

if Elevators:
    Eflapconfig = ['simpleflap', ESpanStart, ESpanEnd, EChord, ETaper, EDeflectionAngle]
    Devices = [Eflapconfig]
    SElevator, Area, Cutter, Cutter1, Cutter2, CutBrick = HLD.AddHighLiftDevices(SLS_TP, Devices)
    rs.DeleteObjects([Cutter, Cutter1, Cutter2])
    SLS_TP = rs.BooleanDifference(SLS_TP,CutBrick)
    # shift elevator down to allow for setting angle about LE and not 1/4 chord
    # point where we attach it
    SElevator = rs.MoveObject(SElevator,(0.0, 0.0, -0.25*RootChord_TP*math.sin\
    (math.radians(myTwistFunctionTailplane(0.0)))))
    PElevator = act.MirrorObjectXZ(SElevator)
STail = rs.JoinSurfaces([SLS_TP,SWingTip_TP])
rs.DeleteObjects([SLS_TP,SWingTip_TP])
PTail = act.MirrorObjectXZ(STail)
Tail = rs.JoinSurfaces([STail, PTail])
rs.DeleteObjects([STail, PTail])
# shift tail down to allow for setting angle about LE and not 1/4 chord point where we attach it
Tail = rs.MoveObject(Tail,(0.0, 0.0, -0.25*RootChord_TP*math.sin\
(math.radians(myTwistFunctionTailplane(0.0)))))
SLS_TP_TE = rs.MoveObject(SLS_TP_TE,(0.0, 0.0, -0.25*RootChord_TP*math.sin\
(math.radians(myTwistFunctionTailplane(0.0)))))

if VTail:
    RotVec = rs.VectorCreate([FinX,y_tail_boom/1000.0,0],[FinX+1,y_tail_boom/1000.0,0])
    RotCent = [FinX,y_tail_boom/1000.0,0]
    if CutStructure:
        #Tail = rs.BooleanUnion([Tail, SFin, PFin])
        Tail = CutCylinder(BoomLength,-ActualSemiSpan_TP,BoomZ, BoomLength,\
        ActualSemiSpan_TP,BoomZ,TailSparODia,Tail)
        TailSpar = CreateSpar(BoomLength,-ActualSemiSpan_TP,BoomZ, BoomLength,\
        ActualSemiSpan_TP,BoomZ, TailSparODia-SparClearance, TailSparIDia)

        Tail, TailS1 = BooleanSplitY(BoomY-(y_tail_boom/1000.0*(1.0-1.0/math.cos\
        (CantAngle))+ActualSemiSpan_TP)+0.05*Chord/1000.0,CutterSize,Tail)
        Tail, TailS2 = BooleanSplitY(BoomY-(y_tail_boom/1000.0*(1.0-1.0/math.cos\
        (CantAngle))+ActualSemiSpan_TP)-0.05*Chord/1000.0,CutterSize,Tail)
        Tail, TailS3 = BooleanSplitY(ESpanStart*ActualSemiSpan_TP+0.00175,CutterSize,Tail)
        TailS1, TailS4 = BooleanSplitY(ESpanEnd*ActualSemiSpan_TP-0.00075,CutterSize,TailS1)
        Tail, TailS5 = BooleanSplitY(-ESpanStart*ActualSemiSpan_TP-0.00175,CutterSize,Tail)
        Tail, TailS6 = BooleanSplitY(-ESpanEnd*ActualSemiSpan_TP+0.00075,CutterSize,Tail)
    
    SLS_TP_TE_P = act.MirrorObjectXZ(SLS_TP_TE)
    SLS_TP_TE_S = rs.CopyObject(SLS_TP_TE)
    rs.DeleteObjects([SLS_TP_TE])
    SLS_TP_TE = rs.JoinCurves([SLS_TP_TE_S, SLS_TP_TE_P])
    rs.DeleteObjects([SLS_TP_TE_S, SLS_TP_TE_P])
    
    Tail = rs.MoveObject(Tail, [0,y_tail_boom/1000.0*(1.0-1.0/math.cos(CantAngle))+ActualSemiSpan_TP,0])
    SLS_TP_TE = rs.MoveObject(SLS_TP_TE, [0,y_tail_boom/1000.0*\
    (1.0-1.0/math.cos(CantAngle))+ActualSemiSpan_TP,0])
    Tail = rs.RotateObject(Tail,RotCent,math.degrees(CantAngle), axis = RotVec)
    SLS_TP_TE = rs.RotateObject(SLS_TP_TE,RotCent,math.degrees(CantAngle), axis = RotVec)
    TailP = act.MirrorObjectXZ(Tail)
    CentrePod = pod([FinX, 0, y_tail_boom/1000.0*math.tan(CantAngle)],RootChord_TP,12, 0.0)
    
    if CutStructure:
        ypos1=y_tail_boom/1000.0*(1.0-1.0/math.cos(CantAngle))+ActualSemiSpan_TP
        TailS1 = rs.MoveObject(TailS1, [0,ypos1,0])
        TailS2 = rs.MoveObject(TailS2, [0,ypos1,0])
        TailS3 = rs.MoveObject(TailS3, [0,ypos1,0])
        TailS4 = rs.MoveObject(TailS4, [0,ypos1,0])
        TailS5 = rs.MoveObject(TailS5, [0,ypos1,0])
        TailS6 = rs.MoveObject(TailS6, [0,ypos1,0])
        TailSpar = rs.MoveObject(TailSpar, [0,ypos1,0])
        TailS1 = rs.RotateObject(TailS1,RotCent,math.degrees(CantAngle), axis = RotVec)
        TailS2 = rs.RotateObject(TailS2,RotCent,math.degrees(CantAngle), axis = RotVec)
        TailS3 = rs.RotateObject(TailS3,RotCent,math.degrees(CantAngle), axis = RotVec)
        TailS4 = rs.RotateObject(TailS4,RotCent,math.degrees(CantAngle), axis = RotVec)
        TailS5 = rs.RotateObject(TailS5,RotCent,math.degrees(CantAngle), axis = RotVec)
        TailS6 = rs.RotateObject(TailS6,RotCent,math.degrees(CantAngle), axis = RotVec)
        TailSpar = rs.RotateObject(TailSpar,RotCent,math.degrees(CantAngle), axis = RotVec)
        TailS2 = CutCylinder(0.25*Chord/1000.0,BoomY,BoomZ, BoomLength,BoomY,BoomZ,BoomODia,TailS2)
        TailP1 = act.MirrorObjectXZ(TailS1)
        TailP2 = act.MirrorObjectXZ(TailS2)
        TailP3 = act.MirrorObjectXZ(TailS3)
        TailP4 = act.MirrorObjectXZ(TailS4)
        TailP5 = act.MirrorObjectXZ(TailS5)
        TailP6 = act.MirrorObjectXZ(TailS6)
        TailPSpar = act.MirrorObjectXZ(TailSpar)
        Tail = rs.BooleanUnion([Tail, TailP])
    
    if Elevators:
        SElevator = rs.MoveObject(SElevator, [0,y_tail_boom/1000.0*\
        (1.0-1.0/math.cos(CantAngle))+ActualSemiSpan_TP,0])
        PElevator = rs.MoveObject(PElevator, [0,y_tail_boom/1000.0*\
        (1.0-1.0/math.cos(CantAngle))+ActualSemiSpan_TP,0])
        SElevator = rs.RotateObject(SElevator,RotCent,math.degrees(CantAngle), axis = RotVec)
        PElevator = rs.RotateObject(PElevator,RotCent,math.degrees(CantAngle), axis = RotVec)
        SElevatorP = act.MirrorObjectXZ(SElevator)
        PElevatorP = act.MirrorObjectXZ(PElevator)

#SLS_TP_TE_Wake_Curve = rs.OffsetCurve( SLS_TP_TE, [1.0,0.0,0.0],Chord/6000)
SLS_TP_TE_Wake_Curve = rs.CopyObject( SLS_TP_TE, [Chord/6000,0.0,0.0])
SLS_TP_TE_Surf = rs.AddEdgeSrf([SLS_TP_TE, SLS_TP_TE_Wake_Curve])
PLS_TP_TE_Surf = act.MirrorObjectXZ(SLS_TP_TE_Surf)
#TP_TE_Surf= rs.BooleanUnion([SLS_TP_TE_Surf, PLS_TP_TE_Surf])

DivPts_TP = rs.DivideCurve(SLS_TP_TE,SegmentNo_TP)
rs.DeleteObjects([SLS_TP_TE, SLS_TP_TE_Wake_Curve])

#for ii in range(0, SegmentNo_TP):
#    harpoonRefZone(fo, [DivPts_TP[ii].X, DivPts_TP[ii].Y, DivPts_TP[ii].Z],\
#    [DivPts_TP[ii+1].X, DivPts_TP[ii+1].Y, DivPts_TP[ii+1].Z], RefLev, 0.2, 0.015, 14.0, 4.0)
#    harpoonRefZone(fo, [DivPts_TP[ii].X, -DivPts_TP[ii].Y, DivPts_TP[ii].Z],\
#    [DivPts_TP[ii+1].X, -DivPts_TP[ii+1].Y, DivPts_TP[ii+1].Z], RefLev, 0.2, 0.015, 14.0, 4.0)

if Elevators:
    Zset = RootChord_TP * EChord * math.sin(math.radians(myTwistFunctionTailplane(1.0)))
    for ii in range(int(ESpanStart*SegmentNo_TP)-1, int(ESpanEnd*SegmentNo_TP)+1):
        harpoonRefZone(fo, [DivPts_TP[ii].X-(EChord-0.03)*RootChord_TP*\
        myChordFunctionTailplane(ii/SegmentNo_TP), DivPts_TP[ii].Y, DivPts_TP[ii].Z - Zset],\
        [DivPts_TP[ii+1].X-(EChord-0.03)*RootChord_TP*myChordFunctionTailplane(ii/SegmentNo_TP),\
        DivPts_TP[ii+1].Y, DivPts_TP[ii+1].Z - Zset], RefLev+4, 0.04*RootChord_TP, 0.02, 0.0, 0.0)
        harpoonRefZone(fo, [DivPts_TP[ii].X-(EChord-0.03)*RootChord_TP*\
        myChordFunctionTailplane(ii/SegmentNo_TP), -DivPts_TP[ii].Y, DivPts_TP[ii].Z - Zset], \
        [DivPts_TP[ii+1].X-(EChord-0.03)*RootChord_TP*myChordFunctionTailplane(ii/SegmentNo_TP),\
        -DivPts_TP[ii+1].Y, DivPts_TP[ii+1].Z - Zset], RefLev+4, 0.04*RootChord_TP, 0.02, 0.0, 0.0)

harpoonRefZone(fo, [FinX, DivPts_TP[SegmentNo_TP].Y*0.985, DivPts_TP[SegmentNo_TP].Z],\
[FinX, DivPts_TP[SegmentNo_TP].Y*1.015, DivPts_TP[SegmentNo_TP].Z], RefLev-1,\
RootChord_TP*3.0, 0.03, 14.0, 4.0)
harpoonRefZone(fo, [FinX, -DivPts_TP[SegmentNo_TP].Y*0.985, DivPts_TP[SegmentNo_TP].Z],\
[FinX, -DivPts_TP[SegmentNo_TP].Y*1.015, DivPts_TP[SegmentNo_TP].Z], RefLev-1,\
RootChord_TP*3.0, 0.03, 14.0, 4.0)

# =====  Tailfin =============================================================

if VTail == False:
    P_Fin = [FinX+RootChord_TP/30.0,BoomY,0.0]
    FinChord = Chord_fin/1000.0
    FinSpan = Height_fin/1000.0
    FinTC = 12
    LooseSurf_Fin = 1
    SegmentNo_Fin= 100
    
    TipRequired_Fin = 1
    SectionsRequired_Fin = True
    #Fin = liftingsurface.LiftingSurface(P_Fin, mySweepAngleFunctionFin, myDihedralFunctionFin,\
    #myTwistFunctionFin, myChordFunctionFin, myAirfoilFunctionFin, LooseSurf_Fin, SegmentNo_Fin)
    Fin = liftingsurface.LiftingSurface(P_Fin, mySweepAngleFunctionFin, myDihedralFunctionFin,\
    myTwistFunctionFin, myChordFunctionFin, myAirfoilFunctionFin, LooseSurf_Fin, SegmentNo_Fin,\
    TipRequired_Fin, SectionsRequired_Fin)
    
    # Specify the desired aspect ratio and span
    Fin.TargetAspectRatio = Height_fin*2/Chord_fin
    Fin.TargetArea = Chord_fin*Height_fin/1000000.0
    Fin.wTargetAspectRatio = 1
    Fin.wTargetArea = 1
    ChordFactor_Fin = 0.658
    ScaleFactor_Fin = 0.208
    OptimizeChordScale_Fin=0
    
    SLS_Fin, ActualSemiSpan_Fin, LSP_area_Fin,  RootChord_Fin, AR_Fin, SWingTip_Fin, Sections_Fin =\
    Fin.GenerateLiftingSurface(ChordFactor_Fin, ScaleFactor_Fin, OptimizeChordScale_Fin)
    #SLS_Fin, ActualSemiSpan_Fin, LSP_area_Fin,  RootChord_Fin, AR_Fin, SWingTip_Fin =\
    #Fin.GenerateLiftingSurface(ChordFactor_Fin, ScaleFactor_Fin, OptimizeChordScale_Fin)
    print "Fin.Scalefactor, ScaleFactor =",Fin.ScaleFactor,ScaleFactor_Fin
    print "Fin.Chordfactor, ChordFactor =",Fin.ChordFactor,ChordFactor_Fin
    
    SLS_Fin_TE = ExtractTrailingEdge(SLS_Fin)
    
    if Rudder:
        Rflapconfig1 = ['simpleflap', RSpanStart, RSpanEnd, RChord, RTaper, RDeflectionAngle]
        Devices = [Rflapconfig1]
        SRudder, Area, Cutter, Cutter1, Cutter2, CutBrick = HLD.AddHighLiftDevices(SLS_Fin, Devices)
        rs.DeleteObjects([Cutter, Cutter1, Cutter2])
        SLS_Fin = rs.BooleanDifference(SLS_Fin,CutBrick)
    
    SFin = rs.JoinSurfaces([SLS_Fin,SWingTip_Fin])
    P_FinAft = [P_Fin[0]+1, P_Fin[1], P_Fin[2]]
    RotVec = rs.VectorCreate(P_Fin, P_FinAft)
    SFin = rs.RotateObject(SFin, P_Fin, -FinAngle, axis = RotVec)
    
    if Rudder:
        SRudder = rs.RotateObject(SRudder, P_Fin, -FinAngle, axis = RotVec)
        # move fins up to prevent them poking through the bottom of an elevator with setting angle
        SRudder = rs.MoveObject(SRudder, [0.0, 0.0, 0.25*RootChord_TP*math.sin(math.radians\
        (myTwistFunctionTailplane(0.0)))]) 
        PRudder = act.MirrorObjectXZ(SRudder)
    
    # move fins up to prevent them poking through the bottom of an elevator with setting angle
    SFin = rs.MoveObject(SFin, [0.0, 0.0, 0.25*RootChord_TP*math.sin(math.radians\
    (myTwistFunctionTailplane(0.0)))]) 
    rs.CapPlanarHoles(SFin)
    PFin = act.MirrorObjectXZ(SFin)
    
    SLS_Fin_TE_Wake_Curve = rs.CopyObject( SLS_Fin_TE, [Chord/6000,0.0,0.0])
    SLS_Fin_TE_Surf = rs.AddEdgeSrf([SLS_Fin_TE, SLS_Fin_TE_Wake_Curve])
    SLS_Fin_TE_Surf = rs.RotateObject(SLS_Fin_TE_Surf, P_Fin, -FinAngle, axis = RotVec)
    # move fins up to prevent them poking through the bottom of an elevator with setting angle
    SLS_Fin_TE_Surf = rs.MoveObject(SLS_Fin_TE_Surf, [0.0, 0.0, 0.25*RootChord_TP*math.sin\
    (math.radians(myTwistFunctionTailplane(0.0)))]) 
    PLS_Fin_TE_Surf = act.MirrorObjectXZ(SLS_Fin_TE_Surf)
    
    SLS_Fin_TE = rs.RotateObject(SLS_Fin_TE, P_Fin, -FinAngle, axis = RotVec)
    # move fins up to prevent them poking through the bottom of an elevator with setting angle
    SLS_Fin_TE = rs.MoveObject(SLS_Fin_TE, [0.0, 0.0, 0.25*RootChord_TP*math.sin\
    (math.radians(myTwistFunctionTailplane(0.0)))]) 
    DivPts_Fin = rs.DivideCurve(SLS_Fin_TE,SegmentNo_Fin)
    
    rs.DeleteObjects([SLS_Fin,SWingTip_Fin, SLS_Fin_TE, SLS_Fin_TE_Wake_Curve])
    
    if Rudder:
        for ii in range(int(RSpanStart*SegmentNo_Fin)-1, int(RSpanEnd*SegmentNo_Fin)+1):
            harpoonRefZone(fo, [DivPts_Fin[ii].X-(RChord-0.03)*RootChord_Fin*\
            myChordFunctionFin(ii/SegmentNo_Fin), DivPts_Fin[ii].Y-(RootChord_Fin*FinTC/100.0/2.0),\
            DivPts_Fin[ii].Z], [DivPts_Fin[ii+1].X-(RChord-0.03)*RootChord_Fin*\
            myChordFunctionFin(ii/SegmentNo_Fin), DivPts_Fin[ii+1].Y+(RootChord_Fin*FinTC/100.0/2.0),\
            DivPts_Fin[ii+1].Z], RefLev+4, 0.05*RootChord_Fin, 0.02, 0.0, 0.0)
            harpoonRefZone(fo, [DivPts_Fin[ii].X-(RChord-0.03)*RootChord_Fin*\
            myChordFunctionFin(ii/SegmentNo_Fin), -DivPts_Fin[ii].Y-(RootChord_Fin*FinTC/100.0/2.0),\
            DivPts_Fin[ii].Z], [DivPts_Fin[ii+1].X-(RChord-0.03)*RootChord_Fin*\
            myChordFunctionFin(ii/SegmentNo_Fin), -DivPts_Fin[ii+1].Y+(RootChord_Fin*FinTC/100.0/2.0),\
            DivPts_Fin[ii+1].Z], RefLev+4, 0.05*RootChord_Fin, 0.02, 0.0, 0.0)
    
    # harpoonRefZone(fo, [FinX+RootChord_Fin, BoomY+0.0075, ActualSemiSpan_Fin/2],\
    #[FinX+RootChord_Fin, BoomY-0.0075, ActualSemiSpan_Fin/2], RefLev, 0.2,\
    #ActualSemiSpan_Fin, 14.0, 4.0)
    # harpoonRefZone(fo, [FinX+RootChord_Fin, -BoomY-0.0075, ActualSemiSpan_Fin/2],\
    #[FinX+RootChord_Fin, -BoomY+0.0075, ActualSemiSpan_Fin/2], RefLev, 0.2,\
    #ActualSemiSpan_Fin, 14.0, 4.0)
    harpoonRefZone(fo, [FinX, BoomY*0.975, ActualSemiSpan_Fin], [FinX, BoomY*1.025,\
    ActualSemiSpan_Fin], RefLev, RootChord_Fin*3, 0.03, 14.0, 4.0)
    harpoonRefZone(fo, [FinX, -BoomY*0.975, ActualSemiSpan_Fin], [FinX, -BoomY*1.025,\
    ActualSemiSpan_Fin], RefLev, RootChord_Fin*3, 0.03, 14.0, 4.0)

# =====  CentrePod =============================================================

if Pod:
    PayloadPodLength = RootChord*1.1
    CentrePod = pod([-0.05, 0, z_uncarriage/1000.0/2],PayloadPodLength,14, 0.3)
    P_Fin = [0.05,0,0]
    FinChord = Chord/1000.0
    FinSpan = -z_uncarriage/1000.0/4.0
    FinTC = 12
    LooseSurf_Fin = 1
    SegmentNo_Fin= 2
    TipRequired_Fin = 1
    SectionsRequired_Fin = False
    #PodFin = liftingsurface.LiftingSurface(P_Fin, mySweepAngleFunctionTailplane,\
    #myDihedralFunctionTailplane, myTwistFunctionTailplane, myChordFunctionTailplane,\
    #myAirfoilFunctionTailplane, LooseSurf_Fin, SegmentNo_Fin)
    PodFin = liftingsurface.LiftingSurface(P_Fin, mySweepAngleFunctionTailplane,\
    myDihedralFunctionTailplane, myTwistFunctionTailplane, myChordFunctionTailplane,\
    myAirfoilFunctionTailplane, LooseSurf_Fin, SegmentNo_Fin, TipRequired_Fin, SectionsRequired_Fin)
    # Specify the desired aspect ratio and span
    PodFin.TargetAspectRatio = -z_uncarriage/Chord*2.5
    PodFin.TargetArea = (-z_uncarriage/1000.0/2)*(Chord/1000.0)
    PodFin.wTargetAspectRatio = 1
    PodFin.wTargetArea = 1
    ScaleFactor_Fin = 0.175
    ChordFactor_Fin = 1.0231386102
    OptimizeChordScale_Fin=0
    CLS_Pod, ActualSemiSpan_PodFin, LSP_area_PodFin,  RootChord_PodFin, AR_PodFin,\
    WingTip_PodFin, Sections_PodFin = PodFin.GenerateLiftingSurface(ChordFactor_Fin,\
    ScaleFactor_Fin, OptimizeChordScale_Fin)
    #CLS_Pod, ActualSemiSpan_PodFin, LSP_area_PodFin,  RootChord_PodFin, AR_PodFin,\
    #WingTip_PodFin = PodFin.GenerateLiftingSurface(ChordFactor_Fin, ScaleFactor_Fin,\
    #OptimizeChordScale_Fin)
    print "Pod.Scalefactor, ScaleFactor =",PodFin.ScaleFactor,ScaleFactor_Fin
    print "Pod.Chordfactor, ChordFactor =",PodFin.ChordFactor,ChordFactor_Fin
    
    PodFin = rs.JoinSurfaces([CLS_Pod,WingTip_PodFin])
    rs.DeleteObjects([CLS_Pod,WingTip_PodFin])
    P_FinAft = [P_Fin[0]+1, P_Fin[1], P_Fin[2]]
    RotVec = rs.VectorCreate(P_Fin, P_FinAft)
    PodFin = rs.RotateObject(PodFin, P_Fin, 90, axis = RotVec)
    CentrePod = rs.BooleanUnion([CentrePod, PodFin])
    rs.DeleteObjects(PodFin)

# =====  Fuselage =============================================================

FuselageLength = (Len_Nose+2*Len_Engine+x_fnt_bkhd-x_rear_bkhd)/1000.0

FuselageFinennesInPercent = ((Width_Fuse/1000.0)/FuselageLength)*100.0

MainEngineNacelle = pod([-(Len_Nose+x_fnt_bkhd)/1000.0, 0, 0],FuselageLength,\
FuselageFinennesInPercent, 0.6)

PropDia = Dprop/1000.0
PropX = (2*Len_Engine-x_rear_bkhd-Width_Fuse/4.0)/1000.0

MainPropP1 = rs.AddPoint([PropX,0,PropDia/2])
MainPropP2 = rs.AddPoint([PropX,PropDia/2,0])
MainPropP3 = rs.AddPoint([PropX,0,-PropDia/2])
MainProp = rs.AddCircle3Pt(MainPropP1,MainPropP2,MainPropP3)

rs.AddPlanarSrf(MainProp)

rs.DeleteObjects([MainPropP1,MainPropP2,MainPropP3,MainProp])

# =====  Landing gear =============================================================

if NoseGear:
    # Nose gear
    NoseWheelScaleFactor = 1 # with respect to main gear
    NoseWheelX = -x_fnt_bkhd/1000.0
    NoseWheelCentre = [NoseWheelX, 0, z_uncarriage/1000.0]
    NoseWheelRadius = 0.5*NoseWheelScaleFactor*Dia_Wheels/1000.0
    NoseStrutLength = -z_uncarriage/1000
    NoseGear = LandingGear(NoseWheelCentre, NoseWheelRadius, NoseStrutLength)
    harpoonRefZone(fo, [NoseWheelX, 0.05, -(NoseStrutLength+NoseWheelRadius)/2],\
    [NoseWheelX, -0.05, -(NoseStrutLength+NoseWheelRadius)/2], RefLev, RootChord,\
    -(NoseStrutLength+NoseWheelRadius), -4.0, 0.0)
else:
    # Tail gear
    TailWheelScaleFactor = 0.5 # with respect to main gear
    TailWheelX = -x_tail_spar/1000.0
    STailWheelCentre = [TailWheelX, BoomY, z_uncarriage/1000.0/2]
    TailWheelRadius = 0.5*TailWheelScaleFactor*Dia_Wheels/1000.0
    TailStrutLength = -z_uncarriage/1000/2
    TailGearStbd = LandingGear(STailWheelCentre, TailWheelRadius, TailStrutLength)
    TailGearPort = act.MirrorObjectXZ(TailGearStbd)

# MainGear
MainWheelX = -x_rear_bkhd/1000.0
MainWheelY = y_tail_boom/1000.0
MainWheelCentre = [MainWheelX, MainWheelY, z_uncarriage/1000.0]
MainWheelRadius = 0.5*Dia_Wheels/1000.0
MainStrutLength = -z_uncarriage/1000
MainGearPort = LandingGear(MainWheelCentre, MainWheelRadius, MainStrutLength)
MainGearStbd = act.MirrorObjectXZ(MainGearPort)
harpoonRefZone(fo, [MainWheelX, MainWheelY+0.05, -(MainStrutLength+MainWheelRadius)/2],\
[MainWheelX, MainWheelY-0.05, -(MainStrutLength+MainWheelRadius)/2], RefLev, RootChord,\
-(MainStrutLength+MainWheelRadius), -4.0, 0.0)
harpoonRefZone(fo, [MainWheelX, -MainWheelY-0.05, -(MainStrutLength+MainWheelRadius)/2],\
[MainWheelX, -MainWheelY+0.05, -(MainStrutLength+MainWheelRadius)/2], RefLev, RootChord,\
-(MainStrutLength+MainWheelRadius), -4.0, 0.0)

# =====  Assembly =============================================================
# now union all the bits together to create the airframe
#
TWing = rs.JoinSurfaces([SLS,PLS,SWingTip, PWingTip])
rs.DeleteObjects([SLS,PLS,SWingTip, PWingTip])

if CutStructure:
    TWing = CutCylinder(RootChord/4.0,-ActualSemiSpan,0.0,RootChord/4.0,ActualSemiSpan,\
    0.0,MainSparODia,TWing)
    Spar = CreateSpar(RootChord/4.0,-ActualSemiSpan,0.0,RootChord/4.0,ActualSemiSpan,\
    0.0,MainSparODia-SparClearance,MainSparIDia)
    TWing, TWingS1 = BooleanSplitY(BoomY+0.1*Chord/1000.0,CutterSize,TWing)
    TWingP1, TWing = BooleanSplitY(-BoomY-0.1*Chord/1000.0,CutterSize,TWing)
    TWing, TWingS8 = BooleanSplitY(BoomY-0.1*Chord/1000.0,CutterSize,TWing)
    TWingP8, TWing = BooleanSplitY(-BoomY+0.1*Chord/1000.0,CutterSize,TWing)
    TWingS2, TWingS3 = BooleanSplitY(0.9945*ASpanStart*Span/2000.0-RibThick,CutterSize,TWingS1)
    TWingS4, TWingS5 = BooleanSplitY(0.9945*ASpanStart*Span/2000.0,CutterSize,TWingS3)
    TWingS6, TWingS7 = BooleanSplitY(0.9991*ASpanEnd*Span/2000.0,CutterSize,TWingS5)
    TWingP3, TWingP2 = BooleanSplitY(-0.9945*ASpanStart*Span/2000.0+RibThick,CutterSize,TWingP1)
    TWingP5, TWingP4 = BooleanSplitY(-0.9945*ASpanStart*Span/2000.0,CutterSize,TWingP3)
    TWingP7, TWingP6 = BooleanSplitY(-0.9991*ASpanEnd*Span/2000.0,CutterSize,TWingP5)
    
    SPodSurf = CutCylinder(RootChord/4.0,-ActualSemiSpan,0.0,RootChord/4.0,ActualSemiSpan,0.0,\
    MainSparODia,SPodSurf)
    PPodSurf = CutCylinder(RootChord/4.0,-ActualSemiSpan,0.0,RootChord/4.0,ActualSemiSpan,0.0,\
    MainSparODia,PPodSurf)
    TWingS8 = CutCylinder(RootChord/4.0,BoomY,BoomZ, BoomLength,BoomY,BoomZ,BoomODia,TWingS8)
    TWingP8 = CutCylinder(RootChord/4.0,-BoomY,BoomZ, BoomLength,-BoomY,BoomZ,BoomODia,TWingP8)
    SPodSurf = rs.BooleanUnion([SPodSurf,TWingS8])
    PPodSurf = rs.BooleanUnion([PPodSurf,TWingP8])
    
    
    # insert aileron control surface hinge spars
    if Ailerons:
        xpos1=((1.0-4.9*AChord/6.0)*myChordFunctionTractor(ASpanStart))\
        *RootChord*math.cos(math.radians(SettingAngle))
        ypos1a=0.994*ASpanStart*ActualSemiSpan-RibThick
        ypos1b=-0.994*ASpanStart*ActualSemiSpan+RibThick
        zpos1=-((1.0-4.9*AChord/6.0)*myChordFunctionTractor(ASpanStart)-0.25)\
        *RootChord*math.sin(math.radians(SettingAngle))
        xpos2=((1.0-5.0*AChord/6.0)*myChordFunctionTractor(ASpanEnd))\
        *RootChord*math.cos(math.radians(SettingAngle))
        ypos2=1.001*ActualSemiSpan
        zpos2=-((1.0-5.0*AChord/6.0)*myChordFunctionTractor(ASpanEnd)-0.25)\
        *RootChord*math.sin(math.radians(SettingAngle))
        TWingS4 = CutCylinder(xpos1,ypos1a,zpos1,xpos2,ypos2,zpos2,HingeSparODia,TWingS4)
        TWingS7 = CutCylinder(xpos1,ypos1a,zpos1,xpos2,ypos2,zpos2,HingeSparODia,TWingS7)
        SAileron = CutCylinder(xpos1,ypos1a,zpos1,xpos2,ypos2,zpos2,HingeSparODia,SAileron)
        SAileronSpar = CreateSpar(xpos1,ypos1a,zpos1,xpos2,ypos2,zpos2,HingeSparODia\
        -SparClearance,HingeSparIDia)
        TWingP4 = CutCylinder(xpos1,ypos1b,zpos1,xpos2,-ypos2,zpos2,HingeSparODia,TWingP4)
        TWingP7 = CutCylinder(xpos1,ypos1b,zpos1,xpos2,-ypos2,zpos2,HingeSparODia,TWingP7)
        PAileron = CutCylinder(xpos1,ypos1b,zpos1,xpos2,-ypos2,zpos2,HingeSparODia,PAileron)
        PAileronSpar = CreateSpar(xpos1,ypos1b,zpos1,xpos2,-ypos2,zpos2,HingeSparODia\
        -SparClearance,HingeSparIDia)
    
    if VTail == False:
        SFin = CutCylinder(FinX+RootChord_TP/30.0+RootChord_Fin/4.0,BoomY,BoomZ,\
        FinX+RootChord_TP/30.0+RootChord_Fin/4.0,BoomY,BoomZ+CutterSize, FinSparODia,SFin)
        SFinSpar = CreateSpar(FinX+RootChord_TP/30.0+RootChord_Fin/4.0,BoomY,BoomZ+BoomODia/2,\
        FinX+RootChord_TP/30.0+RootChord_Fin/4.0,BoomY,BoomZ+ActualSemiSpan_Fin, FinSparODia\
        -SparClearance, FinSparIDia)
        PFin = CutCylinder(FinX+RootChord_TP/30.0+RootChord_Fin/4.0,-BoomY,BoomZ,\
        FinX+RootChord_TP/30.0+RootChord_Fin/4.0,-BoomY,BoomZ+CutterSize, FinSparODia,PFin)
        PFinSpar = CreateSpar(FinX+RootChord_TP/30.0+RootChord_Fin/4.0,-BoomY,BoomZ+BoomODia/2\
        ,FinX+RootChord_TP/30.0+RootChord_Fin/4.0,-BoomY,BoomZ+ActualSemiSpan_Fin, FinSparODia\
        -SparClearance, FinSparIDia)
        
        SFin, SFin1 = BooleanSplitZ(BoomZ+0.0128,CutterSize,SFin)
        PFin, PFin1 = BooleanSplitZ(BoomZ+0.0128,CutterSize,PFin)
        SFin1, SFin2 = BooleanSplitZ(BoomZ+RSpanEnd*ActualSemiSpan_Fin-0.00075,CutterSize,SFin1)
        PFin1, PFin2 = BooleanSplitZ(BoomZ+RSpanEnd*ActualSemiSpan_Fin-0.00075,CutterSize,PFin1)
        
        # insert rudder control surface hinge spars
        if Rudder:
            xpos1=FinX+RootChord_TP/30.0+RootChord_Fin/4.0+(0.75-4.9*RChord/6.0)\
            *myChordFunctionFin(RSpanStart)*RootChord_Fin*math.cos(math.radians\
            (myTwistFunctionFin(RSpanStart)))
            xpos2=FinX+RootChord_TP/30.0+RootChord_Fin/4.0+(0.75-4.9*RChord/6.0)\
            *myChordFunctionFin(RSpanEnd)*RootChord_Fin*math.cos(math.radians\
            (myTwistFunctionFin(RSpanEnd)))
            SFin = CutCylinder(\
            xpos1\
            ,BoomY,BoomZ,\
            xpos2\
            ,BoomY,BoomZ+ActualSemiSpan_Fin,HingeSparODia,SFin)
            SRudder = CutCylinder(xpos1,BoomY,BoomZ,xpos2,BoomY,BoomZ+ActualSemiSpan_Fin,\
            HingeSparODia,SRudder)
            SFin2 = CutCylinder(xpos1,BoomY,BoomZ,xpos2,BoomY,BoomZ+ActualSemiSpan_Fin+0.001,\
            HingeSparODia,SFin2)
            SRudderSpar = CreateSpar(xpos1,BoomY,BoomZ,xpos2,BoomY,BoomZ+ActualSemiSpan_Fin+0.001,\
            HingeSparODia-SparClearance,HingeSparIDia)
            PFin = CutCylinder(xpos1,-BoomY,BoomZ,xpos2,-BoomY,BoomZ+ActualSemiSpan_Fin,\
            HingeSparODia,PFin)
            PRudder = CutCylinder(xpos1,-BoomY,BoomZ,xpos2,-BoomY,BoomZ+ActualSemiSpan_Fin,\
            HingeSparODia,PRudder)
            PFin2 = CutCylinder(xpos1,-BoomY,BoomZ,xpos2,-BoomY,BoomZ+ActualSemiSpan_Fin+0.001,\
            HingeSparODia,PFin2)
            PRudderSpar = CreateSpar(xpos1,-BoomY,BoomZ,xpos2,-BoomY,BoomZ+ActualSemiSpan_Fin+0.001,\
            HingeSparODia-SparClearance,HingeSparIDia)

        Tail = rs.BooleanUnion([Tail, SFin, PFin])
        Tail = CutCylinder(BoomLength,-ActualSemiSpan_TP,BoomZ, BoomLength,\
        ActualSemiSpan_TP,BoomZ,TailSparODia,Tail)
        Tail = CutCylinder(FinX+RootChord_TP/30.0+RootChord_Fin/4.0,BoomY,BoomZ,\
        FinX+RootChord_TP/30.0+RootChord_Fin/4.0,BoomY,BoomZ+ActualSemiSpan_Fin, FinSparODia,Tail)
        Tail = CutCylinder(FinX+RootChord_TP/30.0+RootChord_Fin/4.0,-BoomY,BoomZ,\
        FinX+RootChord_TP/30.0+RootChord_Fin/4.0,-BoomY,BoomZ+ActualSemiSpan_Fin, FinSparODia,Tail)
        TailSpar = CreateSpar(BoomLength,-ActualSemiSpan_TP,BoomZ, BoomLength,\
        ActualSemiSpan_TP,BoomZ, TailSparODia-SparClearance, TailSparIDia)

        Tail, TailS1 = BooleanSplitY(BoomY+0.05*Chord/1000.0,CutterSize,Tail)
        TailP1, Tail = BooleanSplitY(-BoomY-0.05*Chord/1000.0,CutterSize,Tail)
        Tail, TailS2 = BooleanSplitY(BoomY-0.05*Chord/1000.0,CutterSize,Tail)
        TailP2, Tail = BooleanSplitY(-BoomY+0.05*Chord/1000.0,CutterSize,Tail)
        Tail, TailS3 = BooleanSplitY(ESpanStart*ActualSemiSpan_TP+0.00175,CutterSize,Tail)
        TailP3, Tail = BooleanSplitY(-ESpanStart*ActualSemiSpan_TP-0.00175,CutterSize,Tail)
        TailS1, TailS4 = BooleanSplitY(ESpanEnd*ActualSemiSpan_TP-0.002,CutterSize,TailS1)
        TailP4, TailP1 = BooleanSplitY(-ESpanEnd*ActualSemiSpan_TP+0.002,CutterSize,TailP1)
        
        TailS2 = CutCylinder(0.25*Chord/1000.0,BoomY,BoomZ, BoomLength,BoomY,BoomZ,BoomODia,TailS2)
        TailP2 = CutCylinder(0.25*Chord/1000.0,-BoomY,BoomZ, BoomLength,-BoomY,BoomZ,BoomODia,TailP2)
        
        # insert elevator control surface hinge spars
        if Elevators:
            xpos1=BoomLength+(0.75-4.9*EChord/6.0)*myChordFunctionTailplane(ESpanStart)\
            *RootChord_TP*math.cos(math.radians(myTwistFunctionTailplane(ESpanStart)))
            zpos1=(0.75-4.9*EChord/6.0)*myChordFunctionTailplane(ESpanStart)\
            *RootChord_TP*math.sin(math.radians(myTwistFunctionTailplane(ESpanStart)))
            xpos2=BoomLength+(0.75*(1.0-EChord))*myChordFunctionTailplane(ESpanEnd)\
            *RootChord_TP*math.cos(math.radians(myTwistFunctionTailplane(ESpanEnd)))
            zpos2=(0.75*(1.0-EChord))*myChordFunctionTailplane(ESpanEnd)\
            *RootChord_TP*math.sin(math.radians(myTwistFunctionTailplane(ESpanEnd)))
            Tail = CutCylinder(xpos1,-ActualSemiSpan_TP,zpos1,xpos2,ActualSemiSpan_TP,zpos2,\
            HingeSparODia,Tail)
            TailS4 = CutCylinder(xpos1,0.0,zpos1,xpos2,ActualSemiSpan_TP,zpos2,\
            HingeSparODia,TailS4)
            SElevator = CutCylinder(xpos1,0.0,zpos1,xpos2,ActualSemiSpan_TP,zpos2,\
            HingeSparODia,SElevator)
            SElevatorSpar = CreateSpar(xpos1,0.0,zpos1,xpos2,ActualSemiSpan_TP,zpos2,\
            HingeSparODia-SparClearance,HingeSparIDia)
            TailP4 = CutCylinder(xpos1,0.0,zpos1,xpos2,-ActualSemiSpan_TP,zpos2,\
            HingeSparODia,TailP4)
            PElevator = CutCylinder(xpos1,0.0,zpos1,xpos2,-ActualSemiSpan_TP,zpos2,\
            HingeSparODia,PElevator)
            PElevatorSpar = CreateSpar(xpos1,0.0,zpos1,xpos2,-ActualSemiSpan_TP,zpos2,\
            HingeSparODia-SparClearance,HingeSparIDia)

if VTail:
    if CutStructure:
        AirFrame = rs.BooleanUnion([SPodSurf,TWing,MainEngineNacelle,PPodSurf])
    else:
        AirFrame= rs.BooleanUnion([SBoom,SPodSurf,PBoom,TWing,MainEngineNacelle,\
        PPodSurf, Tail, TailP, NoseGear, MainGearPort, MainGearStbd])
else:
    if CutStructure:
        AirFrame = rs.BooleanUnion([TWing,MainEngineNacelle])
    else:
        AirFrame= rs.BooleanUnion([SBoom,SPodSurf,PBoom,TWing,MainEngineNacelle,\
        PPodSurf, Tail, SFin, PFin, NoseGear, MainGearPort, MainGearStbd])

# =====  end of geometry construction ==========================================

# close Harpoon meshing files
# harpoonClose(fo, "Decode1coarse.cas", DesiredyPlus, Chord/1000.0, rho_C,\
#V_L, Visc_C, RefLev, FirstCellHtMult)
harpoonClose(fo, "Decode1fine.cas", DesiredyPlus, Chord/1000.0, rho_C,\
V_L, Visc_C, RefLev, FirstCellHtMult)
fo.close()
if 1: # set to 1 to write out files but 0 for wing optimization
    
    # now write out matching XFLR5 XML file - this can use the actual
    # wing sections from the Airconics geometry if a complex shape has been used
    # but currently uses NACA sections for the wing, tail elevator and fin to
    # reduce the number of sections to be analyseds by XFLR5
    
    fo = open("Decode1_XFLR5_plane.xml", "w")
    Posn=0.0
    AFrontVertPlane = rs.PlaneFromPoints([-10,0,0],[-10,1,0],[-10,1,1])   
    AFrontVertPlaneSrf =  rs.AddPlaneSurface(AFrontVertPlane, 2.5,2.5)
    ARearVertPlane = rs.PlaneFromPoints([10,0,0],[10,1,0],[10,1,1])   
    ARearVertPlaneSrf =  rs.AddPlaneSurface(ARearVertPlane, 2.5,2.5)
    y_position=[0]
    ClosestPoints = rs.CurveClosestObject(Sections[1], AFrontVertPlaneSrf)
    if ClosestPoints[2][0]>-9:
        LeadingPoint = rs.AddPoint(ClosestPoints[2])
        Xslice0 = ClosestPoints[2][0]
        Zslice0 = ClosestPoints[2][2]
    else:
        LeadingPoint = rs.AddPoint(ClosestPoints[1])
        Xslice0 = ClosestPoints[1][0]
        Zslice0 = ClosestPoints[1][2]
    xOffset=[Xslice0*1000.0]
    ClosestPoints = rs.CurveClosestObject(Sections[1], ARearVertPlaneSrf)
    if ClosestPoints[2][0]>-9:
        TrailingPoint = rs.AddPoint(ClosestPoints[2])
        Xslice1 = ClosestPoints[2][0]
        Zslice1 = ClosestPoints[2][2]
    else:
        TrailingPoint = rs.AddPoint(ClosestPoints[1])
        Xslice1 = ClosestPoints[1][0]
        Zslice1 = ClosestPoints[1][2]
    
    epsilon=0
    # use the next line for ACTUAL section data as written to files by AirConics
    # Airf,Chrd=myAirfoilFunctionTractor(epsilon, LeadingPoint, myChordFunctionTractor,\
    #ChordFactor, myDihedralFunctionTractor, myTwistFunctionTractor)
    # Optional: uncomment this to close the trailing edge with a line.
    # act.AddTEtoOpenAirfoil(Airf)
    
    Dihedral=[myDihedralFunctionTractor(0.0)]
#    use these three lines for ACTUAL section data as written to files by AirConics
#    ChordArray=[1000.0]
#    Twist=[0.0]
#    Foil=["myfoil_0.dat"]
    # use the next three lines INSTEAD to adopt a fixed NACA section instead of slice through the actual wing
    ChordArray=[math.sqrt((Xslice1-Xslice0)*(Xslice1-Xslice0)+(Zslice1-Zslice0)*(Zslice1-Zslice0))*1000.0]
    Twist=[myTwistFunctionTractor(0.0)]
    Foil=["NACA 23012"]
    
    # use the next line for ACTUAL section data as written to files by AirConics
    # curve2dat(Airf,100,'myfoil_0.dat', MoveLEtoZero = True, Channel = False)
    rs.DeleteObject(LeadingPoint)
    rs.DeleteObject(TrailingPoint)
    
    # set section positions to define wing in XFLR5
    Nepsilon = 0
    Vepsilon = [0.0, 0.2, 0.4, 0.599, ASpanStart, 0.8, 0.9, 0.95, 0.975, 0.979, ASpanEnd, 0.9875, 1.0]
    Tepsilon = 0.001+1.0/SegmentNo
    for ii in range (0, SegmentNo):
        epsilonM1=epsilon
        epsilon=ii/(SegmentNo-1)
        if ((ii > 0 and Nepsilon < len(Vepsilon) and math.fabs(epsilon - Vepsilon[Nepsilon])\
        < Tepsilon ) or (ii == SegmentNo-1)):
            if(Nepsilon < len(Vepsilon)):
                Vepsilon[Nepsilon] = -1.0
            Nepsilon = Nepsilon+1
            filnam = "myfoil_"+str(ii)+".dat"
            if ( Ailerons and epsilon >= ASpanStart and epsilon <= ASpanEnd ):
                filnam = "myfoil_flap_"+str(ii)+".dat"
            ClosestPoints = rs.CurveClosestObject(Sections[ii+1], AFrontVertPlaneSrf)
            if ClosestPoints[2][0]>-9:
                LeadingPoint = rs.AddPoint(ClosestPoints[2])
                Xslice0 = ClosestPoints[2][0]
                Zslice0 = ClosestPoints[2][2]
            else:
                LeadingPoint = rs.AddPoint(ClosestPoints[1])
                Xslice0 = ClosestPoints[1][0]
                Zslice0 = ClosestPoints[1][2]
            list.append(xOffset, Xslice0*1000.0)
            ClosestPoints = rs.CurveClosestObject(Sections[ii+1], ARearVertPlaneSrf)
            if ClosestPoints[2][0]>-9:
                TrailingPoint = rs.AddPoint(ClosestPoints[2])
                Xslice1 = ClosestPoints[2][0]
                Zslice1 = ClosestPoints[2][2]
            else:
                TrailingPoint = rs.AddPoint(ClosestPoints[1])
                Xslice1 = ClosestPoints[1][0]
                Zslice1 = ClosestPoints[1][2]
            list.append(y_position, 1000.0*ActualSemiSpan*epsilon)
            list.append(Dihedral, myDihedralFunctionTractor(epsilon))
            # use these three lines for ACTUAL section data as written to files by AirConics
#            list.append(ChordArray, 1000.0)
#            list.append(Twist, 0.0)
#            list.append(Foil, filnam)
            # use the next three lines INSTEAD to adopt a fixed NACA section instead of slice through the actual wing
            list.append(ChordArray, math.sqrt((Xslice1-Xslice0)*(Xslice1-Xslice0)\
            +(Zslice1-Zslice0)*(Zslice1-Zslice0))*1000.0)
            list.append(Twist, -myTwistFunctionTractor(epsilon))
            filnam = "NACA 23012"
            if ( Ailerons and epsilon >= ASpanStart and epsilon <= ASpanEnd ):
                filnam = "NACA 23012flap"
            list.append(Foil, filnam)
            
            # calculate offset and chord from leading and trailing edges
            # use the next line for ACTUAL section data as written to files by AirConics
            # Airf,Chrd=myAirfoilFunctionTractor(epsilon, LeadingPoint, myChordFunctionTractor,\
            #ChordFactor, myDihedralFunctionTractor, myTwistFunctionTractor)
            # Optional: uncomment this to close the trailing edge with a line.
            # act.AddTEtoOpenAirfoil(Airf)
            # use the next line for ACTUAL section data as written to files by AirConics
            # curve2dat(Airf,100,filnam, MoveLEtoZero = True, Channel = False)
            rs.DeleteObject(LeadingPoint)
            rs.DeleteObject(TrailingPoint)
    XFLR5Data(fo,"Decode1_Wing")
    XFLR5MainWing(fo, Posn, Nepsilon, y_position, ChordArray, xOffset, Dihedral, Twist, Foil)
    
    if CutStructure:
        print y_position
        for ii in range (0, Nepsilon):
            Spar, SubSpar = BooleanSplitY(y_position[Nepsilon-ii-1]/1000.0,CutterSize,Spar)
        for ii in range (0, Nepsilon-1):
            SubSpar, Spar = BooleanSplitY(-y_position[Nepsilon-ii-1]/1000.0,CutterSize,Spar)
    
    y_position_TP=[0]
    ClosestPoints = rs.CurveClosestObject(Sections_TP[1], AFrontVertPlaneSrf)
    if ClosestPoints[2][0]>-9:
        LeadingPoint = rs.AddPoint(ClosestPoints[2])
        Xslice0 = ClosestPoints[2][0]
        Zslice0 = ClosestPoints[2][2]
    else:
        LeadingPoint = rs.AddPoint(ClosestPoints[1])
        Xslice0 = ClosestPoints[1][0]
        Zslice0 = ClosestPoints[1][2]
    xOffset_TP=[Xslice0*1000.0]
    ClosestPoints = rs.CurveClosestObject(Sections_TP[1], ARearVertPlaneSrf)
    if ClosestPoints[2][0]>-9:
        TrailingPoint = rs.AddPoint(ClosestPoints[2])
        Xslice1 = ClosestPoints[2][0]
        Zslice1 = ClosestPoints[2][2]
    else:
        TrailingPoint = rs.AddPoint(ClosestPoints[1])
        Xslice1 = ClosestPoints[1][0]
        Zslice1 = ClosestPoints[1][2]
    ChordArray_TP=[math.sqrt((Xslice1-Xslice0)*(Xslice1-Xslice0)+(Zslice1-Zslice0)*(Zslice1-Zslice0))*1000.0]
    rs.DeleteObject(LeadingPoint)
    rs.DeleteObject(TrailingPoint)
    Dihedral_TP=[myDihedralFunctionTailplane(0.0)+CantAngle]
    Twist_TP=[-myTwistFunctionTailplane(0.0)]
    Foil_TP=["NACA 0212"] # NB fixed foil shape for TailPlane - ensure this matches definition used by AirConics
    epsilon=0
    
    # set section positions to define elevator in XFLR5
    Nepsilon = 0
    Vepsilon = [0.0, 0.2, 0.4, 0.6, 0.8, 0.9, 0.95, 0.975, 0.9875, 1.0]
    Tepsilon = 0.001+1.0/SegmentNo_TP
    for ii in range (0, SegmentNo_TP):
        epsilonM1=epsilon
        epsilon=ii/(SegmentNo-1)
        if ((ii > 0 and Nepsilon < len(Vepsilon) and math.fabs(epsilon - Vepsilon[Nepsilon])\
        < Tepsilon ) or (ii == SegmentNo_TP-1)):
            if(Nepsilon < len(Vepsilon)):
                Vepsilon[Nepsilon] = -1.0
            Nepsilon = Nepsilon+1
            list.append(y_position_TP, 1000.0*ActualSemiSpan_TP*epsilon)
            list.append(Dihedral_TP, myDihedralFunctionTailplane(epsilon)+CantAngle)
            list.append(Twist_TP, -myTwistFunctionTailplane(epsilon))
            list.append(Foil_TP, "NACA 0212") # NB fixed foil shape for TailPlane
            #                       ensure this matches definition used by AirConics
            # calculate offset and chord from leading and trailing edges
            ClosestPoints = rs.CurveClosestObject(Sections_TP[ii+1], AFrontVertPlaneSrf)
            if ClosestPoints[2][0]>-9:
                LeadingPoint = rs.AddPoint(ClosestPoints[2])
                Xslice0 = ClosestPoints[2][0]
                Zslice0 = ClosestPoints[2][2]
            else:
                LeadingPoint = rs.AddPoint(ClosestPoints[1])
                Xslice0 = ClosestPoints[1][0]
                Zslice0 = ClosestPoints[1][2]
            list.append(xOffset_TP, Xslice0*1000.0)
            ClosestPoints = rs.CurveClosestObject(Sections_TP[ii+1], ARearVertPlaneSrf)
            if ClosestPoints[2][0]>-9:
                TrailingPoint = rs.AddPoint(ClosestPoints[2])
                Xslice1 = ClosestPoints[2][0]
                Zslice1 = ClosestPoints[2][2]
            else:
                TrailingPoint = rs.AddPoint(ClosestPoints[1])
                Xslice1 = ClosestPoints[1][0]
                Zslice1 = ClosestPoints[1][2]
            list.append(ChordArray_TP, math.sqrt((Xslice1-Xslice0)*(Xslice1-Xslice0)\
            +(Zslice1-Zslice0)*(Zslice1-Zslice0))*1000.0)
            #print math.sqrt((Xslice1-Xslice0)*(Xslice1-Xslice0)+(Zslice1-Zslice0)\
            #*(Zslice1-Zslice0))*1000.0, Xslice1-Xslice0, Zslice1-Zslice0,\
            #1000.0*Chord*myChordFunctionNACA_low(epsilon)
            rs.DeleteObject(LeadingPoint)
            rs.DeleteObject(TrailingPoint)
    XFLR5Elevator(fo, Posn, Nepsilon, y_position_TP, ChordArray_TP, xOffset_TP,\
    Dihedral_TP, Twist_TP, Foil_TP)
    
    if VTail == False:
        y_position_Fin=[0]
        Tilt_angle=0.0
        Symetric=False
        isDoubleFin=True # there is a bug in XFLR5 for reading double fins in XML
        #               you have to edit the plane manually after reading the XML
        isSymFin=False
        ClosestPoints = rs.CurveClosestObject(Sections_Fin[1], AFrontVertPlaneSrf)
        if ClosestPoints[2][0]>-9:
            LeadingPoint = rs.AddPoint(ClosestPoints[2])
            Xslice0 = ClosestPoints[2][0]
            Zslice0 = ClosestPoints[2][2]
        else:
            LeadingPoint = rs.AddPoint(ClosestPoints[1])
            Xslice0 = ClosestPoints[1][0]
            Zslice0 = ClosestPoints[1][2]
        xOffset_Fin=[Xslice0*1000.0]
        ClosestPoints = rs.CurveClosestObject(Sections_Fin[1], ARearVertPlaneSrf)
        if ClosestPoints[2][0]>-9:
            TrailingPoint = rs.AddPoint(ClosestPoints[2])
            Xslice1 = ClosestPoints[2][0]
            Zslice1 = ClosestPoints[2][2]
        else:
            TrailingPoint = rs.AddPoint(ClosestPoints[1])
            Xslice1 = ClosestPoints[1][0]
            Zslice1 = ClosestPoints[1][2]
        ChordArray_Fin=[math.sqrt((Xslice1-Xslice0)*(Xslice1-Xslice0)+(Zslice1-Zslice0)\
        *(Zslice1-Zslice0))*1000.0]
        rs.DeleteObject(LeadingPoint)
        rs.DeleteObject(TrailingPoint)
        Dihedral_Fin=[myDihedralFunctionFin(0.0)]
        Twist_Fin=[-myTwistFunctionFin(0.0)]
        Foil_Fin=["NACA 0212"] # NB fixed foil shape for Fin
        #               ensure this matches definition used by AirConics
        epsilon=0
        
        # set section positions to define fin in XFLR5
        Nepsilon = 0
        Vepsilon = [0.0, 0.2, 0.4, 0.6, 0.8, 0.9, 0.95, 0.975, 0.9875, 1.0]
        Tepsilon = 0.001+1.0/SegmentNo_Fin
        for ii in range (0, SegmentNo_Fin):
            epsilonM1=epsilon
            epsilon=ii/(SegmentNo-1)
            if ((ii > 0 and Nepsilon < len(Vepsilon) and math.fabs(epsilon -\
            Vepsilon[Nepsilon]) < Tepsilon ) or (ii == SegmentNo_Fin-1)):
                if(Nepsilon < len(Vepsilon)):
                    Vepsilon[Nepsilon] = -1.0
                Nepsilon = Nepsilon+1
                list.append(y_position_Fin, 1000.0*ActualSemiSpan_Fin*epsilon)
                list.append(Dihedral_Fin, myDihedralFunctionFin(epsilon))
                list.append(Twist_Fin, -myTwistFunctionFin(epsilon))
                list.append(Foil_Fin, "NACA 0212") # NB fixed foil shape for Fin
                #           ensure this matches definition used by AirConics
                # calculate offset and chord from leading and trailing edges
                ClosestPoints = rs.CurveClosestObject(Sections_Fin[ii+1], AFrontVertPlaneSrf)
                if ClosestPoints[2][0]>-9:
                    LeadingPoint = rs.AddPoint(ClosestPoints[2])
                    Xslice0 = ClosestPoints[2][0]
                    Zslice0 = ClosestPoints[2][2]
                else:
                    LeadingPoint = rs.AddPoint(ClosestPoints[1])
                    Xslice0 = ClosestPoints[1][0]
                    Zslice0 = ClosestPoints[1][2]
                list.append(xOffset_Fin, Xslice0*1000.0)
                ClosestPoints = rs.CurveClosestObject(Sections_Fin[ii+1], ARearVertPlaneSrf)
                if ClosestPoints[2][0]>-9:
                    TrailingPoint = rs.AddPoint(ClosestPoints[2])
                    Xslice1 = ClosestPoints[2][0]
                    Zslice1 = ClosestPoints[2][2]
                else:
                    TrailingPoint = rs.AddPoint(ClosestPoints[1])
                    Xslice1 = ClosestPoints[1][0]
                    Zslice1 = ClosestPoints[1][2]
                list.append(ChordArray_Fin, math.sqrt((Xslice1-Xslice0)*(Xslice1-Xslice0)\
                +(Zslice1-Zslice0)*(Zslice1-Zslice0))*1000.0)
                #print math.sqrt((Xslice1-Xslice0)*(Xslice1-Xslice0)+(Zslice1-Zslice0)\
                #*(Zslice1-Zslice0))*1000.0, Xslice1-Xslice0, Zslice1-Zslice0,\
                #1000.0*Chord*myChordFunctionNACA_low(epsilon)
                rs.DeleteObject(LeadingPoint)
                rs.DeleteObject(TrailingPoint)
        XFLR5Fin(fo, Posn, y_tail_boom, Nepsilon, y_position_Fin, ChordArray_Fin,\
        xOffset_Fin, Dihedral_Fin, Twist_Fin, Foil_Fin, Tilt_angle, Symetric, isDoubleFin, isSymFin)
    
    XFLR5Close(fo)
    fo.close()
    rs.DeleteObject(AFrontVertPlaneSrf)
    rs.DeleteObject(ARearVertPlaneSrf)

# remove unwanted construction entities

rs.DeleteObjects(Sections)
rs.DeleteObjects(Sections_TP)
if VTail == False:
    rs.DeleteObjects(Sections_Fin)


# now write out matching Fluent journal files

    fo = open("C:\\Users\\Andy\\Documents\\airconicsv021\\Decode1_SA_coarse_15ms.jou", "w")
    for AoA in range(2,16,2):
        fluentData(fo, "Decode1coarse.cas", Awing/1000000.0, rho_C, Visc_C, Chord, 0.0, Vmax_C)
        fluentModel(fo, 0) # model 0 is SA while 1 is Ke and 2 is Kw
        fluentBCs(fo, Vmax_C, AoA, "rhinoceros_binary_stl___aug_10_2015.1", 0)
        fluentMethod(fo, 0, 0) # model and method, method 0 is first order while 1 is second order
        fluentInitc(fo, Vmax_C, AoA)
        fluentCalc(fo, 500, 0.2)
        fluentAdapt(fo, 30, 200)
        fluentCalc(fo, 500, 0.7)
        fluentMethod(fo, 0, 1) # model and method, method 0 is first order while 1 is second order
        fluentCalc(fo, 500, 0.7)
        fluentAdapt(fo, 50, 100)
        fluentCalc(fo, 500, 0.2)
        fluentRunOn(fo, 1500)
        fluentReport(fo, "Decode1_SA_coarse")
    fo.close()
    
    fo = open("Decode1_Ke_coarse_30ms.jou", "w")
    for AoA in range(2,16,2):
        fluentData(fo, "Decode1coarse.cas", Awing/1000000.0, rho_C, Visc_C, Chord, 0.0, Vmax_C)
        fluentInitc(fo, Vmax_C, AoA)
        if (AoA > -8):
            fluentModel(fo, 0) # model 0 is SA while 1 is Ke and 2 is Kw
            fluentBCs(fo, Vmax_C, AoA, "rhinoceros_binary_stl___aug_10_2015.1", 0)
            fluentMethod(fo, 0, 0) # model and method, method 0 is first order while 1 is second order
            fluentCalc(fo, 100, 0.2)
        fluentModel(fo, 1) # model 0 is SA while 1 is Ke and 2 is Kw
        fluentBCs(fo, Vmax_C, AoA, "rhinoceros_binary_stl___aug_10_2015.1", 1)
        fluentMethod(fo, 1, 0) # model and method, method 0 is first order while 1 is second order
        fluentCalc(fo, 500, 0.2)
        fluentAdapt(fo, 30, 200)
        fluentCalc(fo, 500, 0.7)
        fluentMethod(fo, 1, 1) # model and method, method 0 is first order while 1 is second order
        fluentCalc(fo, 500, 0.7)
        fluentAdapt(fo, 50, 100)
        fluentCalc(fo, 500, 0.2)
        fluentRunOn(fo, 1500)
        fluentReport(fo, "Decode1_Ke_coarse")
    fo.close()
    
    fo = open("Decode1_Kw_coarse_30ms.jou", "w")
    for AoA in range(-12,8,2):
        fluentData(fo, "Decode1coarse.cas", Awing/1000000.0, rho_C, Visc_C, Chord, 0.0, Vmax_C)
        fluentInitc(fo, Vmax_C, AoA)
        if (AoA > -8):
            fluentModel(fo, 0) # model 0 is SA while 1 is Ke and 2 is Kw
            fluentBCs(fo, Vmax_C, AoA, "rhinoceros_binary_stl___aug_10_2015.1", 0)
            fluentMethod(fo, 0, 0) # model and method, method 0 is first order while 1 is second order
            fluentCalc(fo, 100, 0.2)
        fluentModel(fo, 2) # model 0 is SA while 1 is Ke and 2 is Kw
        fluentBCs(fo, Vmax_C, AoA, "rhinoceros_binary_stl___aug_10_2015.1", 2)
        fluentMethod(fo, 2, 0) # model and method, method 0 is first order while 1 is second order
        fluentCalc(fo, 500, 0.2)
        fluentAdapt(fo, 30, 200)
        fluentCalc(fo, 500, 0.7)
        fluentMethod(fo, 2, 1) # model and method, method 0 is first order while 1 is second order
        fluentCalc(fo, 500, 0.7)
        fluentAdapt(fo, 50, 100)
        fluentCalc(fo, 500, 0.2)
        fluentRunOn(fo, 1500)
        fluentReport(fo, "Decode1_Kw_coarse")
    fo.close()
    
    
    fo = open("Decode1_SA_fine_30ms.jou", "w")
    for AoA in range(2,16,2):
        fluentData(fo, "Decode1fine.cas", Awing/1000000.0, rho_C, Visc_C, Chord, 0.0, Vmax_C)
        fluentModel(fo, 0) # model 0 is SA while 1 is Ke and 2 is Kw
        fluentBCs(fo, Vmax_C, AoA, "rhinoceros_binary_stl___aug_10_2015.1", 0)
        fluentMethod(fo, 0, 0) # model and method, method 0 is first order while 1 is second order
        fluentInitc(fo, Vmax_C, AoA)
        fluentCalc(fo, 1500, 0.2)
        fluentAdapt(fo, 0, 5)
        fluentCalc(fo, 1500, 0.7)
        fluentMethod(fo, 0, 1) # model and method, method 0 is first order while 1 is second order
        fluentCalc(fo, 1500, 0.7)
        fluentAdapt(fo, 0.5, 1.0)
        fluentCalc(fo, 1500, 0.2)
        fluentRunOn(fo, 1500)
        fluentReport(fo, "Decode1_SA_fine")
    fo.close()
    
    fo = open("Decode1_Ke_fine_30ms.jou", "w")
    for AoA in range(2,16,2):
        fluentData(fo, "Decode1fine.cas", Awing/1000000.0, rho_C, Visc_C, Chord, 0.0, Vmax_C)
        fluentInitc(fo, Vmax_C, AoA)
        if (AoA > -8):
            fluentModel(fo, 0) # model 0 is SA while 1 is Ke and 2 is Kw
            fluentBCs(fo, Vmax_C, AoA, "rhinoceros_binary_stl___aug_10_2015.1", 0)
            fluentMethod(fo, 0, 0) # model and method, method 0 is first order while 1 is second order
            fluentCalc(fo, 100, 0.2)
        fluentModel(fo, 1) # model 0 is SA while 1 is Ke and 2 is Kw
        fluentBCs(fo, Vmax_C, AoA, "rhinoceros_binary_stl___aug_10_2015.1", 1)
        fluentMethod(fo, 1, 0) # model and method, method 0 is first order while 1 is second order
        fluentCalc(fo, 1500, 0.2)
        fluentAdapt(fo, 0, 5)
        fluentCalc(fo, 1500, 0.7)
        fluentMethod(fo, 1, 1) # model and method, method 0 is first order while 1 is second order
        fluentCalc(fo, 1500, 0.7)
        fluentAdapt(fo, 0.5, 1.0)
        fluentCalc(fo, 1500, 0.2)
        fluentRunOn(fo, 1500)
        fluentReport(fo, "Decode1_Ke_fine")
    fo.close()
    
    fo = open("Decode1_Kw_fine_30ms.jou", "w")
    for AoA in range(-4,18,1):
        fluentData(fo, "Decode1fine.cas", Awing/1000000.0, rho_C, Visc_C, Chord, 0.0, Vmax_C)
        fluentInitc(fo, Vmax_C, AoA)
        if (AoA > -8):
            fluentModel(fo, 0) # model 0 is SA while 1 is Ke and 2 is Kw
            fluentBCs(fo, Vmax_C, AoA, "rhinoceros_binary_stl___aug_10_2015.1\nrhinoceros_binary_stl___aug_10_2015.1.1", 0)
            fluentMethod(fo, 0, 0) # model and method, method 0 is first order while 1 is second order
            fluentCalc(fo, 150, 0.2)
        fluentModel(fo, 2) # model 0 is SA while 1 is Ke and 2 is Kw
        fluentBCs(fo, Vmax_C, AoA, "rhinoceros_binary_stl___aug_10_2015.1\nrhinoceros_binary_stl___aug_10_2015.1.1", 2)
        fluentMethod(fo, 2, 0) # model and method, method 0 is first order while 1 is second order
        fluentCalc(fo, 500, 0.2)
        fluentAdapt(fo, 0, 5)
        fluentCalc(fo, 500, 0.7)
        fluentMethod(fo, 2, 1) # model and method, method 0 is first order while 1 is second order
        fluentCalc(fo, 500, 0.7)
        fluentAdapt(fo, 0.5, 1.0)
        fluentCalc(fo, 50, 0.2)
        fluentAdapt(fo, 0.5, 1.0)
        fluentCalc(fo, 50, 0.2)
        fluentAdapt(fo, 0.5, 1.0)
        fluentCalc(fo, 500, 0.2)
        fluentRunOn(fo, 1000)
        fluentReport(fo, "Decode1_Kw_fine")
    fo.close()
