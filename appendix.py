# Example Python script for building a twin boom UAV geometry, to be run in the
# Rhinoceros 5.0 CAD engine.
# ==============================================================================
# Provided as part of Keane, A. J., Sobester, A., Scanlan, J. P., "Small 
# Unmanned Fixed-wing Aircraft Design: A Practical Approach", John Wiley & Sons,
# 2017.
#
# This script uses the AirCONICS aircraft geometry toolbox (included):
# Aircraft CONfiguration through Integrated Cross-disciplinary Scripting 
# version 0.1.1b
# Andras Sobester, 2014.
#
# It also uses the UIUC library of airfoil coordinates, downloaded in 2016 from
# the following URL: http://m-selig.ae.illinois.edu/ads/coord_database.html
# ==============================================================================
#
# Before running this script, please edit airconics_setup.py, where you can 
# specify the path where the code provided is installed, as well as the location
# of the airfoil coordinate library.
# ==============================================================================

# This program, including the AirCONICS aicraft geometry toolbox, are free 
# software: you can redistribute it and/or modify  it under the terms of the GNU
# Lesser General Public License as published by the Free Software Foundation, 
# either version 3 of the License, or any later version.
# 
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser
# General Public License for more details.
# 
# You should have received a copy of the GNU General Public License and GNU
# Lesser General Public License along with this program. If not, see
# <http://www.gnu.org/licenses/>.


#===============================================================================
# PREAMBLE
#===============================================================================
from __future__ import division
import math, rhinoscriptsyntax as rs
import primitives 
import airconics_setup
import liftingsurface
import AirCONICStools as act
import HighLiftDevices as HLD
import landinggear as LG
import uavauxtools as UAT
import payload


global BoomInner, BoomOuter
global SettingAngle, TaperRatio
global fo

rs.UnitAbsoluteTolerance(0.0001)
rs.UnitAngleTolerance(0.1)
rs.UnitRelativeTolerance(0.1)

#===============================================================================
# GEOMETRY PARAMETERS
#===============================================================================
 
# === Airframe geometry parameters =============================================

# Name	=	Value	#	Long Name / Definition
Awing	=	965830.1	#	total wing area Dec: 965830.1
AR	=	9	#	aspect  ratio (span^2 / area)  Decode1: 9 [6,20]
TaperRatio = 0.8 #Decode1: 0.8 [0.6, 1]

Dprop	=	435.0	#	propeller diameter d: 435

Atail	=	158945.7	#	tailplane area
ARtail	=	4.0	#	tailplane aspect ratio (span^2 / area)

Height_fin	=	293.0	#	fin height (or semi-span) for two fins
Chord_fin	=	195.3	#	fin mean chord

Depth_Fuse	=	200.0	#	fuselage depth d: 200
Width_Fuse	=	150.0	#	fuselage width d: 150

Len_Nose	=	150.0	#	nose length (forward of front bulkhead) Dec: 150

Dia_Wheels	=	100.0	#	diam of main undercariage wheels
Len_Engine	=	125.0	#	length of engine

x_fnt_bkhd	=	640.3	#	long position of front bulkhead Dec: 640.3
x_tail_spar	=	-995.3	#	long position of tailplane spar

x_rear_bkhd	=	-200.0	#	long position of rear bulkhead Dec -200
x_mid_bkhd	=	220.2	#	long position of middle bulkhead
z_fuse_base	=	-110.0	#	vert position of base of fuselage
z_tail_boom	=	0.0	#	vert position of tailboom
z_engine	=	60.0	#	vert position of engine
z_uncarriage	=	-300.0	#	vert position of centre of main d:-300
                            # undercarriage wheels
y_tail_boom	=	239.2	#	horizontal position of tail booms
x_main_spar	=	0.0	#	long position of main spar

FinAngle = 90



SWEEP_QTR_deg = 0
SWEEP_QTR_rad = math.radians(SWEEP_QTR_deg)
SWEEP_LE_rad = math.atan(math.tan(SWEEP_QTR_rad)+\
(1/AR)*(1-TaperRatio)/(1+TaperRatio))
SWEEP_LE_deg = math.degrees(SWEEP_LE_rad)


# Aileron geometry
ASpanStart = 0.6
ASpanEnd = 0.98
AChord = 0.3
ATaper = 1.0
ADeflectionAngle = 20.0

# Elevator geometry
ESpanStart = 0.01
ESpanEnd = 0.98
EChord = 0.3
ETaper = 1.0
EDeflectionAngle = 0.0

# Rudder geometry
RSpanStart = 0.0555
RSpanEnd = 0.98
RChord = 0.3
RTaper = 1.0
RDeflectionAngle = 0.0


# Use these flags to turn components on and off

VTail = False
Pod = True
NoseGear = False
Ailerons = True
Elevators = True
Rudder = True


# The geometry of the internal structure
# (all dimensions in m)
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

# cut up the structure if needed
CutStructure = True



# ============= Variables to sweep =============================================
# EXAMPLE: 'Decode 1' pusher UAV

PayloadPodLength = 0.4

#-x_fnt_bkhd/1000.0, z_uncarriage/1000 - nose/tail gear length
# NoseGear - if False tail gear is drawn

# ==============================================================================


Span = math.sqrt(AR*Awing)
Chord = Awing/Span

Span_tail = math.sqrt(ARtail*Atail)
Chord_tail = Atail/Span_tail


# ==== 3D wing main wing definition ============================================
# ==== through definition of spanwise parameter variations =====================

def myDihedralFunctionPusher(Epsilon):
    # User-defined function describing the variation of dihedral as a function
    # of the leading edge coordinate
    return 0

def myTwistFunctionPusher(Epsilon):
    # User-defined function describing the variation of twist as a function
    # of the leading edge coordinate
    RootTwist = 0
    TipTwist = 2 # typical reduction in twist to give washout
    TipTwist = 0
    return RootTwist + Epsilon*(TipTwist-RootTwist)

def myChordFunctionPusher(Epsilon):
    # User-defined function describing the variation of chord as a function of 
    # the leading edge coordinate
    
    if Epsilon < 0.2:
        return 1
    else:
        ChordLengths = [1, TaperRatio]
        EpsArray = [0.2, 1]
        f = act.linear_interpolation(EpsArray, ChordLengths)
        return f(Epsilon)

def myAirfoilFunctionPusher(Epsilon, LEPoint, ChordFunct, ChordFactor, \
    DihedralFunct, TwistFunct):
    # Defines the variation of cross section as a function of Epsilon
    
    AirfoilChordLength = (ChordFactor*ChordFunct(Epsilon))/math.cos \
    (math.radians(TwistFunct(Epsilon)))

    # Instantiate class to set up a generic airfoil with these basic parameters
    Af = primitives.Airfoil(LEPoint, AirfoilChordLength, DihedralFunct(Epsilon)\
    , TwistFunct(Epsilon),
    EnforceSharpTE = True)

    SmoothingPasses = 1
    
    # Add airfoil curve to document and retrieve handles to it and its chord
    # - in this case NACA23012, with DesignLiftCoefficient = 0.3,
    # MaxCamberLocFracChord = 0.15 and MaxThicknessPercChord = 15
    Airf,Chrd = primitives.Airfoil.AddNACA5(Af, 0.3, 0.15, 15, SmoothingPasses)
    
    # A possible alternative - a NACA 4-digit section (e.g., NACA2212)
    # Airf,Chrd = primitives.Airfoil.AddNACA4(Af, 2, 2, 12, SmoothingPasses)
    
    return Airf, Chrd
    
def mySweepAngleFunctionPusher(Epsilon):
    # User-defined function describing the variation of sweep angle as a
    # function of the leading edge coordinate
    return SWEEP_LE_deg




# ==== 3D tailplane definition =================================================
# ==== through definition of spanwise parameter variations =====================

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

    # Add NACA0012 airfoil curve to document and retrieve handles to it and
    # its chord
    Airf,Chrd = primitives.Airfoil.AddNACA4(Af, 0, 0, 12, SmoothingPasses)
    
    return Airf, Chrd

def mySweepAngleFunctionTailplane(Epsilon):
    # User-defined function describing the variation of sweep angle as a funct.
    # of the leading edge coordinate
    return 0

def myDihedralFunctionTailplane(Epsilon):
    # User-defined function describing the variation of dihedral as a function
    # of the leading edge coordinate
    return 0

#==== end of tailplane definition ==============================================


# ==== 3D tailfin definition ===================================================
# ==== through definition of spanwise parameter variations =====================

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

    # Add NACA0012 airfoil curve to document and retrieve handles to it and
    # its chord
    Airf,Chrd = primitives.Airfoil.AddNACA4(Af, 0, 0, 12, SmoothingPasses)

    return Airf, Chrd
    
def mySweepAngleFunctionFin(Epsilon):
    # User-defined function describing the variation of sweep angle as a fn
    # of the leading edge coordinate
    return 0

def myDihedralFunctionFin(Epsilon):
    # User-defined function describing the variation of dihedral as a function
    # of the leading edge coordinate
    return 0

#========= end of tailfin definition ===========================================

#========== END GEOMETRY PARAMETER DEFINITION ==================================




#===============================================================================
# AIRFRAME GEOMETRY GENERATION
#===============================================================================

rs.EnableRedraw(False)

# Wing apex location
P = (0,0,0)

LooseSurf = 3 # Tightly fit surface (loft_type arhument in AddLoftSrf)
SegmentNo = 2 # Number of airfoil sections to loft wing surface over

SettingAngle = 2.53 # as computed with XFLR5 to give Cl of 0.28 at 30 m/s

TipRequired = 1 # Flat wingtip

SectionsRequired = False # when optimizing this currently needs to be False but
                         # when creating XFLR5 files it must be True

Wing = liftingsurface.LiftingSurface(P, mySweepAngleFunctionPusher,\
myDihedralFunctionPusher, myTwistFunctionPusher, myChordFunctionPusher,\
myAirfoilFunctionPusher, LooseSurf, SegmentNo, TipRequired, SectionsRequired)

# The wing thus defined, we now scale it to the target aspect ratio and area
Wing.TargetAspectRatio = AR
Wing.TargetArea = Awing/1000000.0
Wing.wTargetAspectRatio = 1
Wing.wTargetArea = 1

# Initial iterate
ChordFactor = 0.26
ScaleFactor = 1.47

OptimizeChordScale=1

SLS, ActualSemiSpan, LSP_area,  RootChord, AR, SWingTip, Sections =\
Wing.GenerateLiftingSurface(ChordFactor, ScaleFactor, OptimizeChordScale)


# Rotate the wing to the specified setting angle
TipX = math.tan( SWEEP_LE_rad ) * ActualSemiSpan
RotVec = rs.VectorCreate((0,0,0),(0,1,0))
SLS = rs.RotateObject(SLS, (0,0,0), -SettingAngle, axis = RotVec)
SWingTip = rs.RotateObject(SWingTip, (0,0,0), -SettingAngle, axis = RotVec)
SLS_TE = UAT.ExtractTrailingEdge(SLS)
# shift wing up to allow for setting angle about LE and 
# not 1/4 chord point where we attach it
SLS = rs.MoveObject(SLS,\
(0.0, 0.0, 0.25*RootChord*math.sin(math.radians(SettingAngle))))
SWingTip = rs.MoveObject(SWingTip,\
(0.0, 0.0, 0.25*RootChord*math.sin(math.radians(SettingAngle))))
SLS_TE = rs.MoveObject(SLS_TE,\
(0.0, 0.0, 0.25*RootChord*math.sin(math.radians(SettingAngle))))
PWingTip = act.MirrorObjectXZ(SWingTip)
rs.DeleteObject(SLS_TE)

# Adding ailerons to each wing
if Ailerons:

    SLScopy=rs.CopyObject(SLS)
    Aflapconfig1 = \
    ['simpleflap', ASpanStart, ASpanEnd, AChord, ATaper, ADeflectionAngle]
    Devices = [Aflapconfig1]
    SAileron, Area, Cutter, Cutter1, Cutter2, SAileronCutBrick = \
    HLD.AddHighLiftDevices(SLS, Devices)
    rs.DeleteObjects([Cutter, Cutter1, Cutter2])
    PLS = act.MirrorObjectXZ(SLS)
    PAileron = act.MirrorObjectXZ(SAileron)

else:
    PLS = act.MirrorObjectXZ(SLS)


# =====  Booms =============================================================

PodLength = 1.5*Chord/1000.0
BoomLength = -x_tail_spar/1000.0
BoomY = y_tail_boom/1000.0
BoomZ = z_tail_boom/1000.0
[SPodSurf] = payload.pod([0.025*PodLength,BoomY,BoomZ],PodLength,11, 0.1)
if CutStructure:
    SPodSurf = UAT.CutCylinder(RootChord/4.0,\
    BoomY,BoomZ, BoomLength,BoomY,BoomZ,BoomODia,SPodSurf)

    SBoom = UAT.CreateSpar(RootChord/4.0+MainSparODia/2.0,BoomY,BoomZ,\
    BoomLength-TailSparODia/2.0,BoomY,BoomZ, BoomODia-SparClearance, BoomIDia)
else:
    SBoom = \
    rs.AddCylinder([RootChord/4.0,BoomY,BoomZ], [BoomLength,BoomY,BoomZ], 0.01)

PPodSurf = act.MirrorObjectXZ(SPodSurf)
PBoom = act.MirrorObjectXZ(SBoom)
if CutStructure: # cut booms to enable biased meshing along their lengths away
#                   from where the pod ends
    SBoom, SBoom1 = UAT.BooleanSplitX(0.875*PodLength,CutterSize,SBoom)
    PBoom, PBoom1 = UAT.BooleanSplitX(0.875*PodLength,CutterSize,PBoom)



# =====  Tailplane =============================================================
TailChord = Chord_tail/1000.0
FinX = -x_tail_spar/1000.0-TailChord/4.0
P_TP = [FinX,0,0]
LooseSurf_TP = 1
SegmentNo_TP = 2

TipRequired_TP = 1
SectionsRequired_TP = False 

Tailplane = liftingsurface.LiftingSurface(P_TP, mySweepAngleFunctionTailplane,\
myDihedralFunctionTailplane, myTwistFunctionTailplane, myChordFunctionTailplane,\
myAirfoilFunctionTailplane, LooseSurf_TP, SegmentNo_TP, TipRequired_TP,\
SectionsRequired_TP)

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

OptimizeChordScale_TP = True

SLS_TP, ActualSemiSpan_TP, LSP_area_TP,  \
RootChord_TP, AR_TP, SWingTip_TP, Sections_TP =\
Tailplane.GenerateLiftingSurface(ChordFactor_TP, ScaleFactor_TP, OptimizeChordScale_TP)

SLS_TP_TE = UAT.ExtractTrailingEdge(SLS_TP)

if Elevators:
    Eflapconfig = \
    ['simpleflap', ESpanStart, ESpanEnd, EChord, ETaper, EDeflectionAngle]
    Devices = [Eflapconfig]
    SElevator, Area, Cutter, Cutter1, Cutter2, CutBrick = \
    HLD.AddHighLiftDevices(SLS_TP, Devices)
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

# shift tail down to allow for setting angle about LE and 
# not 1/4 chord point where we attach it

Tail = rs.MoveObject(Tail,(0.0, 0.0, -0.25*RootChord_TP*math.sin\
(math.radians(myTwistFunctionTailplane(0.0)))))
SLS_TP_TE = rs.MoveObject(SLS_TP_TE,(0.0, 0.0, -0.25*RootChord_TP*math.sin\
(math.radians(myTwistFunctionTailplane(0.0)))))

if VTail:
    RotVec = rs.VectorCreate([FinX,y_tail_boom/1000.0,0],[FinX+1,y_tail_boom/1000.0,0])
    RotCent = [FinX,y_tail_boom/1000.0,0]
    if CutStructure:
        #Tail = rs.BooleanUnion([Tail, SFin, PFin])
        Tail = UAT.CutCylinder(BoomLength,-ActualSemiSpan_TP,BoomZ, BoomLength,\
        ActualSemiSpan_TP,BoomZ,TailSparODia,Tail)
        TailSpar = UAT.CreateSpar(BoomLength,-ActualSemiSpan_TP,BoomZ, BoomLength,\
        ActualSemiSpan_TP,BoomZ, TailSparODia-SparClearance, TailSparIDia)

        Tail, TailS1 = UAT.BooleanSplitY(BoomY-(y_tail_boom/1000.0*(1.0-1.0/math.cos\
        (CantAngle))+ActualSemiSpan_TP)+0.05*Chord/1000.0,CutterSize,Tail)
        Tail, TailS2 = UAT.BooleanSplitY(BoomY-(y_tail_boom/1000.0*(1.0-1.0/math.cos\
        (CantAngle))+ActualSemiSpan_TP)-0.05*Chord/1000.0,CutterSize,Tail)
        Tail, TailS3 = UAT.BooleanSplitY(ESpanStart*ActualSemiSpan_TP+0.00175,CutterSize,Tail)
        TailS1, TailS4 = UAT.BooleanSplitY(ESpanEnd*ActualSemiSpan_TP-0.00075,CutterSize,TailS1)
        Tail, TailS5 = UAT.BooleanSplitY(-ESpanStart*ActualSemiSpan_TP-0.00175,CutterSize,Tail)
        Tail, TailS6 = UAT.BooleanSplitY(-ESpanEnd*ActualSemiSpan_TP+0.00075,CutterSize,Tail)
    
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
    rs.DeleteObject(SLS_TP_TE)
    CentrePod = payload.pod([FinX, 0, y_tail_boom/1000.0*math.tan(CantAngle)],RootChord_TP,12, 0.0)
    
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
        TailS2 = UAT.CutCylinder(0.25*Chord/1000.0,BoomY,BoomZ, BoomLength,BoomY,BoomZ,BoomODia,TailS2)
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
    
    rs.DeleteObjects([SLS_Fin,SWingTip_Fin])



# =====  CentrePod =============================================================

if Pod:
    
    CentrePod = payload.pod([-0.05, 0, z_uncarriage/1000.0/2],\
    PayloadPodLength,14, 0.3)

    P_Fin = [0.05,0,0]
    FinChord = Chord/1000.0
    FinSpan = -z_uncarriage/1000.0/4.0
    FinTC = 12
    LooseSurf_Fin = 1
    SegmentNo_Fin= 2
    TipRequired_Fin = 1
    SectionsRequired_Fin = False
    
    PodFin = liftingsurface.LiftingSurface(P_Fin,\
    mySweepAngleFunctionTailplane,\
    myDihedralFunctionTailplane, myTwistFunctionTailplane,\
    myChordFunctionTailplane,\
    myAirfoilFunctionTailplane, LooseSurf_Fin, SegmentNo_Fin, TipRequired_Fin,\
    SectionsRequired_Fin)

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

MainEngineNacelle = payload.pod([-(Len_Nose+x_fnt_bkhd)/1000.0, 0, 0],FuselageLength,\
FuselageFinennesInPercent, 0.6)

PropDia = Dprop/1000.0
PropX = (2*Len_Engine-x_rear_bkhd-Width_Fuse/4.0)/1000.0

MainPropP1 = rs.AddPoint([PropX,0,PropDia/2])
MainPropP2 = rs.AddPoint([PropX,PropDia/2,0])
MainPropP3 = rs.AddPoint([PropX,0,-PropDia/2])
MainProp = rs.AddCircle3Pt(MainPropP1,MainPropP2,MainPropP3)

PropDisk = rs.AddPlanarSrf(MainProp)

rs.DeleteObjects([MainPropP1,MainPropP2,MainPropP3,MainProp])




# =====  Landing gear =============================================================

if NoseGear:
    # Nose gear
    NoseWheelScaleFactor = 1 # with respect to main gear
    NoseWheelX = -x_fnt_bkhd/1000.0
    NoseWheelCentre = [NoseWheelX, 0, z_uncarriage/1000.0]
    NoseWheelRadius = 0.5*NoseWheelScaleFactor*Dia_Wheels/1000.0
    NoseStrutLength = -z_uncarriage/1000
    CompleteNoseGear = LG.LandingGear(NoseWheelCentre, NoseWheelRadius, NoseStrutLength)
else:
    # Tail gear
    TailWheelScaleFactor = 0.5 # with respect to main gear
    TailWheelX = -x_tail_spar/1000.0
    STailWheelCentre = [TailWheelX, BoomY, z_uncarriage/1000.0/2]
    TailWheelRadius = 0.5*TailWheelScaleFactor*Dia_Wheels/1000.0
    TailStrutLength = -z_uncarriage/1000/2
    TailGearStbd = LG.LandingGear(STailWheelCentre, TailWheelRadius, TailStrutLength)
    TailGearPort = act.MirrorObjectXZ(TailGearStbd)

# MainGear
MainWheelX = -x_rear_bkhd/1000.0
MainWheelY = y_tail_boom/1000.0
MainWheelCentre = [MainWheelX, MainWheelY, z_uncarriage/1000.0]
MainWheelRadius = 0.5*Dia_Wheels/1000.0
MainStrutLength = -z_uncarriage/1000
MainGearPort = LG.LandingGear(MainWheelCentre, MainWheelRadius, MainStrutLength)
MainGearStbd = act.MirrorObjectXZ(MainGearPort)





# =====  Assembly =============================================================
# now union all the bits together to create the airframe
#
TWing = rs.JoinSurfaces([SLS,PLS,SWingTip, PWingTip])
rs.DeleteObjects([SLS,PLS,SWingTip, PWingTip])


if CutStructure:
    TWing = UAT.CutCylinder(RootChord/4.0,-ActualSemiSpan,0.0,RootChord/4.0,ActualSemiSpan,\
    0.0,MainSparODia,TWing)
    Spar = UAT.CreateSpar(RootChord/4.0,-ActualSemiSpan,0.0,RootChord/4.0,ActualSemiSpan,\
    0.0,MainSparODia-SparClearance,MainSparIDia)
    TWing, TWingS1 =   UAT.BooleanSplitY(BoomY+0.1*Chord/1000.0,CutterSize,TWing)
    TWingP1, TWing =   UAT.BooleanSplitY(-BoomY-0.1*Chord/1000.0,CutterSize,TWing)
    TWing, TWingS8 =   UAT.BooleanSplitY(BoomY-0.1*Chord/1000.0,CutterSize,TWing)
    TWingP8, TWing =   UAT.BooleanSplitY(-BoomY+0.1*Chord/1000.0,CutterSize,TWing)
    TWingS2, TWingS3 = UAT.BooleanSplitY(0.9945*ASpanStart*Span/2000.0-RibThick,CutterSize,TWingS1)
    TWingS4, TWingS5 = UAT.BooleanSplitY(0.9945*ASpanStart*Span/2000.0,CutterSize,TWingS3)
    TWingS6, TWingS7 = UAT.BooleanSplitY(0.9991*ASpanEnd*Span/2000.0,CutterSize,TWingS5)
    TWingP3, TWingP2 = UAT.BooleanSplitY(-0.9945*ASpanStart*Span/2000.0+RibThick,CutterSize,TWingP1)
    TWingP5, TWingP4 = UAT.BooleanSplitY(-0.9945*ASpanStart*Span/2000.0,CutterSize,TWingP3)
    TWingP7, TWingP6 = UAT.BooleanSplitY(-0.9991*ASpanEnd*Span/2000.0,CutterSize,TWingP5)
    
    SPodSurf = UAT.CutCylinder(RootChord/4.0,-ActualSemiSpan,0.0,RootChord/4.0,ActualSemiSpan,0.0,\
    MainSparODia,SPodSurf)
    PPodSurf = UAT.CutCylinder(RootChord/4.0,-ActualSemiSpan,0.0,RootChord/4.0,ActualSemiSpan,0.0,\
    MainSparODia,PPodSurf)
    TWingS8 = UAT.CutCylinder(RootChord/4.0,BoomY,BoomZ, BoomLength,BoomY,BoomZ,BoomODia,TWingS8)
    TWingP8 = UAT.CutCylinder(RootChord/4.0,-BoomY,BoomZ, BoomLength,-BoomY,BoomZ,BoomODia,TWingP8)
    SPodSurf = rs.BooleanUnion([SPodSurf,TWingS8])
    PPodSurf = rs.BooleanUnion([PPodSurf,TWingP8])
    
    
    # insert aileron control surface hinge spars
    if Ailerons:
        print('Aileron hinges OFF')
        xpos1=((1.0-4.9*AChord/6.0)*myChordFunctionPusher(ASpanStart))\
        *RootChord*math.cos(math.radians(SettingAngle))
        ypos1a=0.994*ASpanStart*ActualSemiSpan-RibThick
        ypos1b=-0.994*ASpanStart*ActualSemiSpan+RibThick
        zpos1=-((1.0-4.9*AChord/6.0)*myChordFunctionPusher(ASpanStart)-0.25)\
        *RootChord*math.sin(math.radians(SettingAngle))
        xpos2=((1.0-5.0*AChord/6.0)*myChordFunctionPusher(ASpanEnd))\
        *RootChord*math.cos(math.radians(SettingAngle))
        ypos2=1.001*ActualSemiSpan
        zpos2=-((1.0-5.0*AChord/6.0)*myChordFunctionPusher(ASpanEnd)-0.25)\
        *RootChord*math.sin(math.radians(SettingAngle))
        TWingS4 = UAT.CutCylinder(xpos1,ypos1a,zpos1,xpos2,ypos2,zpos2,HingeSparODia,TWingS4)
        TWingS7 = UAT.CutCylinder(xpos1,ypos1a,zpos1,xpos2,ypos2,zpos2,HingeSparODia,TWingS7)
        SAileronSpar = UAT.CreateSpar(xpos1,ypos1a,zpos1,xpos2,ypos2,zpos2,HingeSparODia\
        -SparClearance,HingeSparIDia)
        TWingP4 = UAT.CutCylinder(xpos1,ypos1b,zpos1,xpos2,-ypos2,zpos2,HingeSparODia,TWingP4)
        TWingP7 = UAT.CutCylinder(xpos1,ypos1b,zpos1,xpos2,-ypos2,zpos2,HingeSparODia,TWingP7)
        PAileronSpar = UAT.CreateSpar(xpos1,ypos1b,zpos1,xpos2,-ypos2,zpos2,HingeSparODia\
        -SparClearance,HingeSparIDia)
        
        
        PAileronCutBrick = act.MirrorObjectXZ(SAileronCutBrick)
        PAileronCutBrick_cpy = rs.CopyObject(PAileronCutBrick,[0,-RibThick,0])
        SAileronCutBrick_cpy = rs.CopyObject(SAileronCutBrick,[0,RibThick,0])

        TWingP6 = rs.BooleanDifference(TWingP6,PAileronCutBrick)
        TWingP6 = rs.BooleanDifference(TWingP6,PAileronCutBrick_cpy)

        TWingS6 = rs.BooleanDifference(TWingS6,SAileronCutBrick)
        TWingS6 = rs.BooleanDifference(TWingS6,SAileronCutBrick_cpy)

    
    
    if VTail == False:
        SFin = UAT.CutCylinder(FinX+RootChord_TP/30.0+RootChord_Fin/4.0,BoomY,BoomZ,\
        FinX+RootChord_TP/30.0+RootChord_Fin/4.0,BoomY,BoomZ+CutterSize, FinSparODia,SFin)
        SFinSpar = UAT.CreateSpar(FinX+RootChord_TP/30.0+RootChord_Fin/4.0,BoomY,BoomZ+BoomODia/2,\
        FinX+RootChord_TP/30.0+RootChord_Fin/4.0,BoomY,BoomZ+ActualSemiSpan_Fin, FinSparODia\
        -SparClearance, FinSparIDia)
        PFin = UAT.CutCylinder(FinX+RootChord_TP/30.0+RootChord_Fin/4.0,-BoomY,BoomZ,\
        FinX+RootChord_TP/30.0+RootChord_Fin/4.0,-BoomY,BoomZ+CutterSize, FinSparODia,PFin)
        PFinSpar = UAT.CreateSpar(FinX+RootChord_TP/30.0+RootChord_Fin/4.0,-BoomY,BoomZ+BoomODia/2\
        ,FinX+RootChord_TP/30.0+RootChord_Fin/4.0,-BoomY,BoomZ+ActualSemiSpan_Fin, FinSparODia\
        -SparClearance, FinSparIDia)
        
        SFin, SFin1 = UAT.BooleanSplitZ(BoomZ+0.0128,CutterSize,SFin)
        PFin, PFin1 = UAT.BooleanSplitZ(BoomZ+0.0128,CutterSize,PFin)
        SFin1, SFin2 = UAT.BooleanSplitZ(BoomZ+RSpanEnd*ActualSemiSpan_Fin-0.00075,CutterSize,SFin1)
        PFin1, PFin2 = UAT.BooleanSplitZ(BoomZ+RSpanEnd*ActualSemiSpan_Fin-0.00075,CutterSize,PFin1)
        
        # insert rudder control surface hinge spars
        if Rudder:
            xpos1=FinX+RootChord_TP/30.0+RootChord_Fin/4.0+(0.75-4.9*RChord/6.0)\
            *myChordFunctionFin(RSpanStart)*RootChord_Fin*math.cos(math.radians\
            (myTwistFunctionFin(RSpanStart)))
            xpos2=FinX+RootChord_TP/30.0+RootChord_Fin/4.0+(0.75-4.9*RChord/6.0)\
            *myChordFunctionFin(RSpanEnd)*RootChord_Fin*math.cos(math.radians\
            (myTwistFunctionFin(RSpanEnd)))
            SFin = UAT.CutCylinder(\
            xpos1\
            ,BoomY,BoomZ,\
            xpos2\
            ,BoomY,BoomZ+ActualSemiSpan_Fin,HingeSparODia,SFin)
            SRudder = UAT.CutCylinder(xpos1,BoomY,BoomZ,xpos2,BoomY,BoomZ+ActualSemiSpan_Fin,\
            HingeSparODia,SRudder)
            SFin2 = UAT.CutCylinder(xpos1,BoomY,BoomZ,xpos2,BoomY,BoomZ+ActualSemiSpan_Fin+0.001,\
            HingeSparODia,SFin2)
            SRudderSpar = UAT.CreateSpar(xpos1,BoomY,BoomZ,xpos2,BoomY,BoomZ+ActualSemiSpan_Fin+0.001,\
            HingeSparODia-SparClearance,HingeSparIDia)
            PFin = UAT.CutCylinder(xpos1,-BoomY,BoomZ,xpos2,-BoomY,BoomZ+ActualSemiSpan_Fin,\
            HingeSparODia,PFin)
            PRudder = UAT.CutCylinder(xpos1,-BoomY,BoomZ,xpos2,-BoomY,BoomZ+ActualSemiSpan_Fin,\
            HingeSparODia,PRudder)
            PFin2 = UAT.CutCylinder(xpos1,-BoomY,BoomZ,xpos2,-BoomY,BoomZ+ActualSemiSpan_Fin+0.001,\
            HingeSparODia,PFin2)
            PRudderSpar = UAT.CreateSpar(xpos1,-BoomY,BoomZ,xpos2,-BoomY,BoomZ+ActualSemiSpan_Fin+0.001,\
            HingeSparODia-SparClearance,HingeSparIDia)

        Tail = rs.BooleanUnion([Tail, SFin, PFin])
        Tail = UAT.CutCylinder(BoomLength,-ActualSemiSpan_TP,BoomZ, BoomLength,\
        ActualSemiSpan_TP,BoomZ,TailSparODia,Tail)
        Tail = UAT.CutCylinder(FinX+RootChord_TP/30.0+RootChord_Fin/4.0,BoomY,BoomZ,\
        FinX+RootChord_TP/30.0+RootChord_Fin/4.0,BoomY,BoomZ+ActualSemiSpan_Fin, FinSparODia,Tail)
        Tail = UAT.CutCylinder(FinX+RootChord_TP/30.0+RootChord_Fin/4.0,-BoomY,BoomZ,\
        FinX+RootChord_TP/30.0+RootChord_Fin/4.0,-BoomY,BoomZ+ActualSemiSpan_Fin, FinSparODia,Tail)
        TailSpar = UAT.CreateSpar(BoomLength,-ActualSemiSpan_TP,BoomZ, BoomLength,\
        ActualSemiSpan_TP,BoomZ, TailSparODia-SparClearance, TailSparIDia)

        Tail, TailS1 = UAT.BooleanSplitY(BoomY+0.05*Chord/1000.0,CutterSize,Tail)
        TailP1, Tail = UAT.BooleanSplitY(-BoomY-0.05*Chord/1000.0,CutterSize,Tail)
        Tail, TailS2 = UAT.BooleanSplitY(BoomY-0.05*Chord/1000.0,CutterSize,Tail)
        TailP2, Tail = UAT.BooleanSplitY(-BoomY+0.05*Chord/1000.0,CutterSize,Tail)
        Tail, TailS3 = UAT.BooleanSplitY(ESpanStart*ActualSemiSpan_TP+0.00175,CutterSize,Tail)
        TailP3, Tail = UAT.BooleanSplitY(-ESpanStart*ActualSemiSpan_TP-0.00175,CutterSize,Tail)
        TailS1, TailS4 = UAT.BooleanSplitY(ESpanEnd*ActualSemiSpan_TP-0.002,CutterSize,TailS1)
        TailP4, TailP1 = UAT.BooleanSplitY(-ESpanEnd*ActualSemiSpan_TP+0.002,CutterSize,TailP1)
        
        TailS2 = UAT.CutCylinder(0.25*Chord/1000.0,BoomY,BoomZ, BoomLength,BoomY,BoomZ,BoomODia,TailS2)
        TailP2 = UAT.CutCylinder(0.25*Chord/1000.0,-BoomY,BoomZ, BoomLength,-BoomY,BoomZ,BoomODia,TailP2)
        
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
            Tail = UAT.CutCylinder(xpos1,-ActualSemiSpan_TP,zpos1,xpos2,ActualSemiSpan_TP,zpos2,\
            HingeSparODia,Tail)
            TailS4 = UAT.CutCylinder(xpos1,0.0,zpos1,xpos2,ActualSemiSpan_TP,zpos2,\
            HingeSparODia,TailS4)
            SElevator = UAT.CutCylinder(xpos1,0.0,zpos1,xpos2,ActualSemiSpan_TP,zpos2,\
            HingeSparODia,SElevator)
            SElevatorSpar = UAT.CreateSpar(xpos1,0.0,zpos1,xpos2,ActualSemiSpan_TP,zpos2,\
            HingeSparODia-SparClearance,HingeSparIDia)
            TailP4 = UAT.CutCylinder(xpos1,0.0,zpos1,xpos2,-ActualSemiSpan_TP,zpos2,\
            HingeSparODia,TailP4)
            PElevator = UAT.CutCylinder(xpos1,0.0,zpos1,xpos2,-ActualSemiSpan_TP,zpos2,\
            HingeSparODia,PElevator)
            PElevatorSpar = UAT.CreateSpar(xpos1,0.0,zpos1,xpos2,-ActualSemiSpan_TP,zpos2,\
            HingeSparODia-SparClearance,HingeSparIDia)

if VTail:
    if CutStructure:
        AirFrame = rs.BooleanUnion([SPodSurf,TWing,MainEngineNacelle,PPodSurf])
    else:
        AirFrame= rs.BooleanUnion([SBoom,SPodSurf,PBoom,TWing,MainEngineNacelle,\
        PPodSurf, Tail, TailP, CompleteNoseGear, MainGearPort, MainGearStbd])
else:
    if CutStructure:
        AirFrame = rs.BooleanUnion([TWing,MainEngineNacelle])
    else:
        AirFrame= rs.BooleanUnion([SBoom,SPodSurf,PBoom,TWing,MainEngineNacelle,\
        PPodSurf, Tail, SFin, PFin, CompleteNoseGear, MainGearPort, MainGearStbd])

# =====  end of geometry construction ==========================================


# remove unwanted construction entities

rs.DeleteObjects(Sections)
rs.DeleteObjects(Sections_TP)
rs.DeleteObject(SLScopy)

if VTail == False:
    rs.DeleteObjects(Sections_Fin)

rs.DeleteObject(SLS)

