# Example Python script for building a wing with internal ribs and multiple control
# surfaces, to be run in the Rhinoceros 5.0 CAD engine.
# ==============================================================================
#
# This script uses the AirCONICS aircraft geometry toolbox (included):
# Aircraft CONfiguration through Integrated Cross-disciplinary Scripting 
# version 0.1.1b
# Andy Keane & Andras Sobester, 2017.
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
 
# Airframe parameters

# Name	=	Value	#	Long Name / Definition
Awing	=	8275000.0 # 8442291.0	#	total wing area, Decode1: 965830.1 (mm^2)
AR	=	9	#	aspect  ratio (span^2 / area)  Decode1: 9 [6,20]
y_tail_boom	=	423.7	#	horizontal position of tail booms (mm) Decode1: 239.2
TaperRatio = 0.7 #Decode1: 0.8 [0.6, 1]
SWEEP_QTR_deg = 1 # wing sweep in degrees
SettingAngle = 2.53 # as computed with XFLR5 to give desired Cl (e.g., 0.28 at 30 m/s)
foam_cutter_thickness_ratio = 0.8 # the thickness ratio of the cuter that creates the structural shell (0.8)
no_ribs = 9 # the number of ribs needed inside the wing structure (each side, typically 5 or 9 to give 2 or 4 control surfaces each side)
SparDia = 0.115 # main spar base diameter in m
SparVertPosn = 1.0/40.0 # vertical shift of spar as a fraction of root chord to ensure spar lies within wing (typically 1/40)
SparCentreLength = 0.08 # the semi-length of the parallel central spar to create the taper (as a fraction of semi-span, typically 0.25, at least less than 0.9)
SparOval = 1.025 # vertical scaling factor to generate oval Xsection main spar (typically 1.1 to resist bending)
TaperSpar = 1 # turn on the taper on the main spar if required (1=on, 0=off)
SparConeLength = 2.0157 # the length of the cone added to the parallel central spar to create the taper (as a fraction of semi-span, at least 1.1)
HingeSparDia = 0.02 # hinge spar diameter in m (typically around 1/12 times main spar diam)
rib_thick = 0.006 # rib thickness in m (0.006)
flap_cap_thick = 0.004 # thickness of end caps on flap foam parts in m (0.004)
Cstrut_on = 1 # turn on the central reinforcing strut if required (1=on, 0=off)
TEspar_on = 1 # turn on the trailing edge reinforcing spar if required (1=on, 0=off)
TESparDia = 0.01 # trailing edge spar diameter in m (typically around 1/50 times main spar diam)

# Derived parameters
RibPosn = [] # array of rib positions - if these need to be manually fixed alter loop below that positions them
RibPosn.append(0.0) # rib#0 is at the centre of the aircraft and not use
RibPosn.append(y_tail_boom/1000.0) # rib#1 is the first rib along the wing set inline with the tail boom (foam inside of this is normally not used)
SWEEP_QTR_rad = math.radians(SWEEP_QTR_deg) # and in radians
SWEEP_LE_rad = math.atan(math.tan(SWEEP_QTR_rad)+\
(1/AR)*(1-TaperRatio)/(1+TaperRatio))
SWEEP_LE_deg = math.degrees(SWEEP_LE_rad) # resulting leading edge sweep in degrees
Span = math.sqrt(AR*Awing)
Chord = Awing/Span

# ==== 3D wing main wing definition ============================================
# ==== through definition of spanwise parameter variations =====================

def myDihedralFunction(Epsilon):
    # User-defined function describing the variation of dihedral as a function
    # of the leading edge coordinate
    return 0

def myTwistFunction(Epsilon):
    # User-defined function describing the variation of twist as a function
    # of the leading edge coordinate
    RootTwist = 0
    TipTwist = 2 # typical reduction in twist to give washout
    TipTwist = 0
    return RootTwist + Epsilon*(TipTwist-RootTwist)

def myChordFunction(Epsilon):
    # User-defined function describing the variation of chord as a function of 
    # the leading edge coordinate
    
    if Epsilon < 0.1:
        return 1
    else:
        ChordLengths = [1, TaperRatio]
        EpsArray = [0.1, 1]
        f = act.linear_interpolation(EpsArray, ChordLengths)
        return f(Epsilon)

def myAirfoilFunction(Epsilon, LEPoint, ChordFunct, ChordFactor, \
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
    
def mySweepAngleFunction(Epsilon):
    # User-defined function describing the variation of sweep angle as a
    # function of the leading edge coordinate
    return SWEEP_LE_deg

#===============================================================================
# AIRFRAME GEOMETRY GENERATION
#===============================================================================

rs.EnableRedraw(False)

# Wing apex location
P = (0,0,0)

LooseSurf = 3 # Tightly fit surface (loft_type argument in AddLoftSrf)
SegmentNo = 21 # Number of airfoil sections to loft wing surface over


TipRequired = 1 # Flat wingtip

SectionsRequired = False # when optimizing this currently needs to be False but
                         # when creating XFLR5 files it must be True

Wing = liftingsurface.LiftingSurface(P, mySweepAngleFunction,\
myDihedralFunction, myTwistFunction, myChordFunction,\
myAirfoilFunction, LooseSurf, SegmentNo, TipRequired, SectionsRequired)

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

OWing = liftingsurface.LiftingSurface(P, mySweepAngleFunction,\
myDihedralFunction, myTwistFunction, myChordFunction,\
myAirfoilFunction, LooseSurf, SegmentNo, TipRequired, SectionsRequired)
# The wing thus defined, we now scale it to the target aspect ratio and area
OWing.TargetAspectRatio = AR/foam_cutter_thickness_ratio
OWing.TargetArea = foam_cutter_thickness_ratio*Awing/1000000.0
OWing.wTargetAspectRatio = 1
OWing.wTargetArea = 1

# Initial iterate
ChordFactor = 0.26
ScaleFactor = 1.47

OptimizeChordScale=1

OSLS, OActualSemiSpan, OLSP_area,  ORootChord, OAR, OSWingTip, OSections =\
OWing.GenerateLiftingSurface(ChordFactor, ScaleFactor, OptimizeChordScale)

# Rotate the wing to the specified setting angle
TipX = math.tan( SWEEP_LE_rad ) * ActualSemiSpan
RotVec = rs.VectorCreate((0,0,0),(0,1,0))
SLS = rs.RotateObject(SLS, (0,0,0), -SettingAngle, axis = RotVec)
SWingTip = rs.RotateObject(SWingTip, (0,0,0), -SettingAngle, axis = RotVec)
# shift wing up to allow for setting angle about LE and 
# not 1/4 chord point where we attach it
SLS = rs.MoveObject(SLS,\
(0.0, 0.0, 0.25*RootChord*math.sin(math.radians(SettingAngle))))
SWingTip = rs.MoveObject(SWingTip,\
(0.0, 0.0, 0.25*RootChord*math.sin(math.radians(SettingAngle))))
PWingTip = act.MirrorObjectXZ(SWingTip)

OSLS = rs.RotateObject(OSLS, (0,0,0), -SettingAngle, axis = RotVec)
OSWingTip = rs.RotateObject(OSWingTip, (0,0,0), -SettingAngle, axis = RotVec)
# shift wing up to allow for setting angle about LE and 
# not 1/4 chord point where we attach it
OSLS = rs.MoveObject(OSLS,\
(((1-foam_cutter_thickness_ratio)/10.0)*RootChord, 0.0, 0.25*RootChord*math.sin(math.radians(SettingAngle))))
OSWingTip = rs.MoveObject(OSWingTip,\
(((1-foam_cutter_thickness_ratio)/10.0)*RootChord, 0.0, 0.25*RootChord*math.sin(math.radians(SettingAngle))))
OPWingTip = act.MirrorObjectXZ(OSWingTip)

# =====  Assembly =============================================================

PLS = act.MirrorObjectXZ(SLS)
OPLS = act.MirrorObjectXZ(OSLS)
TWing = rs.JoinSurfaces([SLS,PLS,SWingTip, PWingTip])
OWing = rs.JoinSurfaces([OSLS,OPLS,OSWingTip, OPWingTip])
cutter = rs.AddBox(((0.75*RootChord,1.25*ActualSemiSpan,-100),(0.75*RootChord,1.25*ActualSemiSpan,100),(1.75*RootChord,1.25*ActualSemiSpan,100),(1.75*RootChord,1.25*ActualSemiSpan,-100),\
    (0.75*RootChord,-1.25*ActualSemiSpan,-100),(0.75*RootChord,-1.25*ActualSemiSpan,100),(1.75*RootChord,-1.25*ActualSemiSpan,100),(1.75*RootChord,-1.25*ActualSemiSpan,-100)))
OWing = rs.BooleanDifference(OWing, cutter, delete_input=True)
# remove unwanted construction entities
rs.DeleteObjects([SLS,PLS,SWingTip, PWingTip])
rs.DeleteObjects([OSLS,OPLS,OSWingTip, OPWingTip])

#===== Spar Creation ===========================================================

## create main spar from geometry primitives, main spar may be tapered and oval
#Cspar = rs.AddCylinder([RootChord/4.0,SparCentreLength*ActualSemiSpan,ChordFactor*RootChord*SparVertPosn],\
# [RootChord/4.0,-SparCentreLength*ActualSemiSpan,ChordFactor*RootChord*SparVertPosn], SparDia/2)
#if TaperSpar == 1:
#    Scone = rs.AddCone([0,0,0],SparConeLength*ActualSemiSpan,SparDia/2)
#else:
#    Scone = rs.AddCylinder([0,0,0],SparConeLength*ActualSemiSpan,SparDia/2)
#RotVec2 = rs.VectorCreate((0,0,0),(1,0,0))
#Scone = rs.RotateObject(Scone, (0,0,0), -90.0, axis = RotVec2)
#Scone = rs.MoveObject(Scone,(RootChord/4.0,(SparConeLength+SparCentreLength)*ActualSemiSpan,ChordFactor*RootChord*SparVertPosn))
#cutter = rs.AddBox(((-0.75*RootChord,1.01*ActualSemiSpan,-100),(-0.75*RootChord,1.01*ActualSemiSpan,100),(1.75*RootChord,1.01*ActualSemiSpan,100),(1.75*RootChord,1.01*ActualSemiSpan,-100),\
#    (-0.75*RootChord,3.25*ActualSemiSpan,-100),(-0.75*RootChord,3.25*ActualSemiSpan,100),(1.75*RootChord,3.25*ActualSemiSpan,100),(1.75*RootChord,3.25*ActualSemiSpan,-100)))
#Scone = rs.BooleanDifference(Scone, cutter, delete_input=True)
#Pcone = act.MirrorObjectXZ(Scone)
#Spar = rs.BooleanUnion([Cspar, Scone, Pcone]) # the main wing spar
#Spar = rs.ScaleObject( Spar, [RootChord/4.0,0,ChordFactor*RootChord*SparVertPosn], (1,1,SparOval)) # make spar oval in shape

# create main spar from measured data
EC=[];EX=[];EZ=[]
EC.append(rs.AddPoint(	0.0000	,	0.0000	,	0.0000	));	EX.append(rs.AddPoint(	0.0601	,	0.0000	,	0.0000	));	EZ.append(rs.AddPoint(	0.0000	,	0.0000	,	0.0591	))
EC.append(rs.AddPoint(	0.0000	,	0.3500	,	0.0000	));	EX.append(rs.AddPoint(	0.0601	,	0.3500	,	0.0000	));	EZ.append(rs.AddPoint(	0.0000	,	0.3500	,	0.0591	))
EC.append(rs.AddPoint(	0.0000	,	0.5000	,	0.0000	));	EX.append(rs.AddPoint(	0.0601	,	0.5000	,	0.0000	));	EZ.append(rs.AddPoint(	0.0000	,	0.5000	,	0.0591	))
EC.append(rs.AddPoint(	0.0000	,	0.6500	,	0.0000	));	EX.append(rs.AddPoint(	0.0590	,	0.6500	,	0.0000	));	EZ.append(rs.AddPoint(	0.0000	,	0.6500	,	0.0579	))
EC.append(rs.AddPoint(	0.0000	,	0.9500	,	0.0000	));	EX.append(rs.AddPoint(	0.0569	,	0.9500	,	0.0000	));	EZ.append(rs.AddPoint(	0.0000	,	0.9500	,	0.0557	))
EC.append(rs.AddPoint(	0.0000	,	1.2500	,	0.0000	));	EX.append(rs.AddPoint(	0.0548	,	1.2500	,	0.0000	));	EZ.append(rs.AddPoint(	0.0000	,	1.2500	,	0.0538	))
EC.append(rs.AddPoint(	0.0000	,	1.5500	,	0.0000	));	EX.append(rs.AddPoint(	0.0523	,	1.5500	,	0.0000	));	EZ.append(rs.AddPoint(	0.0000	,	1.5500	,	0.0514	))
EC.append(rs.AddPoint(	0.0000	,	1.8500	,	0.0000	));	EX.append(rs.AddPoint(	0.0503	,	1.8500	,	0.0000	));	EZ.append(rs.AddPoint(	0.0000	,	1.8500	,	0.0495	))
EC.append(rs.AddPoint(	0.0000	,	2.1500	,	0.0000	));	EX.append(rs.AddPoint(	0.0483	,	2.1500	,	0.0000	));	EZ.append(rs.AddPoint(	0.0000	,	2.1500	,	0.0473	))
EC.append(rs.AddPoint(	0.0000	,	2.4500	,	0.0000	));	EX.append(rs.AddPoint(	0.0458	,	2.4500	,	0.0000	));	EZ.append(rs.AddPoint(	0.0000	,	2.4500	,	0.0450	))
EC.append(rs.AddPoint(	0.0000	,	2.7500	,	0.0000	));	EX.append(rs.AddPoint(	0.0438	,	2.7500	,	0.0000	));	EZ.append(rs.AddPoint(	0.0000	,	2.7500	,	0.0428	))
EC.append(rs.AddPoint(	0.0000	,	3.0500	,	0.0000	));	EX.append(rs.AddPoint(	0.0413	,	3.0500	,	0.0000	));	EZ.append(rs.AddPoint(	0.0000	,	3.0500	,	0.0407	))
EC.append(rs.AddPoint(	0.0000	,	3.3500	,	0.0000	));	EX.append(rs.AddPoint(	0.0393	,	3.3500	,	0.0000	));	EZ.append(rs.AddPoint(	0.0000	,	3.3500	,	0.0387	))
EC.append(rs.AddPoint(	0.0000	,	3.6500	,	0.0000	));	EX.append(rs.AddPoint(	0.0368	,	3.6500	,	0.0000	));	EZ.append(rs.AddPoint(	0.0000	,	3.6500	,	0.0364	))
EC.append(rs.AddPoint(	0.0000	,	3.9500	,	0.0000	));	EX.append(rs.AddPoint(	0.0348	,	3.9500	,	0.0000	));	EZ.append(rs.AddPoint(	0.0000	,	3.9500	,	0.0346	))
EC.append(rs.AddPoint(	0.0000	,	4.2500	,	0.0000	));	EX.append(rs.AddPoint(	0.0323	,	4.2500	,	0.0000	));	EZ.append(rs.AddPoint(	0.0000	,	4.2500	,	0.0323	))
EC.append(rs.AddPoint(	0.0000	,	4.3500	,	0.0000	));	EX.append(rs.AddPoint(	0.0318	,	4.3500	,	0.0000	));	EZ.append(rs.AddPoint(	0.0000	,	4.3500	,	0.0318	))
SparDia = 2.0*0.0601 # needed for some of the foam cutting processes, based on largest X dimension

EP=[]
for i in range(0, 17):
    EP.append(rs.AddEllipse3Pt(EC[i],EX[i],EZ[i]))

Scone = rs.AddLoftSrf(EP)
rs.DeleteObjects(EC)
rs.DeleteObjects(EX)
rs.DeleteObjects(EZ)
rs.DeleteObjects(EP)
Pcone = act.MirrorObjectXZ(Scone)
# the main wing spar
Spar = rs.MoveObject(rs.BooleanUnion([Scone, Pcone]),(RootChord/4.0,0.0,ChordFactor*RootChord*SparVertPosn))



# create secondary cutter around main spar
Ospar = rs.ScaleObject( Spar, [RootChord/4.0,0,ChordFactor*RootChord*SparVertPosn], (1.5,1,4), copy=True ) # outer surface around spar to define foam and rib spar wrapper shape
# create hinge spar
Tspar = rs.AddCylinder([0.75*RootChord,1.01*ActualSemiSpan,-ChordFactor*RootChord/12.0], [0.75*RootChord,-1.01*ActualSemiSpan,-ChordFactor*RootChord/12.0], HingeSparDia/2) # hinge spar
# create secondary cutter around hinge spar
Otspar = rs.ScaleObject( Tspar, [0.75*RootChord,0,-ChordFactor*RootChord/12.0], (8,1,6), copy=True ) # outer surface around hinge spar to define foam and rib spar wrapper shape

# Htspar is used to define the cutter to create the separation between wing foam blocks and control surface blocks
Htspar = rs.ScaleObject( Tspar, [0.75*RootChord,0,-ChordFactor*RootChord/12.0], (5.5,1,5.5), copy=True )
cutter = rs.ScaleObject( Tspar, [0.75*RootChord,0,-ChordFactor*RootChord/12.0], (5,1,5), copy=True )
Htspar = rs.BooleanDifference(Htspar, cutter, delete_input=True)
cutter = rs.AddBox(((0.75*RootChord,1.25*ActualSemiSpan,-100),(0.75*RootChord,1.25*ActualSemiSpan,100),(1.75*RootChord,1.25*ActualSemiSpan,100),(1.75*RootChord,1.25*ActualSemiSpan,-100),\
    (0.75*RootChord,-1.25*ActualSemiSpan,-100),(0.75*RootChord,-1.25*ActualSemiSpan,100),(1.75*RootChord,-1.25*ActualSemiSpan,100),(1.75*RootChord,-1.25*ActualSemiSpan,-100)))
Htspar = rs.BooleanDifference(Htspar, cutter, delete_input=True)

# Ktspar is used to define the cutter to create the servo mounting areas - the scale sets how big these are
Ktspar = rs.ScaleObject( Tspar, [0.75*RootChord,0,-ChordFactor*RootChord/12.0], (18,1,30), copy=True )
cutter = rs.AddBox(((0.75*RootChord,1.25*ActualSemiSpan,-100),(0.75*RootChord,1.25*ActualSemiSpan,100),(1.75*RootChord,1.25*ActualSemiSpan,100),(1.75*RootChord,1.25*ActualSemiSpan,-100),\
    (0.75*RootChord,-1.25*ActualSemiSpan,-100),(0.75*RootChord,-1.25*ActualSemiSpan,100),(1.75*RootChord,-1.25*ActualSemiSpan,100),(1.75*RootChord,-1.25*ActualSemiSpan,-100)))
Ktspar = rs.BooleanDifference(Ktspar, cutter, delete_input=True)
cutter = rs.CopyObject(Tspar)
Ktspar = rs.BooleanDifference(Ktspar, cutter, delete_input=True)

# Rtspar is used to define the cutter to create the sepration between ribs and control surfaces ribs
Rtspar = rs.ScaleObject( Tspar, [0.75*RootChord,0,-ChordFactor*RootChord/12.0], (5.5,1,5.5), copy=True )
cutter = rs.ScaleObject( Tspar, [0.75*RootChord,0,-ChordFactor*RootChord/12.0], (5,1,5), copy=True )
Rtspar = rs.BooleanDifference(Rtspar, cutter, delete_input=True)
cutter = rs.AddBox(((0.75*RootChord,1.25*ActualSemiSpan,-100),(0.75*RootChord,1.25*ActualSemiSpan,100),(-1.75*RootChord,1.25*ActualSemiSpan,100),(-1.75*RootChord,1.25*ActualSemiSpan,-100),\
    (0.75*RootChord,-1.25*ActualSemiSpan,-100),(0.75*RootChord,-1.25*ActualSemiSpan,100),(-1.75*RootChord,-1.25*ActualSemiSpan,100),(-1.75*RootChord,-1.25*ActualSemiSpan,-100)))
Rtspar = rs.BooleanDifference(Rtspar, cutter, delete_input=True)

cutter = rs.CopyObject(TWing)
Ospar = rs.BooleanIntersection(Ospar, cutter, delete_input=True)

cutter = rs.CopyObject(TWing)
Otspar = rs.BooleanIntersection(Otspar, cutter, delete_input=True)

# now cut out centre of wing
cutter = rs.BooleanDifference(TWing, OWing, delete_input=False)
cutter2 = rs.BooleanUnion([cutter, Ospar])
FWing = rs.BooleanUnion([cutter2, Otspar])
cutter = rs.CopyObject(Spar)
FWing = rs.BooleanDifference(FWing, cutter, delete_input=True)
cutter = rs.CopyObject(Tspar)
FWing = rs.BooleanDifference(FWing, cutter, delete_input=True)

# create trailing edge reinforcement spar if wanted
if TEspar_on == 1:
    STEspar = rs.AddCylinder([0.9*RootChord,0.001,-ChordFactor*RootChord/10.0], [0.8*RootChord,1.01*ActualSemiSpan,-ChordFactor*RootChord/10.0], TESparDia/2)
    PTEspar = act.MirrorObjectXZ(STEspar)

# ===== create ribs ============================================================


for i in range(2, no_ribs): # note we don't actually create the last rib in this loop
    RibPosn.append(RibPosn[1]+(i-1)*(ActualSemiSpan-RibPosn[1])/(no_ribs-1))
RibPosn.append(ActualSemiSpan) # create the tip rib ate the wing end

# first add additional stiffener behind main spar between top and bottom surfaces
if Cstrut_on == 1:
    Mspar = rs.AddCylinder([RootChord/4.0,1.01*ActualSemiSpan,ChordFactor*RootChord*SparVertPosn],\
     [RootChord/4.0,-1.01*ActualSemiSpan,ChordFactor*RootChord*SparVertPosn], SparDia/2)
    OOspar = rs.ScaleObject( Mspar, [RootChord/4.0,0,ChordFactor*RootChord*SparVertPosn], (4.5,1,8.0), copy=True )
    cutter = rs.ScaleObject( Mspar, [RootChord/4.0,0,ChordFactor*RootChord*SparVertPosn], (4.3,1,7.7), copy=True )
    rs.DeleteObject(Mspar)
    OOspar = rs.BooleanDifference(OOspar, cutter, delete_input=True)
    cutter = rs.AddBox(((-0.75*RootChord,3.25*ActualSemiSpan,-100),(-0.75*RootChord,3.25*ActualSemiSpan,100),(RootChord/4.0,3.25*ActualSemiSpan,100),(RootChord/4.0,3.25*ActualSemiSpan,-100),\
        (-0.75*RootChord,-3.25*ActualSemiSpan,-100),(-0.75*RootChord,-3.25*ActualSemiSpan,100),(RootChord/4.0,-3.25*ActualSemiSpan,100),(RootChord/4.0,-3.25*ActualSemiSpan,-100)))
    OOspar = rs.BooleanDifference(OOspar, cutter, delete_input=True)
    cutter = rs.CopyObject(TWing)
    OOspar = rs.BooleanIntersection(OOspar, cutter, delete_input=True)
    FWing = rs.BooleanUnion([FWing, OOspar], delete_input=True)
    rs.DeleteObjects(Mspar)
    rs.DeleteObjects(OOspar)

Scutrib = [] # starboard ribs
Pcutrib = []# port ribs
Xcutrib = [] # starboard rib ends
Ycutrib = []# port rib ends
for i in range(1, no_ribs+1):
    Srib = rs.AddBox(((-100,RibPosn[i],-100),(-100,RibPosn[i]-rib_thick,-100),(100,RibPosn[i]-rib_thick,-100),(100,RibPosn[i],-100),\
    (-100,RibPosn[i],100),(-100,RibPosn[i]-rib_thick,100),(100,RibPosn[i]-rib_thick,100),(100,RibPosn[i],100)))
    cutter = rs.CopyObject(FWing)
    cutter2 = rs.BooleanIntersection(Srib, cutter)
    # now cut rear part of rib off to add to control surface
    if i%2 == 0:
        cutter = rs.CopyObject(Rtspar)
        cutter2 = rs.BooleanDifference(cutter2, cutter, delete_input=True)
        if TEspar_on == 1:
            cutter = rs.CopyObject(STEspar)
            cutter2 = rs.BooleanDifference(cutter2, cutter, delete_input=True)
        Xcutrib.append(rs.CopyObject(cutter2[1])) # starboard central spacers in flaps
        Ycutrib.append(act.MirrorObjectXZ(cutter2[1])) # port central spacers in flaps
        # now addservo mounting area
        Srib = rs.AddBox(((-100,RibPosn[i],-100),(-100,RibPosn[i]-rib_thick,-100),(100,RibPosn[i]-rib_thick,-100),(100,RibPosn[i],-100),\
        (-100,RibPosn[i],100),(-100,RibPosn[i]-rib_thick,100),(100,RibPosn[i]-rib_thick,100),(100,RibPosn[i],100)))
        cutter = rs.CopyObject(Ktspar)
        Srib = rs.BooleanIntersection(Srib, cutter)
        cutter = rs.CopyObject(TWing)
        Srib = rs.BooleanIntersection(Srib, cutter)
        cutter2[0] = rs.BooleanUnion([cutter2[0], Srib])
        rs.DeleteObjects([cutter, Srib])
    Scutrib.append(rs.CopyObject(cutter2[0]))
    Pcutrib.append(act.MirrorObjectXZ(cutter2[0]))
    rs.DeleteObjects(cutter2)


# add trailing edge spar to reinforce flaps if desired

if TEspar_on == 1:
    cutter = rs.CopyObject(STEspar)
    FWing = rs.BooleanDifference(FWing, cutter)
    cutter = rs.CopyObject(PTEspar)
    FWing = rs.BooleanDifference(FWing, cutter)
Tcutrib = [] # starboard outer flap ends
Rcutrib = [] # port outer flap ends

for i in range(1, no_ribs):
    Trib = rs.AddBox(((-100,RibPosn[i]+flap_cap_thick,-100),(-100,RibPosn[i],-100),(100,RibPosn[i],-100),(100,RibPosn[i]+flap_cap_thick,-100),\
    (-100,RibPosn[i]+flap_cap_thick,100),(-100,RibPosn[i],100),(100,RibPosn[i],100),(100,RibPosn[i]+flap_cap_thick,100)))
    if TEspar_on == 1:
        cutter = rs.CopyObject(STEspar)
        Trib = rs.BooleanDifference(Trib, cutter, delete_input=True)
    cutter = rs.CopyObject(FWing)
    cutter2 = rs.BooleanIntersection(Trib, cutter)
    # now cut rear part of rib off to add to control surface
    cutter = rs.CopyObject(Htspar)
    cutter2 = rs.BooleanDifference(cutter2, cutter, delete_input=True)
    if TEspar_on == 1:
        rs.DeleteObjects([cutter2[1]])
        Tcutrib.append(rs.CopyObject(cutter2[0]))
        Rcutrib.append(act.MirrorObjectXZ(cutter2[0]))
    else:
        rs.DeleteObjects([cutter2[0]])
        Tcutrib.append(rs.CopyObject(cutter2[1]))
        Rcutrib.append(act.MirrorObjectXZ(cutter2[1]))
    rs.DeleteObjects(cutter2)

Ocutrib = [] # starboard inner flap ends
Qcutrib = [] # port inner flap ends
for i in range(2, no_ribs+1):
    Orib = rs.AddBox(((-100,RibPosn[i]-rib_thick,-100),(-100,RibPosn[i]-rib_thick-flap_cap_thick,-100),(100,RibPosn[i]-rib_thick-flap_cap_thick,-100),(100,RibPosn[i]-rib_thick,-100),\
    (-100,RibPosn[i]-rib_thick,100),(-100,RibPosn[i]-rib_thick-flap_cap_thick,100),(100,RibPosn[i]-rib_thick-flap_cap_thick,100),(100,RibPosn[i]-rib_thick,100)))
    if TEspar_on == 1:
        cutter = rs.CopyObject(STEspar)
        Orib = rs.BooleanDifference(Orib, cutter, delete_input=True)
    cutter = rs.CopyObject(FWing)
    cutter2 = rs.BooleanIntersection(Orib, cutter)
    # now cut rear part of rib off to add to control surface
    cutter = rs.CopyObject(Htspar)
    cutter2 = rs.BooleanDifference(cutter2, cutter, delete_input=True)
    if TEspar_on == 1:
        rs.DeleteObjects([cutter2[1]])
        Ocutrib.append(rs.CopyObject(cutter2[0]))
        Qcutrib.append(act.MirrorObjectXZ(cutter2[0]))
    else:
        rs.DeleteObjects([cutter2[0]])
        Ocutrib.append(rs.CopyObject(cutter2[1]))
        Qcutrib.append(act.MirrorObjectXZ(cutter2[1]))
    rs.DeleteObjects(cutter2)

rs.DeleteObjects([Rtspar])
FWing = rs.BooleanDifference(FWing, Htspar, delete_input=True)
FFWing = rs.CopyObject(FWing[0]) # will become wing front foam blocks
RFWing = rs.CopyObject(FWing[1]) # will become control surface foam blocks

# ===== chop the foam up into 32 parts =========================================

for i in range(1, no_ribs+1):
    Srib = rs.AddBox(((-100,RibPosn[i],-100),(-100,RibPosn[i]-rib_thick,-100),(100,RibPosn[i]-rib_thick,-100),(100,RibPosn[i],-100),\
    (-100,RibPosn[i],100),(-100,RibPosn[i]-rib_thick,100),(100,RibPosn[i]-rib_thick,100),(100,RibPosn[i],100)))
    Prib = act.MirrorObjectXZ(Srib)
    FFWing = rs.BooleanDifference(FFWing, Srib, delete_input=True)
    FFWing = rs.BooleanDifference(FFWing, Prib, delete_input=True)

for i in range(1, no_ribs+1):
    Srib = rs.AddBox(((-100,RibPosn[i],-100),(-100,RibPosn[i]-rib_thick,-100),(100,RibPosn[i]-rib_thick,-100),(100,RibPosn[i],-100),\
    (-100,RibPosn[i],100),(-100,RibPosn[i]-rib_thick,100),(100,RibPosn[i]-rib_thick,100),(100,RibPosn[i],100)))
    if i<no_ribs+1:
        cutter = rs.AddBox(((-100,RibPosn[i]+flap_cap_thick,-100),(-100,RibPosn[i],-100),(100,RibPosn[i],-100),(100,RibPosn[i]+flap_cap_thick,-100),\
        (-100,RibPosn[i]+flap_cap_thick,100),(-100,RibPosn[i],100),(100,RibPosn[i],100),(100,RibPosn[i]+flap_cap_thick,100)))
        Srib = rs.BooleanUnion([Srib, cutter])
    if i>1:
        cutter = rs.AddBox(((-100,RibPosn[i]-rib_thick,-100),(-100,RibPosn[i]-rib_thick-flap_cap_thick,-100),(100,RibPosn[i]-rib_thick-flap_cap_thick,-100),(100,RibPosn[i]-rib_thick,-100),\
        (-100,RibPosn[i]-rib_thick,100),(-100,RibPosn[i]-rib_thick-flap_cap_thick,100),(100,RibPosn[i]-rib_thick-flap_cap_thick,100),(100,RibPosn[i]-rib_thick,100)))
        Srib = rs.BooleanUnion([Srib, cutter])
    Prib = act.MirrorObjectXZ(Srib)
    RFWing = rs.BooleanDifference(RFWing, Srib, delete_input=True)
    RFWing = rs.BooleanDifference(RFWing, Prib, delete_input=True)
    rs.DeleteObjects(cutter)
    rs.DeleteObjects(cutter2)

# =====  end of foam and rib construction ==============================================

# delete unwanted objects
rs.DeleteObjects(FWing)
rs.DeleteObjects(Ospar)
rs.DeleteObjects(Otspar)
rs.DeleteObjects(Htspar)
rs.DeleteObjects(Ktspar)
rs.DeleteObjects(Rtspar)
rs.DeleteObjects(Orib)
rs.DeleteObjects(Prib)
rs.DeleteObjects(Srib)
rs.DeleteObjects(Trib)
rs.DeleteObjects(Scone)
rs.DeleteObjects(Pcone)
rs.DeleteObjects(cutter)
rs.DeleteObjects(cutter2)
rs.EnableRedraw(True)

# decide which objects to delete from final design
rs.DeleteObjects(TWing) # copy of outer mould surface
rs.DeleteObjects(OWing) # copy of inner surface for generating holes in foam
#rs.DeleteObjects(FFWing[0]) # central front foam part
#rs.DeleteObjects(FFWing[no_ribs-1]) # central front foam part
#rs.DeleteObjects(RFWing[no_ribs-1]) # central rear foam part
#rs.DeleteObjects(FFWing) # front foam parts
#rs.DeleteObjects(RFWing) # rear foam parts
#rs.DeleteObjects(Spar) # main spar
#rs.DeleteObjects(Tspar) # flap hing spar
#rs.DeleteObjects(STEspar) # starboard trailing edge spar
#rs.DeleteObjects(PTEspar) # port trainiling edge spar
#rs.DeleteObjects(Tcutrib) # starboard outer flap ends
#rs.DeleteObjects(Rcutrib) # port outer flap ends
#rs.DeleteObjects(Ocutrib)# starboard inner flap ends
#rs.DeleteObjects(Qcutrib)# port inner flap ends
#rs.DeleteObjects(Scutrib) # starboard ribs
#rs.DeleteObjects(Pcutrib) # port ribs
#rs.DeleteObjects(Xcutrib) # starboard central spacers in flaps
#rs.DeleteObjects(Ycutrib) # port central spacers in flaps

# =====  end of geometry construction ==========================================