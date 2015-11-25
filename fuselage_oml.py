# A function for generating a parametric fuselage external surface (outer mould
# line) model. For a description of the parameterisation, see the article:
# A. Sobester, "'Self-designing' Parametric Geometries", AIAA SciTech 2015,
# Orlando, FL.
# ==============================================================================
# AirCONICS
# Aircraft CONfiguration through Integrated Cross-disciplinary Scripting 
# version 0.2
# Andras Sobester, 2015.
# Bug reports to a.sobester@soton.ac.uk or @ASobester please.
# ==============================================================================

from __future__ import division

import rhinoscriptsyntax as rs, AirCONICStools as act, primitives as prim, itertools

import Rhino

import airconics_setup


def _AirlinerFuselagePlanView(NoseLengthRatio, TailLengthRatio):
# Internal function. Defines the control
# polygons of the fuselage in side view

    kN = NoseLengthRatio/0.182
    tN = TailLengthRatio/0.293

    PlanPort = [
    [0,       0,     0],
    [0*kN,       -0.1,   0],
    [0.332*kN,   -0.395, 0],
    [1.250*kN,   -0.810, 0],
    [2.517*kN,   -1.074, 0],
    [4*kN ,      -1.15,  0],
    [4*kN ,      -1.15,  0],
    # Parallel sided section here
    [22-(22-15.55)*tN,  -1.15,  0],
    [22-(22-15.55)*tN,  -1.15,  0],
    [22-(22-16.428)*tN, -1.126, 0],
    [22-(22-20.3362)*tN,-0.483,0],
    [22,     -0.0987,0]]
    
    for i in range(len(PlanPort)):
        PlanPort[i][0] = PlanPort[i][0]*2.541
        PlanPort[i][1] = PlanPort[i][1]*2.541
        PlanPort[i][2] = PlanPort[i][2]*2.541

    NoseEndX = 4*kN*2.541
    TailStartX = (22-(22-15.55)*tN)*2.541

    return PlanPort, NoseEndX, TailStartX


def _AirlinerFuselageSideView(NoseLengthRatio, TailLengthRatio):
# Internal function. Defines the control
# polygons of the fuselage in side view

    kN = NoseLengthRatio/0.182
    tN = TailLengthRatio/0.293

    # The upper contour control points
    # of the fuselage in side view
    AFSVUpper = [
    [0,     0, 0],
    [0,     0, 0.3],
    [1.395*kN, 0, 1.547],
    [4*kN,     0, 1.686],
    [4*kN,     0, 1.686],
    # parallel section here
    [22-(22-15.55)*tN, 0, 1.686],
    [22-(22-15.55)*tN, 0, 1.686],
    [22-(22-19.195)*tN,0, 1.549],
    [22    ,0, 0.904]]
    
    for i in range(len(AFSVUpper)):
        AFSVUpper[i][0] = AFSVUpper[i][0]*2.541
        AFSVUpper[i][1] = AFSVUpper[i][1]*2.541
        AFSVUpper[i][2] = AFSVUpper[i][2]*2.541
    

    # The lower contour control points
    # of the fuselage in side view
    AFSVLower = [
    [0,     0, 0],
    [0,     0, -0.3],
    [0.947*kN, 0, -0.517],
    [4*kN,     0, -0.654],
    [4*kN,     0, -0.654],
    # Parallel sides section
    [22-(22-15.55)*tN, 0, -0.654],
    [22-(22-15.55)*tN, 0, -0.654],
    # Tailstrike slope section
    [22-(22-18.787)*tN,0, -0.256],
    [22     ,0, 0.694]]
    
    for i in range(len(AFSVLower)):
        AFSVLower[i][0] = AFSVLower[i][0]*2.541
        AFSVLower[i][1] = AFSVLower[i][1]*2.541
        AFSVLower[i][2] = AFSVLower[i][2]*2.541

    return AFSVUpper, AFSVLower

def _FuselageLongitudinalGuideCurves(NoseLengthRatio, TailLengthRatio):
# Internal function. Defines the four longitudinal curves that outline the 
# fuselage (outer mould line). 

    FSVU, FSVL = _AirlinerFuselageSideView(NoseLengthRatio, TailLengthRatio)
    FSVUCurve = rs.AddCurve(FSVU)
    FSVLCurve = rs.AddCurve(FSVL)
    
    AFPVPort, NoseEndX, TailStartX = _AirlinerFuselagePlanView(NoseLengthRatio, TailLengthRatio)
    
    # Generate plan view
    PlanPortCurve      = rs.AddCurve(AFPVPort)
    
    # How wide is the fuselage?
    (Xmin,Ymin,Zmin,Xmax,Ymax,Zmax) = act.ObjectsExtents(PlanPortCurve)
    
    # Generate a slightly wider projection surface
    FSVMeanCurve = rs.MeanCurve(FSVUCurve, FSVLCurve)
    RuleLinePort      = rs.AddLine((0,0,0),(0,-1.1*abs(Ymax-Ymin),0))
    FSVMCEP = rs.CurveEndPoint(FSVMeanCurve)
    AftLoftEdgePort      = rs.CopyObject(RuleLinePort,     FSVMCEP)
    ParallelLoftEdgePort      = rs.CopyObject(FSVMeanCurve,(0,-1.1*abs(Ymax-Ymin),0))
    LSPort      = rs.AddSweep2((FSVMeanCurve,ParallelLoftEdgePort     ),(RuleLinePort,     AftLoftEdgePort     ))

    # Project the plan view onto the mean surface
    PortCurve      = rs.ProjectCurveToSurface(PlanPortCurve     , LSPort     ,(0,0,100))

    # House-keeping
    rs.DeleteObjects([LSPort,PlanPortCurve,ParallelLoftEdgePort,RuleLinePort,AftLoftEdgePort])

    # Tidy up the mean curve. This is necessary for a smooth result and removing
    # it can render the algorithm unstable. However, FitCurve itself may sometimes
    # be slightly unstable.
    FLength = abs(Xmax-Xmin) # establish a reference length
    PortCurveSimplified      = rs.FitCurve(PortCurve,     distance_tolerance = FLength*0.001)
    StarboardCurveSimplified = act.MirrorObjectXZ(PortCurveSimplified)
    
    rs.DeleteObject(PortCurve)
    
    # Compute the actual end points of the longitudinal curves
    (Xmin,Ymin,Zmin,Xmax1,Ymax,Zmax) = act.ObjectsExtents(StarboardCurveSimplified)
    (Xmin,Ymin,Zmin,Xmax2,Ymax,Zmax) = act.ObjectsExtents(PortCurveSimplified)
    (Xmin,Ymin,Zmin,Xmax3,Ymax,Zmax) = act.ObjectsExtents(FSVUCurve)
    (Xmin,Ymin,Zmin,Xmax4,Ymax,Zmax) = act.ObjectsExtents(FSVLCurve)
    EndX = min([Xmax1,Xmax2,Xmax3,Xmax4])

    return StarboardCurveSimplified, PortCurveSimplified, FSVUCurve, FSVLCurve, FSVMeanCurve, NoseEndX, TailStartX, EndX


def _BuildFuselageOML(NoseLengthRatio, TailLengthRatio, CylindricalMidSection, SimplificationReqd):

    MaxFittingAttempts = 6

    FittingAttempts = -1


    NetworkSrfSettings = [
    [35, 20, 15, 5, 20],
    [35, 30, 15, 5, 20],
    [35, 20, 15, 2, 20],
    [30, 30, 15, 2, 20],
    [30, 20, 15, 2, 20],
    [25, 20, 15, 2, 20],
    [20, 20, 15, 2, 20],
    [15, 20, 15, 2, 20]]

    StarboardCurve, PortCurve, FSVUCurve, FSVLCurve, FSVMeanCurve, NoseEndX, TailStartX, EndX = _FuselageLongitudinalGuideCurves(NoseLengthRatio, TailLengthRatio)


    while FittingAttempts <= MaxFittingAttempts:

        FittingAttempts = FittingAttempts + 1 
        
        # Construct array of cross section definition frames
        SX0 = 0
        Step01 = NetworkSrfSettings[FittingAttempts][0]
        SX1 = 0.04*NoseEndX
        Step12 = NetworkSrfSettings[FittingAttempts][1]
        SX2 = SX1 + 0.25*NoseEndX
        Step23 = NetworkSrfSettings[FittingAttempts][2]
        SX3 = NoseEndX
        Step34 = NetworkSrfSettings[FittingAttempts][3]
        SX4 = TailStartX
        Step45 = NetworkSrfSettings[FittingAttempts][4]
        SX5 = EndX
        
        print "Attempting network surface fit with network density setup ", NetworkSrfSettings[FittingAttempts][:]
        
        
        Stations01 = act.pwfrange(SX0,SX1,max([Step01,2]))
        Stations12 = act.pwfrange(SX1,SX2,max([Step12,2]))
        Stations23 = act.pwfrange(SX2,SX3,max([Step23,2]))
        Stations34 = act.pwfrange(SX3,SX4,max([Step34,2]))
        Stations45 = act.pwfrange(SX4,SX5,max([Step45,2]))
    
        StationRange = Stations01[:-1] + Stations12[:-1] + Stations23[:-1] + Stations34[:-1] + Stations45
        C = []
        FirstTime = True
        for XStation in StationRange:
            P = rs.PlaneFromPoints((XStation,0,0),(XStation,1,0),(XStation,0,1))
            IP1 = rs.PlaneCurveIntersection(P,StarboardCurve)
            IP2 = rs.PlaneCurveIntersection(P,FSVUCurve)
            IP3 = rs.PlaneCurveIntersection(P,PortCurve)
            IP4 = rs.PlaneCurveIntersection(P,FSVLCurve)
            IPcentre = rs.PlaneCurveIntersection(P,FSVMeanCurve)
            IPoint1 = rs.AddPoint(IP1[0][1])
            IPoint2 = rs.AddPoint(IP2[0][1])
            IPoint3 = rs.AddPoint(IP3[0][1])
            IPoint4 = rs.AddPoint(IP4[0][1])
            IPointCentre = rs.AddPoint(IPcentre[0][1])
            PseudoDiameter = abs(IP4[0][1].Z-IP2[0][1].Z)
            if CylindricalMidSection and NoseEndX < XStation < TailStartX:
            # Ensure that the parallel section of the fuselage is cylindrical
            # if CylindricalMidSection is True
                print "Enforcing circularity in the central section..."
                if FirstTime:
                    PseudoRadius = PseudoDiameter/2
                    FirstTime = False
                Pc = rs.PointCoordinates(IPointCentre)
                P1 = P2 = P3 = Pc
                P1 = rs.PointAdd(P1,(0,PseudoRadius,0))
                P2 = rs.PointAdd(P2,(0,0,PseudoRadius))
                P3 = rs.PointAdd(P3,(0,-PseudoRadius,0))
                c = rs.AddCircle3Pt(P1, P2, P3)
            else:
                c = rs.AddInterpCurve([IPoint1,IPoint2,IPoint3,IPoint4,IPoint1],knotstyle=3)
                # Once CSec is implemented in Rhino Python, this could be replaced
            rs.DeleteObjects([IPoint1,IPoint2,IPoint3,IPoint4,IPointCentre])
            list.append(C,c)
    
        # Fit fuselage external surface
        CurveNet = []
        for c in C[1:]:
            list.append(CurveNet,c)
        list.append(CurveNet, FSVUCurve)
        list.append(CurveNet, PortCurve)
        list.append(CurveNet, FSVLCurve)
        list.append(CurveNet, StarboardCurve)
        FuselageOMLSurf = rs.AddNetworkSrf(CurveNet)
        rs.DeleteObjects(C)
        
        if not(FuselageOMLSurf==None):
            print "Network surface fit succesful on attempt ", FittingAttempts+1 
            FittingAttempts = MaxFittingAttempts+1 # Force an exit from 'while'

    # If all attempts at fitting a network surface failed, we attempt a Sweep2
    if FuselageOMLSurf==None:
        print "Failed to fit network surface to the external shape of the fuselage"
        print "Attempting alternative fitting method, quality likely to be low..."

        try:
            FuselageOMLSurf = rs.AddSweep2([FSVUCurve,FSVLCurve],C[:])
        except:
            FuselageOMLSurf = False

        SimplificationReqd = True # Enforce simplification
        if not(FuselageOMLSurf):
            print "Alternative fitting method failed too. Out of ideas."

    if FuselageOMLSurf and SimplificationReqd:
        rs.UnselectAllObjects()
        rs.SelectObject(FuselageOMLSurf)
        ToleranceStr = str(0.0005*EndX)
        print "Smoothing..."
        rs.Command("FitSrf " + ToleranceStr)
        rs.UnselectAllObjects()

    # Compute the stern point coordinates of the fuselage
    Pu = rs.CurveEndPoint(FSVUCurve)
    Pl = rs.CurveEndPoint(FSVLCurve)
    SternPoint = [Pu.X, Pu.Y, 0.5*(Pu.Z+Pl.Z)]

    rs.DeleteObjects([FSVUCurve,FSVLCurve,PortCurve,StarboardCurve,FSVMeanCurve])

    return FuselageOMLSurf, SternPoint


def CockpitWindowContours(Height = 1.620, Depth = 5):
    P1 = [0.000,0.076,Height-1.620+2.194]
    P2 = [0.000,0.852,Height-1.620+2.290]
    P3 = [0.000,0.904,Height+0.037]
    P4 = [0.000,0.076,Height]
    CWC1 = rs.AddPolyline([P1,P2,P3,P4,P1])
    rs.SelectObject(CWC1)
    rs.Command("_FilletCorners 0.08 ")

    P1 = [0.000,0.951,Height-1.620+2.289]
    P2 = [0.000,1.343,Height-1.620+2.224]
    P3 = [0.000,1.634,Height-1.620+1.773]
    P4 = [0.000,1.557,Height-1.620+1.588]
    P5 = [0.000,1.027,Height-1.620+1.671]
    CWC2 = rs.AddPolyline([P1,P2,P3,P4,P5,P1])
    rs.SelectObject(CWC2)
    rs.Command("_FilletCorners 0.08 ")

    CWC3 = act.MirrorObjectXZ(CWC1)
    CWC4 = act.MirrorObjectXZ(CWC2)
    
    ExtPathId = rs.AddLine([0,0,0],[Depth, 0, 0])
    
    CWC1s = rs.ExtrudeCurve(CWC1, ExtPathId)
    CWC2s = rs.ExtrudeCurve(CWC2, ExtPathId)
    CWC3s = rs.ExtrudeCurve(CWC3, ExtPathId)
    CWC4s = rs.ExtrudeCurve(CWC4, ExtPathId)

    rs.DeleteObjects([CWC1, CWC2, CWC3, CWC4, ExtPathId])

    return CWC1s, CWC2s, CWC3s, CWC4s


def WindowContour(WinCenter):
    P1 = [WinCenter[0], 0, WinCenter[1] + 0.468/2]
    P2 = [WinCenter[0] + 0.272/2, 0, WinCenter[1]]
    P3 = [WinCenter[0], 0, WinCenter[1] - 0.468/2]
    P4 = [WinCenter[0] - 0.272/2, 0, WinCenter[1]]

    WCurveU = rs.AddInterpCurve([P4, P1, P2], start_tangent = [0, 0, 2.5], 
    end_tangent = [0, 0, -2.5])
    WCurveL = rs.AddInterpCurve([P2, P3, P4], start_tangent = [0, 0, -2.5], 
    end_tangent = [0, 0, 2.5])
    
    WCurve = rs.JoinCurves([WCurveU, WCurveL], delete_input=True)
    return WCurve

def MakeWindow(FuselageSrf, Xwc, Zwc):
    WinCenter = [Xwc, Zwc]
    WCurve = WindowContour(WinCenter)
    
    ExtPathStbd = rs.AddLine([0,0,0],[0,10,0])
    ExtPathPort = rs.AddLine([0,0,0],[0,-10,0])
    
    TubeStbd = rs.ExtrudeCurve(WCurve, ExtPathStbd)
    FuselageSrf, WinStbd = rs.SplitBrep(FuselageSrf, TubeStbd, delete_input=True)
    TubePort = rs.ExtrudeCurve(WCurve, ExtPathPort)
    FuselageSrf, WinPort = rs.SplitBrep(FuselageSrf, TubePort, delete_input=True)

    rs.DeleteObjects([TubeStbd, TubePort, ExtPathStbd, ExtPathPort, WCurve])

    return WinStbd, WinPort, FuselageSrf



def FuselageOML(NoseLengthRatio = 0.182, TailLengthRatio = 0.293, Scaling = [55.902, 55.902, 55.902], NoseCoordinates = [0,0,0], CylindricalMidSection = False, SimplificationReqd = False):
# Instantiates a parametric fuselage outer mould line (OML) geometry for a given
# set of design variables.
    FuselageOMLSurf, SternPoint = _BuildFuselageOML(NoseLengthRatio, TailLengthRatio,CylindricalMidSection,SimplificationReqd)

    if not(FuselageOMLSurf) or FuselageOMLSurf is None:
        return

    ScalingF = [0,0,0]
    ScalingF[0] = Scaling[0]/55.902
    ScalingF[1] = Scaling[1]/55.902
    ScalingF[2] = Scaling[2]/55.902

    # Overall scaling
    FuselageOMLSurf = act.ScaleObjectWorld000(FuselageOMLSurf, ScalingF)


    # A few other ways of performing the scaling...
    # Variant one: this depends on the current CPlane!
    # FuselageOMLSurf = rs.ScaleObject(FuselageOMLSurf, (0,0,0), Scaling)
    
    # Variant two: define plane in World coordinates
    #P = rs.PlaneFromFrame((0,0,0),(1,0,0),(0,1,0))
    #TransfMatrix = Rhino.Geometry.Transform.Scale(P, Scaling[0], Scaling[1], Scaling[2])
    #FuselageOMLSurf = rs.TransformObjects(FuselageOMLSurf, TransfMatrix)

    # Variant three: World coordinate system based scaling
    #xform = rs.XformScale(Scaling)
    #FuselageOMLSurf = rs.TransformObjects(FuselageOMLSurf, xform)

    SternPoint[0] = SternPoint[0]*ScalingF[0]
    SternPoint[1] = SternPoint[1]*ScalingF[1]
    SternPoint[2] = SternPoint[2]*ScalingF[2]

    # Positioning
    MoveVec = rs.VectorCreate(NoseCoordinates, [0,0,0])
    FuselageOMLSurf = rs.MoveObject(FuselageOMLSurf, MoveVec)
    SternPoint[0] = SternPoint[0]+NoseCoordinates[0]
    SternPoint[1] = SternPoint[1]+NoseCoordinates[1]
    SternPoint[2] = SternPoint[2]+NoseCoordinates[2]
    
    return FuselageOMLSurf, SternPoint


if __name__ == '__main__':
    rs.EnableRedraw(False)
    # The defaults will yield a fuselage geometry similar to that of the 
    # Boeing 787-8.
    FuselageOML()
    # Another example: for a fuselage shape similar to that of the Airbus A380
    # comment out the line above and uncomment the line below:
#    FuselageOML(NoseLengthRatio = 0.182, TailLengthRatio = 0.293, 
#    Scaling = [70.4, 67.36, 80.1], 
#    NoseCoordinates = [0,0,0], 
#    CylindricalMidSection = False, 
#    SimplificationReqd = False)
    rs.EnableRedraw()