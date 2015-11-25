# transonic_airliner.py ========================================================
# Parametric geometry covering some of the design space of 'tube and wing' type 
# transonic airliners. Run this as a script to generate geometries approximating 
# those of the B787 Dreamliner (-8 and -9) and Airbus A380 (uncomment the appro-
# priate line at the end of this file - see the header of the function for brief
# explanations of the design variables, if you wish to build your own instances
# of this parametric model.
#
# NOTE: For good results in terms of visualisation, select the 'Rendered' view
# in your Rhinoceros viewports. Better still, download the free Neon render 
# plugin, which should put a 'Raytraced with Neon' entry in your viewport type
# menu.
# ==============================================================================
# AirCONICS
# Aircraft CONfiguration through Integrated Cross-disciplinary Scripting 
# version 0.2.1
# Andras Sobester, 2015.
# Bug reports to a.sobester@soton.ac.uk or @ASobester please.
# ==============================================================================

from __future__ import division

import wing_example_transonic_airliner as ta
import liftingsurface
import rhinoscriptsyntax as rs
import AirCONICStools as act
import engine
import fuselage_oml
import airlinertail as tail
import wing_example_box_wing as bw
import random

def transonic_airliner(Propulsion=1,# 1 - twin, 2 - quad 
EngineDia=2.9, # Diameter of engine intake highlight 
FuselageScaling = [55.902, 55.902, 55.902], # [x,y,z] scale factors
NoseLengthRatio = 0.182, # Proportion of forward tapering section of the fuselage 
TailLengthRatio = 0.293, # Proportion of aft tapering section of the fuselage
WingScaleFactor = 44.56,
WingChordFactor = 1.0,
Topology = 1, # Topology = 2 will yield a box wing airliner - use with caution, this is just for demo purposes.
SpanStation1 = 0.31, # Inboard engine at this span station
SpanStation2 = 0.625, # Outboard engine at this span station (ignored if Propulsion=1)
EngineCtrBelowLE = 0.3558, # Engine below leading edge, normalised by the length of the nacelle - range: [0.35,0.5]
EngineCtrFwdOfLE = 0.9837, # Engine forward of leading edge, normalised by the length of the nacelle - range: [0.85,1.5]
Scarf_deg = 3): # Engine scarf angle


    # Build fuselage geometry
    rs.EnableRedraw(False)
    try:
        FuselageOMLSurf, SternPoint = fuselage_oml.FuselageOML(NoseLengthRatio, TailLengthRatio, 
        Scaling = FuselageScaling, 
        NoseCoordinates = [0,0,0], 
        CylindricalMidSection = False, 
        SimplificationReqd = False)
    except:
        print "Fuselage fitting failed - stopping."
        return

    FuselageHeight = FuselageScaling[2]*0.105
    FuselageLength = FuselageScaling[0]
    FuselageWidth  = FuselageScaling[1]*0.106
    rs.Redraw()

    if FuselageOMLSurf is None:
        print "Failed to fit fuselage surface, stopping."
        return


    FSurf = rs.CopyObject(FuselageOMLSurf)
    
    # Position of the apex of the wing
    if FuselageHeight < 8.0:
        WingApex = [0.1748*FuselageLength,0,-0.0523*FuselageHeight] #787:[9.77,0,-0.307]
    else:
        WingApex = [0.1748*FuselageLength,0,-0.1*FuselageHeight] #787:[9.77,0,-0.307]


    # Set up the wing object, including the list of user-defined functions that
    # describe the spanwise variations of sweep, dihedral, etc.
    LooseSurf = 1
    if Topology == 1:
        SegmentNo = 10
        Wing = liftingsurface.LiftingSurface(WingApex, ta.mySweepAngleFunctionAirliner,
        ta.myDihedralFunctionAirliner, ta.myTwistFunctionAirliner,
        ta.myChordFunctionAirliner, ta.myAirfoilFunctionAirliner, 
        LooseSurf, SegmentNo, TipRequired = True)
    elif Topology == 2:
        SegmentNo = 101
        Wing = liftingsurface.LiftingSurface(WingApex, ta.mySweepAngleFunctionAirliner,
        bw.myDihedralFunctionBoxWing, ta.myTwistFunctionAirliner,
        ta.myChordFunctionAirliner, ta.myAirfoilFunctionAirliner, 
        LooseSurf, SegmentNo, TipRequired = True)

    # Instantiate the wing object and add it to the document
    rs.EnableRedraw(False)
    WingSurf, ActualSemiSpan, LSP_area,  RootChord, AR, WingTip = Wing.GenerateLiftingSurface(WingChordFactor, WingScaleFactor)
    rs.Redraw()


    if Topology == 1:
        # Add wing to body fairing
        WTBFXCentre = WingApex[0] + RootChord/2.0 + RootChord*0.1297 # 787: 23.8
        if FuselageHeight < 8.0:
            WTBFZ = RootChord*0.009 #787: 0.2
            WTBFheight = 0.1212*RootChord #787:2.7
            WTBFwidth = 1.08*FuselageWidth
        else:
            WTBFZ = WingApex[2] + 0.005*RootChord
            WTBFheight = 0.09*RootChord
            WTBFwidth = 1.15*FuselageWidth
    
        WTBFlength = 1.167*RootChord #787:26
    
        WTBFXStern = WTBFXCentre + WTBFlength/2.0
    
        CommS = "_Ellipsoid %3.2f,0,%3.2f %3.2f,0,%3.2f %3.2f,%3.2f,%3.2f %3.2f,0,%3.2f " % (WTBFXCentre, WTBFZ, WTBFXStern, WTBFZ, 0.5*(WTBFXCentre+WTBFXStern), 0.5*WTBFwidth, WTBFZ, 0.5*(WTBFXCentre+WTBFXStern),WTBFheight)
    
        rs.EnableRedraw(False)
    
        rs.CurrentView("Perspective")
        rs.Command(CommS)
        LO = rs.LastCreatedObjects()
        WTBF = LO[0]
        rs.Redraw()        
    
    
        # Trim wing inboard section
        CutCirc = rs.AddCircle3Pt((0,WTBFwidth/4,-45), (0,WTBFwidth/4,45), (90,WTBFwidth/4,0))
        CutCircDisk = rs.AddPlanarSrf(CutCirc)
        CutDisk = CutCircDisk[0]
        rs.ReverseSurface(CutDisk,1)
        rs.TrimBrep(WingSurf, CutDisk)
    elif Topology == 2:
        # Overlapping wing tips
        CutCirc = rs.AddCircle3Pt((0,0,-45), (0,0,45), (90,0,0))
        CutCircDisk = rs.AddPlanarSrf(CutCirc)
        CutDisk = CutCircDisk[0]
        rs.ReverseSurface(CutDisk,1)
        rs.TrimBrep(WingSurf, CutDisk)




    # Engine installation (nacelle and pylon)

    if Propulsion == 1:
        # Twin, wing mounted
        SpanStation = SpanStation1
        NacelleLength = 1.95*EngineDia
        rs.EnableRedraw(False)
        EngineSection, Chord = act.CutSect(WingSurf, SpanStation)
        CEP = rs.CurveEndPoint(Chord)
        EngineStbd, PylonStbd =  engine.TurbofanNacelle(EngineSection, Chord,
        CentreLocation = [CEP.X-EngineCtrFwdOfLE*NacelleLength,CEP.Y,CEP.Z-EngineCtrBelowLE*NacelleLength],
        ScarfAngle = Scarf_deg, HighlightRadius = EngineDia/2.0,
        MeanNacelleLength = NacelleLength)
        rs.Redraw()
    elif Propulsion == 2:
        # Quad, wing-mounted
        NacelleLength = 1.95*EngineDia

        rs.EnableRedraw(False)
        EngineSection, Chord = act.CutSect(WingSurf, SpanStation1)
        CEP = rs.CurveEndPoint(Chord)

        EngineStbd1, PylonStbd1 =  engine.TurbofanNacelle(EngineSection, Chord,
        CentreLocation = [CEP.X-EngineCtrFwdOfLE*NacelleLength,CEP.Y,CEP.Z-EngineCtrBelowLE*NacelleLength],
        ScarfAngle = Scarf_deg, HighlightRadius = EngineDia/2.0,
        MeanNacelleLength = NacelleLength)
        
        rs.DeleteObjects([EngineSection, Chord])

        EngineSection, Chord = act.CutSect(WingSurf, SpanStation2)
        CEP = rs.CurveEndPoint(Chord)

        EngineStbd2, PylonStbd2 =  engine.TurbofanNacelle(EngineSection, Chord,
        CentreLocation = [CEP.X-EngineCtrFwdOfLE*NacelleLength,CEP.Y,CEP.Z-EngineCtrBelowLE*NacelleLength],
        ScarfAngle = Scarf_deg, HighlightRadius = EngineDia/2.0,
        MeanNacelleLength = NacelleLength)
        rs.Redraw()



    # Script for generating and positioning the fin
    rs.EnableRedraw(False)
    # Position of the apex of the fin
    P = [0.6524*FuselageLength,0.003,FuselageHeight*0.384]
    #P = [36.47,0.003,2.254]55.902
    RotVec = rs.VectorCreate([1,0,0],[0,0,0])
    LooseSurf = 1
    SegmentNo = 200
    Fin = liftingsurface.LiftingSurface(P, 
    tail.mySweepAngleFunctionFin,
    tail.myDihedralFunctionFin, 
    tail.myTwistFunctionFin, 
    tail.myChordFunctionFin, 
    tail.myAirfoilFunctionFin, 
    LooseSurf, SegmentNo)
    ChordFactor = 1.01#787:1.01
    if Topology == 1:
        ScaleFactor = WingScaleFactor/2.032 #787:21.93
    elif Topology == 2:
        ScaleFactor = WingScaleFactor/3.5 
    FinSurf, FinActualSemiSpan, FinArea,  FinRootChord, FinAR, FinTip = Fin.GenerateLiftingSurface(ChordFactor, ScaleFactor)
    FinSurf = rs.RotateObject(FinSurf, P, 90, axis = RotVec)
    FinTip = rs.RotateObject(FinTip, P, 90, axis = RotVec)

    if Topology == 1:
        # Tailplane
        P = [0.7692*FuselageLength,0.000,FuselageHeight*0.29]
        RotVec = rs.VectorCreate([1,0,0],[0,0,0])
        LooseSurf = 1
        SegmentNo = 100
        TP = liftingsurface.LiftingSurface(P,
        tail.mySweepAngleFunctionTP, 
        tail.myDihedralFunctionTP,
        tail.myTwistFunctionTP,
        tail.myChordFunctionTP,
        tail.myAirfoilFunctionTP,
        LooseSurf, SegmentNo)
        ChordFactor = 1.01
        ScaleFactor = 0.388*WingScaleFactor #787:17.3
        TPSurf, TPActualSemiSpan, TPArea,  TPRootChord, TPAR, TPTip = TP.GenerateLiftingSurface(ChordFactor, ScaleFactor)

    rs.EnableRedraw(True)

    rs.DeleteObjects([EngineSection, Chord])
    try:
        rs.DeleteObjects([CutCirc])
    except:
        pass

    try:
        rs.DeleteObjects([CutCircDisk])
    except:
        pass

    # Windows
    
    # Cockpit windows:
    rs.EnableRedraw(False)
    
    CockpitWindowTop = 0.305*FuselageHeight
    
    CWC1s, CWC2s, CWC3s, CWC4s = fuselage_oml.CockpitWindowContours(Height = CockpitWindowTop, Depth = 6)

    FuselageOMLSurf, Win1 = rs.SplitBrep(FuselageOMLSurf, CWC1s, delete_input=True)
    FuselageOMLSurf, Win2 = rs.SplitBrep(FuselageOMLSurf, CWC2s, delete_input=True)
    FuselageOMLSurf, Win3 = rs.SplitBrep(FuselageOMLSurf, CWC3s, delete_input=True)
    FuselageOMLSurf, Win4 = rs.SplitBrep(FuselageOMLSurf, CWC4s, delete_input=True)

    rs.DeleteObjects([CWC1s, CWC2s, CWC3s, CWC4s])

    (Xmin,Ymin,Zmin,Xmax,Ymax,Zmax) = act.ObjectsExtents([Win1, Win2, Win3, Win4])
    CockpitBulkheadX = Xmax

    CockpitWallPlane = rs.PlaneFromPoints([CockpitBulkheadX, -15,-15],
    [CockpitBulkheadX,15,-15],
    [CockpitBulkheadX,-15,15])
    
    CockpitWall = rs.AddPlaneSurface(CockpitWallPlane, 30, 30)
    
    
    if 'WTBF' in locals():
        rs.TrimBrep(WTBF, CockpitWall)

    rs.DeleteObject(CockpitWall)


    # Window lines
    WIN = [1]
    NOWIN = [0]

    # A typical window pattern (including emergency exit windows)
    WinVec = WIN + 2*NOWIN + 9*WIN + 3*NOWIN + WIN + NOWIN + 24*WIN + 2*NOWIN + WIN + NOWIN + 14*WIN + 2*NOWIN + WIN + 20*WIN + 2*NOWIN + WIN + NOWIN + 20*WIN

    if FuselageHeight < 8.0:
        # Single deck
        WindowLineHeight = 0.3555*FuselageHeight
        WinX = 0.1157*FuselageLength
        WindowPitch = 0.609
        WinInd = -1
        while WinX < 0.75*FuselageLength:
            WinInd = WinInd + 1
            if WinVec[WinInd] == 1 and WinX > CockpitBulkheadX:
                WinStbd, WinPort, FuselageOMLSurf = fuselage_oml.MakeWindow(FuselageOMLSurf, WinX, WindowLineHeight)
                act.AssignMaterial(WinStbd,"Plexiglass")
                act.AssignMaterial(WinPort,"Plexiglass")
            WinX = WinX + WindowPitch
    else:
        # Fuselage big enough to accommodate two decks 
        # Lower deck
        WindowLineHeight = 0.17*FuselageHeight #0.166
        WinX = 0.1*FuselageLength #0.112
        WindowPitch = 0.609
        WinInd = 0
        while WinX < 0.757*FuselageLength:
            WinInd = WinInd + 1
            if WinVec[WinInd] == 1 and WinX > CockpitBulkheadX:
                WinStbd, WinPort, FuselageOMLSurf = fuselage_oml.MakeWindow(FuselageOMLSurf, WinX, WindowLineHeight)
                act.AssignMaterial(WinStbd,"Plexiglass")
                act.AssignMaterial(WinPort,"Plexiglass")
            WinX = WinX + WindowPitch
        # Upper deck
        WindowLineHeight = 0.49*FuselageHeight
        WinX = 0.174*FuselageLength #0.184
        WinInd = 0
        while WinX < 0.757*FuselageLength:
            WinInd = WinInd + 1
            if WinVec[WinInd] == 1 and WinX > CockpitBulkheadX:
                WinStbd, WinPort, FuselageOMLSurf = fuselage_oml.MakeWindow(FuselageOMLSurf, WinX, WindowLineHeight)
                act.AssignMaterial(WinStbd,"Plexiglass")
                act.AssignMaterial(WinPort,"Plexiglass")
            WinX = WinX + WindowPitch




    rs.Redraw()

    act.AssignMaterial(FuselageOMLSurf,"White_composite_external")
    act.AssignMaterial(WingSurf,"White_composite_external")
    try:
        act.AssignMaterial(TPSurf,"ShinyBARedMetal")
    except:
        pass
    act.AssignMaterial(FinSurf,"ShinyBARedMetal")
    act.AssignMaterial(Win1,"Plexiglass")
    act.AssignMaterial(Win2,"Plexiglass")
    act.AssignMaterial(Win3,"Plexiglass")
    act.AssignMaterial(Win4,"Plexiglass")


    # Mirror the geometry as required
    act.MirrorObjectXZ(WingSurf)
    act.MirrorObjectXZ(WingTip)
    try:
        act.MirrorObjectXZ(TPSurf)
        act.MirrorObjectXZ(TPTip)
    except:
        pass
    if Propulsion == 1:
        for ObjId in EngineStbd:
            act.MirrorObjectXZ(ObjId)
        act.MirrorObjectXZ(PylonStbd)
    elif Propulsion == 2:
        for ObjId in EngineStbd1:
            act.MirrorObjectXZ(ObjId)
        act.MirrorObjectXZ(PylonStbd1)
        for ObjId in EngineStbd2:
            act.MirrorObjectXZ(ObjId)
        act.MirrorObjectXZ(PylonStbd2)


    rs.DeleteObject(FSurf)
    rs.Redraw()



if __name__ == "__main__":

#    A few examples, instances of this parametric aircraft geometry:

#    '787-8'
    transonic_airliner()

#    '787-9'
#    transonic_airliner(FuselageScaling = [61.9, 55.902, 55.902])

#    'A380'
#     transonic_airliner(Propulsion = 2, 
#     FuselageScaling = [70.4, 67.36, 80.1], WingScaleFactor = 59.26)

#    This, for now, is just intended to stretch the model a bit in terms of 
#    topological variety - it is a box wing version of the 787-8. There is 
#    no serious design intent here, merely a hint of some of the possibilities
#    in this model.
#    transonic_airliner(WingScaleFactor = 66, WingChordFactor = 0.5, Topology =2)