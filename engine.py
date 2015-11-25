# A function for generating a parametric high bypass turbofan engine nacelle model.
# Run this file directly to get an example (scroll to the end to see it).
# ==============================================================================
# AirCONICS
# Aircraft CONfiguration through Integrated Cross-disciplinary Scripting 
# version 0.2
# Andras Sobester, 2015.
# Bug reports to a.sobester@soton.ac.uk or @ASobester please.
# ==============================================================================

from __future__ import division
import rhinoscriptsyntax as rs
import primitives, airconics_setup, AirCONICStools as act
import math
import wing_example_transonic_airliner as tea
import liftingsurface

def TurbofanNacelle(EngineSection, Chord, CentreLocation = [0,0,0], 
ScarfAngle = 3, HighlightRadius = 1.45, 
MeanNacelleLength = 5.67):
# The defaults yield a nacelle similar to that of an RR Trent 1000 / GEnx

    HighlightDepth = 0.12*MeanNacelleLength
    SectionNo = 100
    
    # Draw the nacelle with the centre of the intake highlight circle in 0,0,0
    rs.EnableRedraw(False)
    Highlight = rs.AddCircle3Pt((0,0,HighlightRadius),(0,-HighlightRadius,0),(0,0,-HighlightRadius))
    HighlightCutterCircle = rs.AddCircle3Pt((0,0,HighlightRadius*1.5),(0,-HighlightRadius*1.5,0),(0,0,-HighlightRadius*1.5))

    # Fan disk for CFD boundary conditions
    FanCircle = rs.CopyObject(Highlight, (MeanNacelleLength*0.25, 0, 0))
    FanDisk = rs.AddPlanarSrf(FanCircle)
    # Aft outflow for CFD boundary conditions
    BypassCircle = rs.CopyObject(Highlight, (MeanNacelleLength*0.85, 0, 0))
    BypassDisk = rs.AddPlanarSrf(BypassCircle)
    rs.DeleteObjects([FanCircle, BypassCircle])

    # Outflow cone
    TailConeBasePoint = [MeanNacelleLength*0.84, 0,0]
    TailConeApex    = [MeanNacelleLength*1.35, 0, 0]
    TailConeRadius    =  HighlightRadius*0.782
    TailCone = rs.AddCone(TailConeBasePoint, TailConeApex, TailConeRadius)
    # Spinner cone
    SpinnerConeBasePoint = [MeanNacelleLength*0.26, 0,0]
    SpinnerConeApex    = [MeanNacelleLength*0.08, 0, 0]
    SpinnerConeRadius    =  MeanNacelleLength*0.09
    Spinner = rs.AddCone(SpinnerConeBasePoint, SpinnerConeApex, SpinnerConeRadius)

    
    # Tilt the intake
    RotVec = rs.VectorCreate((0,0,0),(0,1,0))
    Highlight = rs.RotateObject(Highlight, (0,0,0), ScarfAngle, axis = RotVec)
    
    # Set up the disk for separating the intake lip later
    HighlightCutterCircle = rs.RotateObject(HighlightCutterCircle, (0,0,0), ScarfAngle, axis = RotVec)
    HighlightCutterDisk = rs.AddPlanarSrf(HighlightCutterCircle)
    rs.DeleteObject(HighlightCutterCircle)
    rs.MoveObject(HighlightCutterDisk, (HighlightDepth, 0,0))
    
    # Build the actual airfoil sections to define the nacelle
    HighlightPointVector = rs.DivideCurve(Highlight, SectionNo)
    
    Sections = []
    TailPoints = []
    Rotation = 0
    Twist = 0
    AirfoilSeligName = 'goe613'
    SmoothingPasses = 1

    for HighlightPoint in HighlightPointVector:
        ChordLength = MeanNacelleLength - HighlightPoint.X
        Af = primitives.Airfoil(HighlightPoint,ChordLength, Rotation, Twist, SeligPath=airconics_setup.SeligPath)
        AfCurve,Chrd = primitives.Airfoil.AddAirfoilFromSeligFile(Af, AirfoilSeligName, SmoothingPasses)
        rs.DeleteObject(Chrd)
        P = rs.CurveEndPoint(AfCurve)
        list.append(TailPoints, P)
        AfCurve = act.AddTEtoOpenAirfoil(AfCurve)
        list.append(Sections, AfCurve)
        Rotation = Rotation + 360.0/SectionNo
    
    list.append(TailPoints, TailPoints[0])
    
    # Build the actual nacelle OML surface
    EndCircle = rs.AddInterpCurve(TailPoints)
    Nacelle = rs.AddSweep2([Highlight, EndCircle], Sections, closed = True)
    # Separate the lip
    Cowling, HighlightSection = rs.SplitBrep(Nacelle, HighlightCutterDisk, True)
    
    
    # Now build the pylon between the engine and the specified chord on the wing
    CP1 = [MeanNacelleLength*0.26+CentreLocation[0],CentreLocation[1],CentreLocation[2]+HighlightRadius*0.1]
    CP2 = [MeanNacelleLength*0.4+CentreLocation[0],CentreLocation[1],HighlightRadius*1.45+CentreLocation[2]]
    CP3 = rs.CurveEndPoint(Chord)
    rs.ReverseCurve(Chord)
    CP4 = rs.CurveEndPoint(Chord)

    # Move the engine into its actual place on the wing
    rs.MoveObjects([HighlightSection, Cowling, FanDisk, BypassDisk, TailCone, Spinner], CentreLocation)

    # Pylon wireframe
    PylonTop = rs.AddInterpCurve([CP1, CP2, CP3, CP4])
    PylonAf = primitives.Airfoil(CP1,MeanNacelleLength*1.35, 90, 0, airconics_setup.SeligPath)
    PylonAfCurve,PylonChord = primitives.Airfoil.AddNACA4(PylonAf, 0, 0, 12, 3)
    LowerTE = rs.CurveEndPoint(PylonChord)
    PylonTE = rs.AddLine(LowerTE, CP4) 

    # Create the actual pylon surface
    PylonLeft = rs.AddNetworkSrf([PylonTop, PylonAfCurve, PylonTE])
    rs.MoveObject(PylonLeft, (0,-CentreLocation[1],0))
    PylonRight = act.MirrorObjectXZ(PylonLeft)
    rs.MoveObject(PylonLeft,  (0,CentreLocation[1],0))
    rs.MoveObject(PylonRight, (0,CentreLocation[1],0))
    PylonAfCurve = act.AddTEtoOpenAirfoil(PylonAfCurve)
    PylonAfSrf = rs.AddPlanarSrf(PylonAfCurve)

    # Assigning basic surface properties
    act.AssignMaterial(Cowling, "ShinyBABlueMetal")
    act.AssignMaterial(HighlightSection, "UnpaintedMetal")
    act.AssignMaterial(TailCone, "UnpaintedMetal")
    act.AssignMaterial(FanDisk, "FanDisk")
    act.AssignMaterial(Spinner, "ShinyBlack")
    act.AssignMaterial(BypassDisk, "FanDisk")
    act.AssignMaterial(PylonLeft,"White_composite_external")
    act.AssignMaterial(PylonRight,"White_composite_external")

    # Clean-up
    rs.DeleteObject(HighlightCutterDisk)
    rs.DeleteObjects(Sections)
    rs.DeleteObject(EndCircle)
    rs.DeleteObject(Highlight)
    rs.DeleteObjects([PylonTop, PylonAfCurve, PylonChord, PylonTE])
    
    
    rs.Redraw()
    
    TFEngine = [Cowling, HighlightSection, TailCone, FanDisk, Spinner, BypassDisk]
    TFPylon = [PylonLeft, PylonRight, PylonAfSrf]
    
    return TFEngine, TFPylon


if __name__ == "__main__":

#    Generate a wing first to attach the engine to
    P = (0,0,0)
    LooseSurf = 1
    SegmentNo = 10
    Wing = liftingsurface.LiftingSurface(P, tea.mySweepAngleFunctionAirliner, 
    tea.myDihedralFunctionAirliner, 
    tea.myTwistFunctionAirliner, 
    tea.myChordFunctionAirliner, 
    tea.myAirfoilFunctionAirliner, 
    LooseSurf, SegmentNo, TipRequired = True)
    ChordFactor = 1
    ScaleFactor = 50
    rs.EnableRedraw(False)
    WingSurf, ActualSemiSpan, LSP_area,  RootChord, AR, WingTip = Wing.GenerateLiftingSurface(ChordFactor, ScaleFactor)
    rs.EnableRedraw()

    SpanStation = 0.3 # The engine is to be placed at 30% span
    EngineDia = 2.9
    NacelleLength = 1.95*EngineDia
    rs.EnableRedraw(False)
    EngineSection, Chord = act.CutSect(WingSurf, SpanStation)
    CEP = rs.CurveEndPoint(Chord)

    # Variables controlling the position of the engine with respect to the wing
    EngineCtrFwdOfLE = 0.98  
    EngineCtrBelowLE = 0.35
    Scarf_deg = 4

#   Now build the engine and its pylon
    EngineStbd, PylonStbd =  TurbofanNacelle(EngineSection, Chord,
    CentreLocation = [CEP.X-EngineCtrFwdOfLE*NacelleLength,CEP.Y,CEP.Z-EngineCtrBelowLE*NacelleLength],
    ScarfAngle = Scarf_deg, HighlightRadius = EngineDia/2.0,
    MeanNacelleLength = NacelleLength)
    
    rs.DeleteObjects([EngineSection, Chord])