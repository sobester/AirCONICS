# HighLiftDevices.py ========================================================
# Parametric model of the high lift systems on an airliner. Import this library
# to add high lift devices such as flaps and slats to your wing geometry.
# Please follow the comments in each function if you wish to alter some of the 
# parameters and geometric variables.
#
# NOTE: For good results in terms of visualisation, select the 'Rendered' view
# in your Rhinoceros viewports. Better still, download the free Neon render 
# plugin, which should put a 'Raytraced with Neon' entry in your viewport type
# menu.
# ==============================================================================
# High Lift System plugin
# version 0.1
# Umang Rajdev, 2015
# ==============================================================================
# AirCONICS
# Aircraft CONfiguration through Integrated Cross-disciplinary Scripting 
# version 0.2
# Andras Sobester, 2015.
# Bug reports to a.sobester@soton.ac.uk or @ASobester please.
# ==============================================================================
# USAGE
# 1-Import HighLiftDevices into the script where high lift devices are to 
# be added.
# 2-Create lists with the specifications of each device, for example:
# aileronconfig1 = ['aileron', 0.7, 0.9, 0.15, 1.2, -30]
# simpleflapconfig1 = ['simpleflap', 0.3, 0.4999, 0.20, 1.25, -15]
# Ensure that the type is one of the following:
#     -'aileron'
#     -'simpleflap'
#     -'fowlerflap'
#     -'singleslot'
#     -'doubeslot'
#     -'tripleslot'
#     -'spoiler'
#     -'kruger'
#     -'splitflap'
# 3-Add all the device specifications to a single list, for example:
# Devices = [aileronconfig1, simpleflapconfig1]
# 4-Run the function AddHighLiftDevices, inputting the wing surface 
# and the list above.
# ==============================================================================

import rhinoscriptsyntax as rs
import AirCONICStools as act
import math

def param_check(type = "flap", SpanStart = 0.4, SpanEnd = 0.7, Chord = 0.1, Taper = 0.8, DeflectionAngle = 20):
    # All values have sensible defaults entered, and can be altered by the user if 
    # required.  Chord is a representation of the root chord 
    # This funtion checks for errors in input values, mainly checking that values are within limits.

    if type == "flap" :
        index = 0
    elif type == "slat" :
        index = 1
    elif type == "spoiler" :
        index = 2
    else:
        index = 3

    StartLimit = [0.1, 0.05, 0.3, 0.01]
    EndLimit = [0.9, 0.95, 0.85, 0.99]
    ChordLowerLimit = [0.03, 0.03, 0.03, 0.03]
    ChordUpperLimit = [0.5, 0.4, 0.6, 0.5]

    # check SpanStart is within limits
    if SpanStart < StartLimit[index] :
        print ("Not within acceptable range, resetting to default values")
        SpanStart = StartLimit[-1]
        print ("SpanStart is: " + str(SpanStart))
    else:
        print ("SpanStart is: " + str(SpanStart))

    # check SpanEnd is within limits
    if SpanEnd > EndLimit[index] :
            print ("Not within acceptable range, resetting to default values")
            SpanEnd = EndLimit[-1]
            print ("SpanEnd is: " + str(SpanEnd))
    else:
        print ("SpanEnd is: " + str(SpanEnd))

    # check Chord is within limits
    if Chord < ChordLowerLimit[index] or Chord > ChordUpperLimit[index] :
            print ("Not within acceptable range, resetting to default values")
            Chord = 0.2
            print ("Chord is: " + str(Chord))
    else:
        print ("Chord is: " + str(Chord))

    return type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle

def domain_gen(surface_id):
    # Function to generate domain over a surface.
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
        print ("Something went wrong in domain generation")

def AddCurvePoints(Curve):
    # Function to add curve mid/end points
    Start = rs.CurveStartPoint(Curve)
    Mid = rs.CurveMidPoint(Curve)
    End = rs.CurveEndPoint(Curve)
    return [Start, Mid, End]

def slat_geometry(surface_id, type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle):
    # Generate the domain over the wing lofter surface, and generate some important curves.
    u0, u1, v0, v1 = domain_gen(surface_id)
    Aerofoil_Tip = rs.AddInterpCrvOnSrfUV(surface_id, [[u1,v0],[u1,v1]])
    Aerofoil_Root = rs.AddInterpCrvOnSrfUV(surface_id, [[u0,v0],[u0,v1]])
    TE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u0,v0],[u1,v0]])
    LE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u0, v1/2.0],[u1,v1/2.0]])

    # Corrected chord, since over the domain, v is defined all around the aerofoil, rather than just along one of the upper or lower surface.
    CorrectedChord = 0.5*Chord

    Span_Root = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart,v0],[u1*SpanStart,v1]])
    Span_Tip = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanEnd,v0],[u1*SpanEnd,v1]])
    Device_Span = rs.TrimSurface(surface_id, 0, (u1*SpanStart, u1*SpanEnd))
    Root_Span = rs.TrimSurface(surface_id, 0, (u0, u1*SpanStart))
    Tip_Span = rs.TrimSurface(surface_id, 0, (u1*SpanEnd, u1))
    Device_LE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart, v1/2.0],[u1*SpanEnd,v1/2.0]])

    Trim_Span = rs.TrimSurface(surface_id, 0, (u1*SpanStart, u1*SpanEnd))
    Chord2 = 0.5
    deltax1 = 0.05   #this is the difference between upper and lower x/c

    # Generate the upper and lower surfaces of the slat
    Upper_Surface = rs.TrimSurface(Trim_Span, 1, (v1*Chord2, v1*(Chord2-CorrectedChord)))
    Lower_Surface = rs.TrimSurface(Trim_Span, 1, (v1*Chord2, v1*(Chord2+CorrectedChord-deltax1)), True)

    # Upper leading edge (ULE), and the second control line on the upper surface
    deltax2 = 0.01  #this is the difference between the 2 upper x/c control points
    ULE = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart, v1*(Chord2-CorrectedChord)],[u1*SpanEnd, v1*(Chord2-CorrectedChord*Taper)]])
    ULE2 = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart, v1*(Chord2-CorrectedChord+deltax2)],[u1*SpanEnd, v1*(Chord2-CorrectedChord*Taper+deltax2)]])

    # Lower leading edge (LLE), and the second control line on the lower surface
    deltax3 = 0.01  #this is the difference between the 2 lower x/c control points
    LLE = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart, v1*(Chord2+CorrectedChord-deltax1)],[u1*SpanEnd, v1*(Chord2+CorrectedChord*Taper-deltax1)]])
    LLE2 = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart, v1*(Chord2+CorrectedChord-deltax1-deltax3)],[u1*SpanEnd, v1*(Chord2+CorrectedChord*Taper-deltax1-deltax3)]])

    # Trailing edge
    TE = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart,v1],[u1*SpanEnd,v1]])

    # Generate mid surface that runs along the aerofoil camber
    Surface_Camber = rs.AddLoftSrf([Device_LE_Curve, TE], loft_type=1)
    a0, a1, b0, b1 = domain_gen(Surface_Camber)

    # Add the control line that defines the curvature of the slat. Distance is measured from the leading edge, and is a multiple of the chord.
    curvefactor = 0.03  #lower this for a curve with a higher gradient. DO NOT INCREASE ABOVE 0.2
    Mid_Control_Line = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(Chord*curvefactor),b0],[a1*(Chord*curvefactor*Taper),b1]])

    # Loft the surface between the 5 control lines-2 on upper surface, 2 on lower and the one above
    Bezier_Loft = rs.AddLoftSrf([ULE, ULE2, Mid_Control_Line, LLE2, LLE], loft_type=1)
    Bezier_Loft_Copy = rs.AddLoftSrf([ULE, ULE2, Mid_Control_Line, LLE2, LLE], loft_type=1)

    # Split the span using the Bezier_Loft as a cutter, and identify the surface with lowest area (outer surface of slat)
    Split_Surface = rs.SplitBrep(Device_Span, Bezier_Loft, True)
    Split_Areas = []
    for i, j in enumerate(Split_Surface):
        Split_SA = rs.SurfaceArea(Split_Surface[i])[0]
        list.append(Split_Areas, Split_SA)

    Min_Area = min(Split_Areas)
    Min_Area_index = [i for i, j in enumerate(Split_Areas) if j == Min_Area]

    # Join the surfaces and get the SA
    Joined_Surfaces = rs.JoinSurfaces([Bezier_Loft, Split_Surface[Min_Area_index[0]]], True)
    del Split_Surface[Min_Area_index[0]]

    # Add endplates
    Slat_Edges = rs.DuplicateEdgeCurves(Joined_Surfaces)
    End_Plate_Wing1 = rs.AddPlanarSrf([Slat_Edges[0], Slat_Edges[-1]])
    End_Plate_Wing2 = rs.AddPlanarSrf([Slat_Edges[2], Slat_Edges[-2]])
    End_Plate_Slat1 = rs.AddPlanarSrf([Slat_Edges[0], Slat_Edges[-1]])
    End_Plate_Slat2 = rs.AddPlanarSrf([Slat_Edges[2], Slat_Edges[-2]])

    # Completed Slat
    Complete_Slat = rs.JoinSurfaces([Joined_Surfaces, End_Plate_Slat1, End_Plate_Slat2], True)
    SA_Slat = rs.SurfaceArea(Complete_Slat)[0]

    # The wing without the slat
    list.append(Split_Surface, Bezier_Loft_Copy)
    list.append(Split_Surface, Root_Span)
    list.append(Split_Surface, Tip_Span)
    list.append(Split_Surface, End_Plate_Wing1)
    list.append(Split_Surface, End_Plate_Wing2)
    Joined_Wing = rs.JoinSurfaces(Split_Surface , True)
    SA_Wing = rs.SurfaceArea(Joined_Wing)[0]

    # Add the centre of rotation (COR)
    rotationoffset = 0.1
    COR1 = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(Chord*curvefactor + rotationoffset),b0],[a1*(Chord*curvefactor*Taper + rotationoffset),b1]])
    COR1_points = AddCurvePoints(COR1)
    COR = rs.AddLine(COR1_points[0], COR1_points[2])
    COR_points = AddCurvePoints(COR)

    # Deploy
    Mid_Control_Line_points = AddCurvePoints(Mid_Control_Line)
    ULE_points = AddCurvePoints(ULE)
    COR_vector = rs.VectorCreate(COR1_points[0], COR_points[2])
    Device_LE_Curve_points = AddCurvePoints(Device_LE_Curve)
    Translation_vector = rs.VectorCreate(Mid_Control_Line_points[2], COR_points[2])

    Deployed_Slat1 = rs.MoveObject(Complete_Slat, Translation_vector)
    Deployed_Slat = rs.RotateObject(Deployed_Slat1, COR_points[1], DeflectionAngle, COR_vector)

    #Delete geometry
    rs.DeleteObjects([ULE, ULE2, LLE, LLE2, TE, Mid_Control_Line, Device_LE_Curve, Surface_Camber, Upper_Surface, Lower_Surface, Device_Span])
    rs.DeleteObjects([Aerofoil_Root, Aerofoil_Tip, LE_Curve, TE_Curve, Span_Root, Span_Tip, Device_LE_Curve])
    rs.DeleteObjects([COR, COR1])
    rs.DeleteObjects(Slat_Edges)
    rs.DeleteObjects(Split_Surface)

    return Deployed_Slat, SA_Slat, Joined_Wing, SA_Wing

def aileron_geometry(surface_id, type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle):
    # Generate the domain over the wing lofter surface, and generate some important curves.
    Device_Edges_Main = rs.DuplicateEdgeCurves(surface_id)
    u0, u1, v0, v1 = domain_gen(surface_id)
    Aerofoil_Tip = rs.AddInterpCrvOnSrfUV(surface_id, [[u1,v0],[u1,v1]])
    Aerofoil_Root = rs.AddInterpCrvOnSrfUV(surface_id, [[u0,v0],[u0,v1]])
    TE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u0,v0],[u1,v0]])
    LE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u0, v1/2.0],[u1,v1/2.0]])

    # Device Curves (only for ref.)
    Device_LE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart, v1/2.0],[u1*SpanEnd,v1/2.0]])
    Span_Root = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart,v0],[u1*SpanStart,v1]])
    Span_Tip = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanEnd,v0],[u1*SpanEnd,v1]])

    # Correction factor, Chord2 (0.5) is applied
    Device_Span = rs.TrimSurface(surface_id, 0, (u1*SpanStart, u1*SpanEnd))
    Chord2 = 0.5

    # Trim the span to upper and Lower surfaces
    Upper_Surface = rs.TrimSurface(Device_Span, 1, (v0, v1*Chord2))
    Lower_Surface = rs.TrimSurface(Device_Span, 1, (v1*(1-Chord2), v1), True)

    # Generate the Trailing Edge
    TE = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart,v1],[u1*SpanEnd,v1]])

    # Generate mid surface that runs along the aerofoil camber
    Surface_Camber = rs.AddLoftSrf([Device_LE_Curve, TE], loft_type=1)
    a0, a1, b0, b1 = domain_gen(Surface_Camber)

    # Add the ULE curves (these are on the surface camber). Also ref. curves for the LLE. These will later on be projected on to the upper and lower surfaces, as required.
    ULE = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord),b0],[a1*(1-Chord*Taper),b1]])
    Offset_ULE = 0.55*Chord*(a1-a0)  # For wings with severe sweep, this number should be decreased if span is long
    ULE2 = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+0.5*a1*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*0.5*Offset_ULE,b1]])

    # Generate the start/mid/end points of the curves
    ULE_points = AddCurvePoints(ULE)
    ULE_points = AddCurvePoints(ULE) 
    ULE2_points = AddCurvePoints(ULE2) 
    TE_points = AddCurvePoints(TE)


    PPP1 = rs.AddPoint(ULE_points[0])
    rs.SelectObject(PPP1)
    PPP2 = rs.AddPoint(ULE_points[2])
    rs.SelectObject(PPP2)
    

    # Join the start/end points to create a straight line (SL)
    ULE_SL = rs.AddLine(ULE_points[0], ULE_points[2])
    ULE2_SL = rs.AddLine(ULE2_points[0], ULE2_points[2])
    
    ULE_SL_points = AddCurvePoints(ULE_SL)

    # Project the SL curves to the respective surfaces (S-> to Surface)
    ULE_S_interim = rs.ExtendCurveLength(ULE_SL, 2, 0, 0.1*(rs.CurveLength(ULE_SL)))
    ULE2_S_interim = rs.ExtendCurveLength(ULE2_SL, 2, 0, 0.1*(rs.CurveLength(ULE2_SL)))
    ULE_S = rs.ProjectCurveToSurface(ULE_S_interim, Upper_Surface, (0, 0, 1))
    ULE2_S = rs.ProjectCurveToSurface(ULE2_S_interim, Upper_Surface, (0, 0, 1))
    LLE_S_interim = rs.ExtendCurveLength(ULE_SL, 2, 1, 0.1*(rs.CurveLength(ULE_SL)))
    LLE2_S_interim = rs.ExtendCurveLength(ULE2_SL, 2, 1, 0.1*(rs.CurveLength(ULE2_SL)))
    LLE_S = rs.ProjectCurveToSurface(LLE_S_interim, Lower_Surface, (0, 0, 1))
    LLE2_S = rs.ProjectCurveToSurface(LLE2_S_interim, Lower_Surface, (0, 0, 1))
    rs.ReverseCurve(LLE_S)   #reverse curves on lower surface, so they are on the same direction as the rest, for the loft
    rs.ReverseCurve(LLE2_S)

    # Generate the LE curved surface
    LE_aileron = rs.AddLoftSrf([ULE2_S, ULE_S, LLE_S, LLE2_S], loft_type=1)


    Cp1 = rs.CurveEndPoint(ULE2_S)
    Cp2 = rs.CurveStartPoint(ULE2_S)
    Cp3 = rs.CurveStartPoint(LLE2_S)
    Cp4 = rs.CurveEndPoint(LLE2_S)
    
    
#
#
#    PPP3 = rs.AddPoint(Cp1)
#    rs.SelectObject(PPP3)
#    PPP4 = rs.AddPoint(Cp2)
#    rs.SelectObject(PPP4)
#    PPP5 = rs.AddPoint(Cp3)
#    rs.SelectObject(PPP5)
#    PPP6 = rs.AddPoint(Cp4)
#    rs.SelectObject(PPP6)
#
#
#
    Poly = rs.AddPolyline([Cp1, Cp2, Cp3, Cp4, Cp1]) 
    rs.SelectObject(Poly)
    
    CutSurface = rs.AddPlanarSrf(Poly)

    d = rs.Distance(Cp1,Cp4)
#    
#    d1 = abs(Cp3.X-ULE_points[0].X)
#
    CutAbove = rs.CopyObject(CutSurface, [0, 0, 0.1*d])
    CutBelow = rs.CopyObject(CutSurface, [0, 0, -0.1*d])
#
#    CutBack = rs.CopyObject(CutSurface, [-d1, 0, 0])
#    
#    UDom = rs.SurfaceDomain(CutBack,0)
#    VDom = rs.SurfaceDomain(CutBack,1)
#    UExt = (UDom[0]+UDom[1])/2.0
#    VExt = VDom[0]
#
#    rs.ExtendSurface(CutBack, (UExt,VExt),2.0*d)
#
#    UDom = rs.SurfaceDomain(CutBack,0)
#    VDom = rs.SurfaceDomain(CutBack,1)
#    UExt = (UDom[0]+UDom[1])/2.0
#    VExt = VDom[1]
#
#    rs.ExtendSurface(CutBack, (UExt,VExt),2.0*d)

#    CutBrick = rs.OffsetSurface(CutBack, 10*d, both_sides=False, create_solid = True)
#    
#    ExtrudePath = rs.AddLine([0,0,0],[50*d,0,0])
#    CutBrick = rs.ExtrudeSurface(CutBack, ExtrudePath, True)
#
#    rs.DeleteObjects([PPP1, PPP2, PPP3, PPP4, PPP5, PPP6, CutBack, ExtrudePath])
    
    LLine = rs.AddLine(Cp1, Cp2)
    Length = rs.CurveLength(LLine)
    Cp1 = rs.PointAdd(Cp1, [-0.6*d,Length/200.0,d]) # note the 200 here matches the scale of the oversized aileron of 1.01
    Cp2 = rs.PointAdd(Cp2, [-0.6*d,-Length/200.0,d])
    Cp3 = rs.PointAdd(Cp3, [-0.6*d,-Length/200.0,-d])
    Cp4 = rs.PointAdd(Cp4, [-0.6*d,Length/200.0,-d])
    Cp5 = rs.PointAdd(Cp1, [50*d,0,0])
    Cp6 = rs.PointAdd(Cp2, [50*d,0,0])
    Cp7 = rs.PointAdd(Cp3, [50*d,0,0])
    Cp8 = rs.PointAdd(Cp4, [50*d,0,0])
    CutBrick = rs.AddBox([Cp1, Cp2, Cp3, Cp4, Cp5, Cp6, Cp7, Cp8])
    rs.DeleteObjects([PPP1, PPP2, LLine])

    # Split the camber surface and lower surface where they intersect with the LE curved surface
    # Once this is done, identify the lowest area surface, since this is part of the device
    Split_Surface = rs.SplitBrep(Lower_Surface, CutBelow, True)
    
    Split_Areas = []
    for i, j in enumerate(Split_Surface):
        Split_SA = rs.SurfaceArea(Split_Surface[i])[0]
        list.append(Split_Areas, Split_SA)
    Min_Area = min(Split_Areas)
    Min_Area_index = [i for i, j in enumerate(Split_Areas) if j == Min_Area]

    Split_Surface2 = rs.SplitBrep(Upper_Surface, CutAbove, True)
    Split_Areas2 = []
    for i, j in enumerate(Split_Surface2):
        Split_SA2 = rs.SurfaceArea(Split_Surface2[i])[0]
        list.append(Split_Areas2, Split_SA2)
    Min_Area2 = min(Split_Areas2)
    Min_Area_index2 = [i for i, j in enumerate(Split_Areas2) if j == Min_Area2]

    # Join the surfaces and get the SA

    Joined_Surfaces = rs.JoinSurfaces([LE_aileron, Split_Surface[Min_Area_index[0]], Split_Surface2[Min_Area_index2[0]]], True)

    rs.CapPlanarHoles(Joined_Surfaces)
    
    
    Aileron_SA = rs.SurfaceArea(Joined_Surfaces)[0]

    # Add the centre of rotation, and the vector (COR)
    COR_interim = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+0.2*a1*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*0.2*Offset_ULE,b1]])
    COR_interim_points = AddCurvePoints(COR_interim)
    COR_interim_SL = rs.AddLine(COR_interim_points[0], COR_interim_points[2])
    COR = rs.ProjectCurveToSurface(COR_interim_SL, Surface_Camber, (0, 0, 1))
    COR_points = AddCurvePoints(COR)
    COR_vector = rs.VectorCreate(COR_points[0], COR_points[2])

    OversizedAileron = rs.CopyObject(Joined_Surfaces)

    AileronMoveBackVec = rs.VectorCreate(COR_points[1], [0,0,0])
    AileronMoveVec = rs.VectorCreate([0,0,0], COR_points[1])

    AileronScale = [1.1, 1.01, 1.1]

    OversizedAileron = rs.MoveObjects(OversizedAileron, AileronMoveVec)
    OversizedAileron = act.ScaleObjectWorld000(OversizedAileron, AileronScale)
    OversizedAileron = rs.MoveObjects(OversizedAileron, AileronMoveBackVec)

    # union the aileron and brick to make a combined cutter
    OversizedAileron2 = rs.CopyObject(OversizedAileron)
    CutBrick = rs.BooleanUnion([CutBrick, OversizedAileron2])

    OversizedAileronUp = rs.CopyObject(OversizedAileron)
    OversizedAileronDn = rs.CopyObject(OversizedAileron)

    # Deploy
    Deployed_Aileron = rs.RotateObject(Joined_Surfaces, COR_points[1], DeflectionAngle, COR_vector)

    OversizedAileronUp = rs.RotateObject(OversizedAileronUp, COR_points[1], DeflectionAngle, COR_vector)
    OversizedAileronDn = rs.RotateObject(OversizedAileronDn, COR_points[1], -DeflectionAngle, COR_vector)

    # Delete objects
    rs.DeleteObjects([Aerofoil_Tip, Aerofoil_Root, TE_Curve, LE_Curve, Device_LE_Curve, Span_Root, Span_Tip, TE])
    rs.DeleteObjects([Surface_Camber, Upper_Surface, Lower_Surface])
    rs.DeleteObjects([LLE_S_interim, LLE2_S_interim, LLE_S, LLE2_S])
    rs.DeleteObjects([ULE, ULE2, ULE_S, ULE2_S])
    rs.DeleteObjects([COR_interim, COR_interim_SL, COR])
    rs.DeleteObjects(Split_Surface)
    rs.DeleteObjects(Split_Surface2)
#    rs.DeleteObjects(Device_Edges)
    rs.DeleteObjects([ULE_SL, ULE2_SL])
    rs.DeleteObjects(Device_Edges_Main)
    rs.DeleteObjects([CutSurface, CutAbove, CutBelow, Poly])

    # Return
    
    
    return Deployed_Aileron, Aileron_SA, OversizedAileron, OversizedAileronUp, OversizedAileronDn, CutBrick

def fowler_geometry(surface_id, type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle):
    # Generate the domain over the wing lofter surface, and generate some important curves.
    Device_Edges_Main = rs.DuplicateEdgeCurves(surface_id)
    u0, u1, v0, v1 = domain_gen(surface_id)
    Aerofoil_Tip = rs.AddInterpCrvOnSrfUV(surface_id, [[u1,v0],[u1,v1]])
    Aerofoil_Root = rs.AddInterpCrvOnSrfUV(surface_id, [[u0,v0],[u0,v1]])
    TE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u0,v0],[u1,v0]])
    LE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u0, v1/2.0],[u1,v1/2.0]])

    # Device Curves (only for ref.)
    Device_LE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart, v1/2.0],[u1*SpanEnd,v1/2.0]])
    Span_Root = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart,v0],[u1*SpanStart,v1]])
    Span_Tip = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanEnd,v0],[u1*SpanEnd,v1]])

    # Correction factor, Chord2 (0.5) is applied
    Device_Span = rs.TrimSurface(surface_id, 0, (u1*SpanStart, u1*SpanEnd))
    Chord2 = 0.5

    # Trim to upper and Lower surfaces
    Upper_Surface = rs.TrimSurface(Device_Span, 1, (v0, v1*Chord2))
    Lower_Surface = rs.TrimSurface(Device_Span, 1, (v1*(1-Chord2), v1), True)

    # Generate the Trailing Edge
    TE = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart,v1],[u1*SpanEnd,v1]])

    # Generate mid surface that runs along the aerofoil camber
    Surface_Camber = rs.AddLoftSrf([Device_LE_Curve, TE], loft_type=1)
    a0, a1, b0, b1 = domain_gen(Surface_Camber)

    # Add the ULE curves (these are on the surface camber). Also ref. curves for the LLE. These will later on be projected on to the respective surfaces, as required.
    ULE = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord),b0],[a1*(1-Chord*Taper),b1]])
    Offset_ULE = 0.5*Chord*(a1-a0)  # For wings with severe sweep, this number should be decreased if long flaps are being used
    ULE2 = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*Offset_ULE,b1]])
    LLE_ref = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*0.25*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*0.25*Offset_ULE,b1]])
    LLE2_ref = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*0.2*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*0.2*Offset_ULE,b1]])

    # Generate the start/mid/end points of the curves
    ULE_points = AddCurvePoints(ULE)
    ULE_points = AddCurvePoints(ULE) 
    ULE2_points = AddCurvePoints(ULE2) 
    LLE_ref_points = AddCurvePoints(LLE_ref) 
    LLE2_ref_points = AddCurvePoints(LLE2_ref) 
    TE_points = AddCurvePoints(TE)

    # Join the start/end points to create a straight line (SL)
    ULE_SL = rs.AddLine(ULE_points[0], ULE_points[2])
    ULE2_SL = rs.AddLine(ULE2_points[0], ULE2_points[2])
    LLE_ref_SL = rs.AddLine(LLE_ref_points[0], LLE_ref_points[2])
    LLE2_ref_SL = rs.AddLine(LLE2_ref_points[0], LLE2_ref_points[2])
    ULE_SL_points = AddCurvePoints(ULE_SL)

    # Project the SL curves to the respective surfaces (S-> to Surface)
    LLE_S_interim = rs.ExtendCurveLength(LLE_ref_SL, 2, 1, 0.1*(rs.CurveLength(LLE_ref_SL)))
    LLE_S = rs.ProjectCurveToSurface(LLE_S_interim, Lower_Surface, (0, 0, 1))
    LLE2_S_interim = rs.ExtendCurveLength(LLE2_ref_SL, 2, 1, 0.1*(rs.CurveLength(LLE2_ref_SL)))
    LLE2_S = rs.ProjectCurveToSurface(LLE2_S_interim, Lower_Surface, (0, 0, 1))
    rs.ReverseCurve(LLE_S)   #reverse curves on lower surface, so they are on the same direction as the rest, for the loft
    rs.ReverseCurve(LLE2_S)
    ULE_S = rs.ProjectCurveToSurface(ULE_SL, Surface_Camber, (0, 0, 1))
    ULE2_S = rs.ProjectCurveToSurface(ULE2_SL, Surface_Camber, (0, 0, 1))
    LLE_S_points = AddCurvePoints(LLE_S)

    # Generate the LE curved surface
    LE_Fowler = rs.AddLoftSrf([ULE2_S, ULE_S, LLE2_S, LLE_S], loft_type=1)

    # Split the camber surface and lower surface where they intersect with the LE curved surface
    # Once this is done, identify the lowest area surface, since this is part of the fowler
    Split_Surface = rs.SplitBrep(Lower_Surface, LE_Fowler, True)
    Split_Areas = []
    for i, j in enumerate(Split_Surface):
        Split_SA = rs.SurfaceArea(Split_Surface[i])[0]
        list.append(Split_Areas, Split_SA)
    Min_Area = min(Split_Areas)
    Min_Area_index = [i for i, j in enumerate(Split_Areas) if j == Min_Area]

    Split_Surface2 = rs.SplitBrep(Surface_Camber, LE_Fowler, True)
    Split_Areas2 = []
    for i, j in enumerate(Split_Surface2):
        Split_SA2 = rs.SurfaceArea(Split_Surface2[i])[0]
        list.append(Split_Areas2, Split_SA2)
    Min_Area2 = min(Split_Areas2)
    Min_Area_index2 = [i for i, j in enumerate(Split_Areas2) if j == Min_Area2]

    # Join the surfaces and get the SA
    Joined_Surfaces = rs.JoinSurfaces([LE_Fowler, Split_Surface[Min_Area_index[0]], Split_Surface2[Min_Area_index2[0]]], True)

    # Add endplates
    Device_Edges = rs.DuplicateEdgeCurves(Joined_Surfaces)
    # This check is required because some of the wings defined are not closed aerofoils and hence have 2 trailing edges.
    if len(Device_Edges_Main) < 4:
        End_Plate_1 = rs.AddEdgeSrf([Device_Edges[0], Device_Edges[5], Device_Edges[8]])
        End_Plate_2 = rs.AddEdgeSrf([Device_Edges[2], Device_Edges[6], Device_Edges[7]])
    else:
        End_Plate_1 = rs.AddEdgeSrf([Device_Edges[0], Device_Edges[5], Device_Edges[8]])
        End_Plate_2 = rs.AddEdgeSrf([Device_Edges[2], Device_Edges[6], Device_Edges[7]])

    # Join the fowler with endplates and obtain the SA
    Complete_Fowler = rs.JoinSurfaces([Joined_Surfaces, End_Plate_1, End_Plate_2], True)
    Fowler_SA = rs.SurfaceArea(Complete_Fowler)[0]

    # Add the centre of rotation, and the vector (COR)
    Translation_vector = rs.VectorCreate(TE_points[2], LLE_S_points[2])
    COR = rs.MoveObject(LLE_S, 0.6*Translation_vector)
    COR_points = AddCurvePoints(COR)
    COR_vector = rs.VectorCreate(COR_points[0], COR_points[2])

    # Deploy
    Deployed_Fowler1 = rs.MoveObject(Complete_Fowler, Translation_vector)
    Deployed_Fowler = rs.RotateObject(Deployed_Fowler1, COR_points[1], DeflectionAngle, COR_vector)

    # Delete objects
    rs.DeleteObjects([Aerofoil_Tip, Aerofoil_Root, TE_Curve, LE_Curve, Device_LE_Curve, Span_Root, Span_Tip, TE])
    rs.DeleteObjects([Surface_Camber, Upper_Surface, Lower_Surface])
    rs.DeleteObjects([LLE_ref, LLE2_ref, LLE_ref_SL, LLE2_ref_SL, LLE_S_interim, LLE2_S_interim, LLE_S, LLE2_S])
    rs.DeleteObjects([ULE, ULE2, ULE_S, ULE2_S])
    rs.DeleteObjects(Split_Surface)
    rs.DeleteObjects(Split_Surface2)
    rs.DeleteObjects(Device_Edges)
    rs.DeleteObjects(Device_Edges_Main)
    rs.DeleteObjects([ULE_SL, ULE2_SL])

    return Deployed_Fowler, Fowler_SA

def splitflap_geometry(surface_id, type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle):
    # Generate the domain over the wing lofter surface, and generate some important curves.
    Device_Edges_Main = rs.DuplicateEdgeCurves(surface_id)
    u0, u1, v0, v1 = domain_gen(surface_id)
    Aerofoil_Tip = rs.AddInterpCrvOnSrfUV(surface_id, [[u1,v0],[u1,v1]])
    Aerofoil_Root = rs.AddInterpCrvOnSrfUV(surface_id, [[u0,v0],[u0,v1]])
    TE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u0,v0],[u1,v0]])
    LE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u0, v1/2.0],[u1,v1/2.0]])

    # Device Curves (only for ref.)
    Device_LE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart, v1/2.0],[u1*SpanEnd,v1/2.0]])
    Span_Root = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart,v0],[u1*SpanStart,v1]])
    Span_Tip = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanEnd,v0],[u1*SpanEnd,v1]])

    # Correction factor, Chord2 (0.5) is applied
    Device_Span = rs.TrimSurface(surface_id, 0, (u1*SpanStart, u1*SpanEnd))
    Chord2 = 0.5

    # Trim to upper and Lower surfaces
    Upper_Surface = rs.TrimSurface(Device_Span, 1, (v0, v1*Chord2))
    Lower_Surface = rs.TrimSurface(Device_Span, 1, (v1*(1-Chord2), v1), True)

    # Generate the Trailing Edge
    TE = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart,v1],[u1*SpanEnd,v1]])

    # Generate mid surface that runs along the aerofoil camber
    Surface_Camber = rs.AddLoftSrf([Device_LE_Curve, TE], loft_type=1)
    a0, a1, b0, b1 = domain_gen(Surface_Camber)

    # Add the ULE curves (these are on the surface camber). Also ref. curves for the LLE. These will later on be projected on to the respective surfaces, as required.
    ULE = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord),b0],[a1*(1-Chord*Taper),b1]])
    Offset_ULE = 0.5*Chord*(a1-a0)  # For wings with severe sweep, this number should be decreased if long flaps are being used
    ULE2 = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*0.25*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*0.25*Offset_ULE,b1]])

    # Generate the start/mid/end points of the curves
    ULE_points = AddCurvePoints(ULE)
    ULE_points = AddCurvePoints(ULE) 
    ULE2_points = AddCurvePoints(ULE2) 
    TE_points = AddCurvePoints(TE)

    # Join the start/end points to create a straight line (SL)
    ULE_SL = rs.AddLine(ULE_points[0], ULE_points[2])
    ULE2_SL = rs.AddLine(ULE2_points[0], ULE2_points[2])
    ULE_SL_points = AddCurvePoints(ULE_SL)

    # Project the SL curves to the respective surfaces (S-> to Surface)
    LLE_S_interim = rs.ExtendCurveLength(ULE_SL, 2, 1, 0.1*(rs.CurveLength(ULE_SL)))
    LLE_S = rs.ProjectCurveToSurface(LLE_S_interim, Lower_Surface, (0, 0, 1))
    LLE2_S_interim = rs.ExtendCurveLength(ULE2_SL, 2, 1, 0.1*(rs.CurveLength(ULE2_SL)))
    LLE2_S = rs.ProjectCurveToSurface(LLE2_S_interim, Lower_Surface, (0, 0, 1))
    rs.ReverseCurve(LLE_S)   #reverse curves on lower surface, so they are on the same direction as the rest, for the loft
    rs.ReverseCurve(LLE2_S)
    ULE_S = rs.ProjectCurveToSurface(ULE_SL, Surface_Camber, (0, 0, 1))
    ULE2_S = rs.ProjectCurveToSurface(ULE2_SL, Surface_Camber, (0, 0, 1))
    LLE_S_points = AddCurvePoints(LLE_S)

    # Generate the LE curved surface
    LE_SplitFlap = rs.AddLoftSrf([ULE2_S, ULE_S, LLE_S, LLE2_S], loft_type=1)

    # Split the camber surface and lower surface where they intersect with the LE curved surface
    # Once this is done, identify the lowest area surface, since this is part of the split flap
    Split_Surface = rs.SplitBrep(Lower_Surface, LE_SplitFlap, True)
    Split_Areas = []
    for i, j in enumerate(Split_Surface):
        Split_SA = rs.SurfaceArea(Split_Surface[i])[0]
        list.append(Split_Areas, Split_SA)
    Min_Area = min(Split_Areas)
    Min_Area_index = [i for i, j in enumerate(Split_Areas) if j == Min_Area]

    Split_Surface2 = rs.SplitBrep(Surface_Camber, LE_SplitFlap, True)
    Split_Areas2 = []
    for i, j in enumerate(Split_Surface2):
        Split_SA2 = rs.SurfaceArea(Split_Surface2[i])[0]
        list.append(Split_Areas2, Split_SA2)
    Min_Area2 = min(Split_Areas2)
    Min_Area_index2 = [i for i, j in enumerate(Split_Areas2) if j == Min_Area2]

    # Join the surfaces and get the SA
    Joined_Surfaces = rs.JoinSurfaces([LE_SplitFlap, Split_Surface[Min_Area_index[0]], Split_Surface2[Min_Area_index2[0]]], True)

    # Add endplates
    Device_Edges = rs.DuplicateEdgeCurves(Joined_Surfaces)
    # This check is required because some of the wings defined are not closed aerofoils and hence have 2 trailing edges.
    if len(Device_Edges_Main) < 4:
        End_Plate_1 = rs.AddEdgeSrf([Device_Edges[0], Device_Edges[5], Device_Edges[8]])
        End_Plate_2 = rs.AddEdgeSrf([Device_Edges[2], Device_Edges[6], Device_Edges[7]])
    else:
        End_Plate_1 = rs.AddEdgeSrf([Device_Edges[0], Device_Edges[5], Device_Edges[8]])
        End_Plate_2 = rs.AddEdgeSrf([Device_Edges[2], Device_Edges[6], Device_Edges[7]])

    # Join the split flap with endplates and obtain the SA
    Complete_SplitFlap = rs.JoinSurfaces([Joined_Surfaces, End_Plate_1, End_Plate_2], True)
    SplitFlap_SA = rs.SurfaceArea(Complete_SplitFlap)[0]

    # Add the centre of rotation, and the vector (COR)
    Translation_vector = rs.VectorCreate(TE_points[2], LLE_S_points[2])
    COR = ULE2_S
    COR_points = AddCurvePoints(COR)
    COR_vector = rs.VectorCreate(COR_points[0], COR_points[2])

    # Deploy
    Deployed_SplitFlap = rs.RotateObject(Complete_SplitFlap, COR_points[1], DeflectionAngle, COR_vector)

    # Delete objects
    rs.DeleteObjects([Aerofoil_Tip, Aerofoil_Root, TE_Curve, LE_Curve, Device_LE_Curve, Span_Root, Span_Tip, TE])
    rs.DeleteObjects([Surface_Camber, Upper_Surface, Lower_Surface])
    rs.DeleteObjects([LLE_S_interim, LLE2_S_interim, LLE_S, LLE2_S])
    rs.DeleteObjects([ULE, ULE2, ULE_S, ULE2_S])
    rs.DeleteObjects(Split_Surface)
    rs.DeleteObjects(Split_Surface2)
    rs.DeleteObjects(Device_Edges)
    rs.DeleteObjects(Device_Edges_Main)
    rs.DeleteObjects([ULE_SL, ULE2_SL])

    return Deployed_SplitFlap, SplitFlap_SA

def spoiler_geometry(surface_id, type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle):
    # Generate the domain over the wing lofter surface, and generate some important curves.
    Device_Edges_Main = rs.DuplicateEdgeCurves(surface_id)
    u0, u1, v0, v1 = domain_gen(surface_id)
    Aerofoil_Tip = rs.AddInterpCrvOnSrfUV(surface_id, [[u1,v0],[u1,v1]])
    Aerofoil_Root = rs.AddInterpCrvOnSrfUV(surface_id, [[u0,v0],[u0,v1]])
    TE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u0,v0],[u1,v0]])
    LE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u0, v1/2.0],[u1,v1/2.0]])

    # Device Curves (only for ref.)
    Device_LE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart, v1/2.0],[u1*SpanEnd,v1/2.0]])
    Span_Root = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart,v0],[u1*SpanStart,v1]])
    Span_Tip = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanEnd,v0],[u1*SpanEnd,v1]])

    # Correction factor, Chord2 (0.5) is applied
    Device_Span = rs.TrimSurface(surface_id, 0, (u1*SpanStart, u1*SpanEnd))
    Chord2 = 0.5

    # Trim the span to upper and Lower surfaces
    Upper_Surface = rs.TrimSurface(Device_Span, 1, (v0, v1*Chord2))
    Lower_Surface = rs.TrimSurface(Device_Span, 1, (v1*(1-Chord2), v1), True)

    # Generate the Trailing Edge
    TE = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart,v1],[u1*SpanEnd,v1]])

    # Generate mid surface that runs along the aerofoil camber. Note that Surface_Camber will be the lower surface of the spoiler.
    Surface_Camber = rs.AddLoftSrf([Device_LE_Curve, TE], loft_type=1)
    a0, a1, b0, b1 = domain_gen(Surface_Camber)

    # Add the ULE curves (these are on the surface camber). Also ref. curves for the LLE. These will later on be projected on to the upper and lower surfaces, as required.
    ULE = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord),b0],[a1*(1-Chord*Taper),b1]])
    Offset_ULE = 0.55*Chord*(a1-a0)  # For wings with severe sweep, this number should be decreased if span is long
    UTE = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*Offset_ULE,b1]])

    # Generate the start/mid/end points of the curves
    ULE_points = AddCurvePoints(ULE)
    UTE_points = AddCurvePoints(UTE)

    # Join the start/end points to create a straight line (SL)
    ULE_SL = rs.AddLine(ULE_points[0], ULE_points[2])
    UTE_SL = rs.AddLine(UTE_points[0], UTE_points[2])

    # Project the SL curves to the respective surfaces (S-> to Surface)
    ULE_S_interim = rs.ExtendCurveLength(ULE_SL, 2, 0, 0.1*(rs.CurveLength(ULE_SL)))
    UTE_S_interim = rs.ExtendCurveLength(UTE_SL, 2, 0, 0.1*(rs.CurveLength(UTE_SL)))
    ULE_S = rs.ProjectCurveToSurface(ULE_S_interim, Upper_Surface, (0, 0, 1))
    Mid_Control_Line_S = rs.ProjectCurveToSurface(ULE_SL, Surface_Camber, (0, 0, 1))
    UTE_S = rs.ProjectCurveToSurface(UTE_S_interim, Upper_Surface, (0, 0, 1))

    # Generate the LE curved surface
    LE_Spoiler_interim = rs.AddLoftSrf([ULE_S, Mid_Control_Line_S], loft_type=1)
    s0, s1, t0, t1 = domain_gen(LE_Spoiler_interim)
    LLE = rs.AddInterpCrvOnSrfUV(LE_Spoiler_interim, [[0.1*s1, t0],[0.1*s1, t1]])
    LE_Spoiler = rs.AddLoftSrf([ULE_S, LLE], loft_type=1)
    Upper_Surface_Spoiler = rs.AddLoftSrf([ULE_S, UTE_S], loft_type=1)
    Lower_Surface_Spoiler = rs.AddLoftSrf([LLE, UTE_S], loft_type=1)

    # Join the surfaces and get the SA
    Joined_Surfaces = rs.JoinSurfaces([LE_Spoiler, Upper_Surface_Spoiler, Lower_Surface_Spoiler], True)
    Spoiler_SA = rs.SurfaceArea(Joined_Surfaces)[0]

    # Add the centre of rotation, and the vector (COR)
    # The COR is ULE_S
    ULE_S_points = AddCurvePoints(ULE_S)
    COR_vector = rs.VectorCreate(ULE_S_points[0], ULE_S_points[2])

    # Deploy
    Deployed_Spoiler = rs.RotateObject(Joined_Surfaces, ULE_S_points[1], DeflectionAngle, COR_vector)

    # Delete objects
    rs.DeleteObjects([Aerofoil_Tip, Aerofoil_Root, TE_Curve, LE_Curve, Device_LE_Curve, Span_Root, Span_Tip, TE])
    rs.DeleteObjects([ULE, UTE])
    rs.DeleteObjects([LE_Spoiler_interim, LLE, Mid_Control_Line_S])
    rs.DeleteObjects([Surface_Camber, Upper_Surface, Lower_Surface])
    rs.DeleteObjects([ULE_SL, UTE_SL, ULE_S, UTE_S])

    # Return
    return Deployed_Spoiler, Spoiler_SA

def singleslot_geometry(surface_id, type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle):
    # Generate the domain over the wing lofter surface, and generate some important curves.
    Device_Edges_Main = rs.DuplicateEdgeCurves(surface_id)
    u0, u1, v0, v1 = domain_gen(surface_id)
    Aerofoil_Tip = rs.AddInterpCrvOnSrfUV(surface_id, [[u1,v0],[u1,v1]])
    Aerofoil_Root = rs.AddInterpCrvOnSrfUV(surface_id, [[u0,v0],[u0,v1]])
    TE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u0,v0],[u1,v0]])
    LE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u0, v1/2.0],[u1,v1/2.0]])

    # Device Curves (only for ref.)
    Device_LE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart, v1/2.0],[u1*SpanEnd,v1/2.0]])
    Span_Root = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart,v0],[u1*SpanStart,v1]])
    Span_Tip = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanEnd,v0],[u1*SpanEnd,v1]])

    # Correction factor, Chord2 (0.5) is applied
    Device_Span = rs.TrimSurface(surface_id, 0, (u1*SpanStart, u1*SpanEnd))
    Chord2 = 0.5

    # Trim to upper and Lower surfaces
    Upper_Surface = rs.TrimSurface(Device_Span, 1, (v0, v1*Chord2))
    Lower_Surface = rs.TrimSurface(Device_Span, 1, (v1*(1-Chord2), v1), True)

    # Generate the Trailing Edge
    TE = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart,v1],[u1*SpanEnd,v1]])

    # Generate mid surface that runs along the aerofoil camber
    Surface_Camber = rs.AddLoftSrf([Device_LE_Curve, TE], loft_type=1)
    a0, a1, b0, b1 = domain_gen(Surface_Camber)

    # Add the ULE curves (these are on the surface camber). Also ref. curves for the LLE. These will later on be projected on to the upper and lower surfaces, as required.
    ULE = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord),b0],[a1*(1-Chord*Taper),b1]])
    Offset_ULE = 0.55*Chord*(a1-a0)  # For wings with severe sweep, this number should be decreased if long flaps are being used
    ULE2 = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*Offset_ULE,b1]])
    LLE_ref = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*0.35*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*0.35*Offset_ULE,b1]])
    LLE2_ref = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*0.2*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*0.2*Offset_ULE,b1]])

    # Generate the start/mid/end points of the curves
    ULE_points = AddCurvePoints(ULE)
    ULE_points = AddCurvePoints(ULE) 
    ULE2_points = AddCurvePoints(ULE2) 
    LLE_ref_points = AddCurvePoints(LLE_ref) 
    LLE2_ref_points = AddCurvePoints(LLE2_ref) 
    TE_points = AddCurvePoints(TE)

    # Join the start/end points to create a straight line (SL)
    ULE_SL = rs.AddLine(ULE_points[0], ULE_points[2])
    ULE2_SL = rs.AddLine(ULE2_points[0], ULE2_points[2])
    LLE_ref_SL = rs.AddLine(LLE_ref_points[0], LLE_ref_points[2])
    LLE2_ref_SL = rs.AddLine(LLE2_ref_points[0], LLE2_ref_points[2])
    ULE_SL_points = AddCurvePoints(ULE_SL)

    # Project the SL curves to the respective surfaces (S-> to Surface)
    LLE_S_interim = rs.ExtendCurveLength(LLE_ref_SL, 2, 1, 0.1*(rs.CurveLength(LLE_ref_SL)))
    LLE_S = rs.ProjectCurveToSurface(LLE_S_interim, Lower_Surface, (0, 0, 1))
    LLE2_S_interim = rs.ExtendCurveLength(LLE2_ref_SL, 2, 1, 0.1*(rs.CurveLength(LLE2_ref_SL)))
    LLE2_S = rs.ProjectCurveToSurface(LLE2_S_interim, Lower_Surface, (0, 0, 1))
    rs.ReverseCurve(LLE_S)   #reverse curves on lower surface, so they are on the same direction as the rest, for the loft
    rs.ReverseCurve(LLE2_S)
    ULE_S_interim = rs.ExtendCurveLength(ULE_SL, 2, 0, 0.1*(rs.CurveLength(ULE_SL)))
    ULE2_S_interim = rs.ExtendCurveLength(ULE2_SL, 2, 0, 0.1*(rs.CurveLength(ULE2_SL)))
    ULE_S = rs.ProjectCurveToSurface(ULE_S_interim, Upper_Surface, (0, 0, 1))
    ULE2_S = rs.ProjectCurveToSurface(ULE2_S_interim, Upper_Surface, (0, 0, 1))
    LLE_S_points = AddCurvePoints(LLE_S)
    ULE_S_points = AddCurvePoints(ULE_S)
    ULE2_S_points = AddCurvePoints(ULE2_S)

    # Generate the LE curved surface
    LE_singleslot = rs.AddLoftSrf([ULE2_S, ULE_S, LLE2_S, LLE_S], loft_type=1)

    # Split the camber surface and lower surface where they intersect with the LE curved surface
    # Once this is done, identify the lowest area surface, since this is part of the flap
    Split_Surface = rs.SplitBrep(Lower_Surface, LE_singleslot, True)
    Split_Areas = []
    for i, j in enumerate(Split_Surface):
        Split_SA = rs.SurfaceArea(Split_Surface[i])[0]
        list.append(Split_Areas, Split_SA)
    Min_Area = min(Split_Areas)
    Min_Area_index = [i for i, j in enumerate(Split_Areas) if j == Min_Area]

    Split_Surface2 = rs.SplitBrep(Upper_Surface, LE_singleslot, True)
    Split_Areas2 = []
    for i, j in enumerate(Split_Surface2):
        Split_SA2 = rs.SurfaceArea(Split_Surface2[i])[0]
        list.append(Split_Areas2, Split_SA2)
    Min_Area2 = min(Split_Areas2)
    Min_Area_index2 = [i for i, j in enumerate(Split_Areas2) if j == Min_Area2]

    # Join the surfaces and get the SA
    Joined_Surfaces = rs.JoinSurfaces([LE_singleslot, Split_Surface[Min_Area_index[0]], Split_Surface2[Min_Area_index2[0]]], True)

    # Add endplates
    Device_Edges = rs.DuplicateEdgeCurves(Joined_Surfaces)
    # This check is required because some of the wings defined are not closed aerofoils and hence have 2 trailing edges.
    if len(Device_Edges_Main) < 4:
        End_Plate_1 = rs.AddEdgeSrf([Device_Edges[0], Device_Edges[5], Device_Edges[8]])
        End_Plate_2 = rs.AddEdgeSrf([Device_Edges[2], Device_Edges[6], Device_Edges[7]])
    else:
        End_Plate_1 = rs.AddEdgeSrf([Device_Edges[0], Device_Edges[5], Device_Edges[9]])
        End_Plate_2 = rs.AddEdgeSrf([Device_Edges[2], Device_Edges[6], Device_Edges[8]])

    # Join the singleslotted with endplates and obtain the SA
    Complete_Singleslot = rs.JoinSurfaces([Joined_Surfaces, End_Plate_1, End_Plate_2], True)
    Singleslot_SA = rs.SurfaceArea(Complete_Singleslot)[0]

    # Add the centre of rotation, and the vector (COR)
    Translation_vector = rs.VectorCreate(ULE2_S_points[2], ULE_S_points[2])
    Translation_vector = 0.75*Translation_vector    #vary this (0.75) for a higher translation
    COR = rs.MoveObject(LLE_S, 0.75*Translation_vector) #vary this (0.75) for the distance to the COR
    COR_points = AddCurvePoints(COR)
    COR_vector = rs.VectorCreate(COR_points[0], COR_points[2])

    # Deploy
    Deployed_Singleslot1 = rs.MoveObject(Complete_Singleslot, Translation_vector)
    Deployed_Singleslot = rs.RotateObject(Deployed_Singleslot1, COR_points[1], DeflectionAngle, COR_vector)

    # Delete objects
    rs.DeleteObjects([Aerofoil_Tip, Aerofoil_Root, TE_Curve, LE_Curve, Device_LE_Curve, Span_Root, Span_Tip, TE])
    rs.DeleteObjects([Surface_Camber,Upper_Surface, Lower_Surface])
    rs.DeleteObjects([LLE_ref, LLE2_ref, LLE_ref_SL, LLE2_ref_SL, LLE_S_interim, LLE2_S_interim, LLE2_S, LLE_S])
    rs.DeleteObjects([ULE, ULE2, ULE_S, ULE2_S])
    rs.DeleteObjects(Split_Surface)
    rs.DeleteObjects(Split_Surface2)
    rs.DeleteObjects(Device_Edges)
    rs.DeleteObjects([ULE_SL, ULE2_SL])
    rs.DeleteObjects(Device_Edges_Main)

    # Return
    return Deployed_Singleslot, Singleslot_SA

def doubleslot_geometry(surface_id, type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle):
    # Generate the domain over the wing lofter surface, and generate some important curves.
    Device_Edges_Main = rs.DuplicateEdgeCurves(surface_id)
    u0, u1, v0, v1 = domain_gen(surface_id)
    Aerofoil_Tip = rs.AddInterpCrvOnSrfUV(surface_id, [[u1,v0],[u1,v1]])
    Aerofoil_Root = rs.AddInterpCrvOnSrfUV(surface_id, [[u0,v0],[u0,v1]])
    TE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u0,v0],[u1,v0]])
    LE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u0, v1/2.0],[u1,v1/2.0]])

    # Device Curves (only for ref.)
    Device_LE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart, v1/2.0],[u1*SpanEnd,v1/2.0]])
    Span_Root = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart,v0],[u1*SpanStart,v1]])
    Span_Tip = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanEnd,v0],[u1*SpanEnd,v1]])

    # Correction factor, Chord2 (0.5) is applied
    Device_Span = rs.TrimSurface(surface_id, 0, (u1*SpanStart, u1*SpanEnd))
    Chord2 = 0.5

    # Trim to upper and Lower surfaces
    Upper_Surface = rs.TrimSurface(Device_Span, 1, (v0, v1*Chord2))
    Lower_Surface = rs.TrimSurface(Device_Span, 1, (v1*(1-Chord2), v1), True)

    # Generate the Trailing Edge
    TE = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart,v1],[u1*SpanEnd,v1]])

    # Generate mid surface that runs along the aerofoil camber
    Surface_Camber = rs.AddLoftSrf([Device_LE_Curve, TE], loft_type=1)
    a0, a1, b0, b1 = domain_gen(Surface_Camber)

    # Add the ULE curves (these are on the surface camber). Also ref. curves for the LLE. These will later on be projected on to the upper and lower surfaces, as required.
    Offset_ULE = 0.55*Chord*(a1-a0)  # For wings with severe sweep, this number should be decreased if long flaps are being used
    ULE = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*0.35*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*0.35*Offset_ULE,b1]]) #alter the 0.35, to make the LE smoother(higher)/more blunt(lower)
    ULE2 = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*Offset_ULE,b1]])
    ULE4 = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*0.7*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*0.7*Offset_ULE,b1]])
    LLE_ref = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*0.25*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*0.25*Offset_ULE,b1]])
    LLE2_ref = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*0.2*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*0.2*Offset_ULE,b1]])
    LLE3_ref = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*0.50*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*0.50*Offset_ULE,b1]])
    LLE4_ref = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*0.65*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*0.65*Offset_ULE,b1]])

    # Generate the start/mid/end points of the curves
    ULE_points = AddCurvePoints(ULE) 
    ULE2_points = AddCurvePoints(ULE2)
    ULE4_points = AddCurvePoints(ULE4) 
    LLE_ref_points = AddCurvePoints(LLE_ref) 
    LLE2_ref_points = AddCurvePoints(LLE2_ref) 
    LLE3_ref_points = AddCurvePoints(LLE3_ref) 
    LLE4_ref_points = AddCurvePoints(LLE4_ref) 
    TE_points = AddCurvePoints(TE)

    # Join the start/end points to create a straight line (SL)
    ULE_SL = rs.AddLine(ULE_points[0], ULE_points[2])
    ULE2_SL = rs.AddLine(ULE2_points[0], ULE2_points[2])
    ULE4_SL = rs.AddLine(ULE4_points[0], ULE4_points[2])
    LLE_ref_SL = rs.AddLine(LLE_ref_points[0], LLE_ref_points[2])
    LLE2_ref_SL = rs.AddLine(LLE2_ref_points[0], LLE2_ref_points[2])
    LLE3_ref_SL = rs.AddLine(LLE3_ref_points[0], LLE3_ref_points[2])
    LLE4_ref_SL = rs.AddLine(LLE4_ref_points[0], LLE4_ref_points[2])
    ULE_SL_points = AddCurvePoints(ULE_SL)

    # Project the SL curves to the respective surfaces (S-> to Surface)
    LLE_S_interim = rs.ExtendCurveLength(LLE_ref_SL, 2, 1, 0.1*(rs.CurveLength(LLE_ref_SL)))
    LLE_S = rs.ProjectCurveToSurface(LLE_S_interim, Lower_Surface, (0, 0, 1))
    LLE2_S_interim = rs.ExtendCurveLength(LLE2_ref_SL, 2, 1, 0.1*(rs.CurveLength(LLE2_ref_SL)))
    LLE2_S = rs.ProjectCurveToSurface(LLE2_S_interim, Lower_Surface, (0, 0, 1))
    LLE3_S_interim = rs.ExtendCurveLength(LLE3_ref_SL, 2, 1, 0.1*(rs.CurveLength(LLE3_ref_SL)))
    LLE3_S = rs.ProjectCurveToSurface(LLE3_S_interim, Lower_Surface, (0, 0, 1))
    LLE4_S_interim = rs.ExtendCurveLength(LLE4_ref_SL, 2, 1, 0.1*(rs.CurveLength(LLE4_ref_SL)))
    LLE4_S = rs.ProjectCurveToSurface(LLE4_S_interim, Lower_Surface, (0, 0, 1))
    rs.ReverseCurve(LLE_S)   #reverse curves on lower surface, so they are on the same direction as the rest, for the loft
    rs.ReverseCurve(LLE2_S)
    rs.ReverseCurve(LLE3_S)
    rs.ReverseCurve(LLE4_S)
    ULE_S_interim = rs.ExtendCurveLength(ULE_SL, 2, 0, 0.1*(rs.CurveLength(ULE_SL)))
    ULE2_S_interim = rs.ExtendCurveLength(ULE2_SL, 2, 0, 0.1*(rs.CurveLength(ULE2_SL)))
    ULE4_S_interim = rs.ExtendCurveLength(ULE4_SL, 2, 0, 0.1*(rs.CurveLength(ULE4_SL)))
    ULE_S = rs.ProjectCurveToSurface(ULE_S_interim, Upper_Surface, (0, 0, 1))
    ULE2_S = rs.ProjectCurveToSurface(ULE2_S_interim, Upper_Surface, (0, 0, 1))
    ULE3_S = rs.ProjectCurveToSurface(LLE3_S_interim, Upper_Surface, (0, 0, 1))
    ULE4_S = rs.ProjectCurveToSurface(ULE4_S_interim, Upper_Surface, (0, 0, 1))
    LLE2_S_points = AddCurvePoints(LLE2_S)
    LLE4_S_points = AddCurvePoints(LLE4_S)

    # Generate the curved surfaces
    LE_part1 = rs.AddLoftSrf([ULE2_S, ULE_S, LLE2_S, LLE_S], loft_type=1)
    TE_part1 = rs.AddLoftSrf([ULE2_S, ULE3_S, LLE3_S, LLE_S], loft_type=1)

    LE_part2 = rs.AddLoftSrf([ULE2_S, ULE4_S, LLE3_S, LLE4_S], loft_type=1)

    # Split the camber surface and lower surface where they intersect with the LE curved surface
    # Once this is done, identify the lowest area surface, since this is part of the singleslotted
    Split_Surface = rs.SplitBrep(Lower_Surface, LE_part2, True)
    Split_Areas = []
    for i, j in enumerate(Split_Surface):
        Split_SA = rs.SurfaceArea(Split_Surface[i])[0]
        list.append(Split_Areas, Split_SA)
    Min_Area = min(Split_Areas)
    Min_Area_index = [i for i, j in enumerate(Split_Areas) if j == Min_Area]

    Split_Surface2 = rs.SplitBrep(Upper_Surface, LE_part2, True)
    Split_Areas2 = []
    for i, j in enumerate(Split_Surface2):
        Split_SA2 = rs.SurfaceArea(Split_Surface2[i])[0]
        list.append(Split_Areas2, Split_SA2)
    Min_Area2 = min(Split_Areas2)
    Min_Area_index2 = [i for i, j in enumerate(Split_Areas2) if j == Min_Area2]

    # Join parts
    Joined_part1 = rs.JoinSurfaces([LE_part1, TE_part1], True)
    Joined_part2 = rs.JoinSurfaces([LE_part2, Split_Surface[Min_Area_index[0]], Split_Surface2[Min_Area_index2[0]]], True)

    # Add endplates
    Device_Edges_part1 = rs.DuplicateEdgeCurves(Joined_part1)
    # This check is required because some of the wings defined are not closed aerofoils and hence have 2 trailing edges.
    if len(Device_Edges_Main) < 4:
        End_Plate_1_1 = rs.AddEdgeSrf([Device_Edges_part1[0], Device_Edges_part1[4]])
        End_Plate_1_2 = rs.AddEdgeSrf([Device_Edges_part1[2], Device_Edges_part1[5]])
    else:
        End_Plate_1_1 = rs.AddEdgeSrf([Device_Edges_part1[0], Device_Edges_part1[2]])
        End_Plate_1_2 = rs.AddEdgeSrf([Device_Edges_part1[1], Device_Edges_part1[4]])

    Device_Edges_part2 = rs.DuplicateEdgeCurves(Joined_part2)
    if len(Device_Edges_Main) < 4:
        End_Plate_2_1 = rs.AddEdgeSrf([Device_Edges_part2[2], Device_Edges_part2[6], Device_Edges_part2[7]])
        End_Plate_2_2 = rs.AddEdgeSrf([Device_Edges_part2[0], Device_Edges_part2[5], Device_Edges_part2[8]])
    else:
        End_Plate_2_1 = rs.AddEdgeSrf([Device_Edges_part2[0], Device_Edges_part2[5], Device_Edges_part2[9]])
        End_Plate_2_2 = rs.AddEdgeSrf([Device_Edges_part2[2], Device_Edges_part2[6], Device_Edges_part2[8]])

    # Join the parts with endplates and obtain the SA
    Complete_part1 = rs.JoinSurfaces([Joined_part1, End_Plate_1_1, End_Plate_1_2], True)
    Part1_SA = rs.SurfaceArea(Complete_part1)[0]

    Complete_part2 = rs.JoinSurfaces([Joined_part2, End_Plate_2_1, End_Plate_2_2], True)
    Part2_SA = rs.SurfaceArea(Complete_part2)[0]

    # Add the centre of rotation, and the vector (COR)
    Translation_vector_1 = rs.VectorCreate(LLE4_S_points[2], LLE2_S_points[2])
    COR_1 = LLE4_S
    COR_1_points = AddCurvePoints(COR_1)
    COR_1_vector = rs.VectorCreate(COR_1_points[0], COR_1_points[2])

    # Deploy
    Deployed_part1_1 = rs.MoveObject(Complete_part1, Translation_vector_1)
    Deployed_part1 = rs.RotateObject(Deployed_part1_1, COR_1_points[1], (DeflectionAngle)/2.0, COR_1_vector)

    COR_2 = rs.MoveObject(LLE4_S, 0.85*Translation_vector_1)
    COR_2_points = AddCurvePoints(COR_2)
    COR_2_vector = rs.VectorCreate(COR_2_points[0], COR_2_points[2])

    Deployed_part2_1 = rs.MoveObject(Complete_part2, 1.4*Translation_vector_1)
    Deployed_part2 = rs.RotateObject(Deployed_part2_1, COR_2_points[1], DeflectionAngle, COR_2_vector)

    # Delete objects
    rs.DeleteObjects([Aerofoil_Tip, Aerofoil_Root, TE_Curve, LE_Curve, Device_LE_Curve, Span_Root, Span_Tip, TE])
    rs.DeleteObjects([Surface_Camber, Upper_Surface, Lower_Surface])
    rs.DeleteObjects([LLE_ref, LLE2_ref, LLE_ref_SL, LLE2_ref_SL, LLE_S_interim, LLE2_S_interim, LLE2_S, LLE_S])
    rs.DeleteObjects([LLE3_ref, LLE4_ref, LLE3_ref_SL, LLE4_ref_SL, LLE3_S_interim, LLE4_S_interim, LLE3_S, LLE4_S])
    rs.DeleteObjects([ULE, ULE2, ULE_S, ULE2_S])
    rs.DeleteObjects([ULE4, ULE4_SL, ULE4_S, ULE3_S])
    rs.DeleteObjects(Split_Surface)
    rs.DeleteObjects(Split_Surface2)
    rs.DeleteObjects(Device_Edges_part1)
    rs.DeleteObjects(Device_Edges_part2)
    rs.DeleteObjects(Device_Edges_Main)
    rs.DeleteObjects([ULE_SL, ULE2_SL])

    # Return
    return Deployed_part1, Part1_SA, Deployed_part2, Part2_SA

def tripleslot_geometry(surface_id, type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle):
    # Generate the domain over the wing lofter surface, and generate some important curves.
    Device_Edges_Main = rs.DuplicateEdgeCurves(surface_id)
    u0, u1, v0, v1 = domain_gen(surface_id)
    Aerofoil_Tip = rs.AddInterpCrvOnSrfUV(surface_id, [[u1,v0],[u1,v1]])
    Aerofoil_Root = rs.AddInterpCrvOnSrfUV(surface_id, [[u0,v0],[u0,v1]])
    TE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u0,v0],[u1,v0]])
    LE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u0, v1/2.0],[u1,v1/2.0]])

    # Device Curves (only for ref.)
    Device_LE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart, v1/2.0],[u1*SpanEnd,v1/2.0]])
    Span_Root = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart,v0],[u1*SpanStart,v1]])
    Span_Tip = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanEnd,v0],[u1*SpanEnd,v1]])

    # Correction factor, Chord2 (0.5) is applied
    Device_Span = rs.TrimSurface(surface_id, 0, (u1*SpanStart, u1*SpanEnd))
    Chord2 = 0.5

    # Trim to upper and Lower surfaces
    Upper_Surface = rs.TrimSurface(Device_Span, 1, (v0, v1*Chord2))
    Lower_Surface = rs.TrimSurface(Device_Span, 1, (v1*(1-Chord2), v1), True)

    # Generate the Trailing Edge
    TE = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart,v1],[u1*SpanEnd,v1]])

    # Generate mid surface that runs along the aerofoil camber
    Surface_Camber = rs.AddLoftSrf([Device_LE_Curve, TE], loft_type=1)
    a0, a1, b0, b1 = domain_gen(Surface_Camber)

    # Add the ULE curves (these are on the surface camber). Also ref. curves for the LLE. These will later on be projected on to the upper and lower surfaces, as required.
    Offset_ULE = 0.55*Chord*(a1-a0)  # For wings with severe sweep, this number should be decreased if long flaps are being used
    ULE = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*0.35*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*0.35*Offset_ULE,b1]]) #alter the 0.35, to make the LE smoother(higher)/more blunt(lower)
    ULE2 = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*Offset_ULE,b1]])
    ULE4 = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*0.7*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*0.7*Offset_ULE,b1]])

    LLE_ref = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*0.25*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*0.25*Offset_ULE,b1]])
    LLE2_ref = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*0.2*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*0.2*Offset_ULE,b1]])
    LLE3_ref = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*0.50*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*0.50*Offset_ULE,b1]])
    LLE4_ref = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord)+a1*0.65*Offset_ULE,b0],[a1*(1-Chord*Taper)+a1*0.65*Offset_ULE,b1]])

    #3rd element % factor
    Slot_Factor = 0.18  #approx. %chord of 3rd element
    LLE5_ref = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord*Slot_Factor)-a1*0.04*Offset_ULE,b0],[a1*(1-Chord*Slot_Factor*Taper)-a1*0.04*Offset_ULE,b1]])
    LLE6_ref = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord*Slot_Factor),b0],[a1*(1-Chord*Slot_Factor*Taper),b1]])
    LLE7_ref = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord*Slot_Factor)-a1*0.10*Offset_ULE,b0],[a1*(1-Chord*Slot_Factor*Taper)-a1*0.10*Offset_ULE,b1]])
    LLE8_ref = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord*Slot_Factor)-a1*0.15*Offset_ULE,b0],[a1*(1-Chord*Slot_Factor*Taper)-a1*0.15*Offset_ULE,b1]])
    ULE5 = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord*Slot_Factor)+a1*0.04*Offset_ULE,b0],[a1*(1-Chord*Slot_Factor*Taper)+a1*0.04*Offset_ULE,b1]])
    ULE6 = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(1-Chord*Slot_Factor)+a1*0.08*Offset_ULE,b0],[a1*(1-Chord*Slot_Factor*Taper)+a1*0.08*Offset_ULE,b1]])

    # Generate the start/mid/end points of the curves
    ULE_points = AddCurvePoints(ULE) 
    ULE2_points = AddCurvePoints(ULE2)
    ULE4_points = AddCurvePoints(ULE4) 
    ULE5_points = AddCurvePoints(ULE5) 
    ULE6_points = AddCurvePoints(ULE6) 
    LLE_ref_points = AddCurvePoints(LLE_ref) 
    LLE2_ref_points = AddCurvePoints(LLE2_ref) 
    LLE3_ref_points = AddCurvePoints(LLE3_ref) 
    LLE4_ref_points = AddCurvePoints(LLE4_ref) 
    LLE5_ref_points = AddCurvePoints(LLE5_ref)
    LLE6_ref_points = AddCurvePoints(LLE6_ref) 
    LLE7_ref_points = AddCurvePoints(LLE7_ref)
    LLE8_ref_points = AddCurvePoints(LLE8_ref)
    TE_points = AddCurvePoints(TE)

    # Join the start/end points to create a straight line (SL)
    ULE_SL = rs.AddLine(ULE_points[0], ULE_points[2])
    ULE2_SL = rs.AddLine(ULE2_points[0], ULE2_points[2])
    ULE4_SL = rs.AddLine(ULE4_points[0], ULE4_points[2])
    ULE5_SL = rs.AddLine(ULE5_points[0], ULE5_points[2])
    ULE6_SL = rs.AddLine(ULE6_points[0], ULE6_points[2])
    LLE_ref_SL = rs.AddLine(LLE_ref_points[0], LLE_ref_points[2])
    LLE2_ref_SL = rs.AddLine(LLE2_ref_points[0], LLE2_ref_points[2])
    LLE3_ref_SL = rs.AddLine(LLE3_ref_points[0], LLE3_ref_points[2])
    LLE4_ref_SL = rs.AddLine(LLE4_ref_points[0], LLE4_ref_points[2])
    LLE5_ref_SL = rs.AddLine(LLE5_ref_points[0], LLE5_ref_points[2])
    LLE6_ref_SL = rs.AddLine(LLE6_ref_points[0], LLE6_ref_points[2])
    LLE7_ref_SL = rs.AddLine(LLE7_ref_points[0], LLE7_ref_points[2])
    LLE8_ref_SL = rs.AddLine(LLE8_ref_points[0], LLE8_ref_points[2])
    ULE_SL_points = AddCurvePoints(ULE_SL)

    # Project the SL curves to the respective surfaces (S-> to Surface)
    LLE_S_interim = rs.ExtendCurveLength(LLE_ref_SL, 2, 1, 0.1*(rs.CurveLength(LLE_ref_SL)))
    LLE_S = rs.ProjectCurveToSurface(LLE_S_interim, Lower_Surface, (0, 0, 1))
    LLE2_S_interim = rs.ExtendCurveLength(LLE2_ref_SL, 2, 1, 0.1*(rs.CurveLength(LLE2_ref_SL)))
    LLE2_S = rs.ProjectCurveToSurface(LLE2_S_interim, Lower_Surface, (0, 0, 1))
    LLE3_S_interim = rs.ExtendCurveLength(LLE3_ref_SL, 2, 1, 0.1*(rs.CurveLength(LLE3_ref_SL)))
    LLE3_S = rs.ProjectCurveToSurface(LLE3_S_interim, Lower_Surface, (0, 0, 1))
    LLE4_S_interim = rs.ExtendCurveLength(LLE4_ref_SL, 2, 1, 0.1*(rs.CurveLength(LLE4_ref_SL)))
    LLE4_S = rs.ProjectCurveToSurface(LLE4_S_interim, Lower_Surface, (0, 0, 1))
    LLE5_S_interim = rs.ExtendCurveLength(LLE5_ref_SL, 2, 1, 0.1*(rs.CurveLength(LLE5_ref_SL)))
    LLE5_S = rs.ProjectCurveToSurface(LLE5_S_interim, Lower_Surface, (0, 0, 1))
    LLE6_S_interim = rs.ExtendCurveLength(LLE6_ref_SL, 2, 1, 0.1*(rs.CurveLength(LLE6_ref_SL)))
    LLE6_S = rs.ProjectCurveToSurface(LLE6_S_interim, Lower_Surface, (0, 0, 1))
    LLE7_S_interim = rs.ExtendCurveLength(LLE7_ref_SL, 2, 1, 0.1*(rs.CurveLength(LLE7_ref_SL)))
    LLE7_S = rs.ProjectCurveToSurface(LLE7_S_interim, Lower_Surface, (0, 0, 1))
    LLE8_S_interim = rs.ExtendCurveLength(LLE8_ref_SL, 2, 1, 0.1*(rs.CurveLength(LLE8_ref_SL)))
    LLE8_S = rs.ProjectCurveToSurface(LLE8_S_interim, Lower_Surface, (0, 0, 1))
    rs.ReverseCurve(LLE_S)   #reverse curves on lower surface, so they are on the same direction as the rest, for the loft
    rs.ReverseCurve(LLE2_S)
    rs.ReverseCurve(LLE3_S)
    rs.ReverseCurve(LLE4_S)
    rs.ReverseCurve(LLE5_S)
    rs.ReverseCurve(LLE6_S)
    rs.ReverseCurve(LLE7_S)
    rs.ReverseCurve(LLE8_S)
    ULE_S_interim = rs.ExtendCurveLength(ULE_SL, 2, 0, 0.1*(rs.CurveLength(ULE_SL)))
    ULE2_S_interim = rs.ExtendCurveLength(ULE2_SL, 2, 0, 0.1*(rs.CurveLength(ULE2_SL)))
    ULE4_S_interim = rs.ExtendCurveLength(ULE4_SL, 2, 0, 0.1*(rs.CurveLength(ULE4_SL)))
    ULE5_S_interim = rs.ExtendCurveLength(ULE5_SL, 2, 0, 0.1*(rs.CurveLength(ULE5_SL)))
    ULE6_S_interim = rs.ExtendCurveLength(ULE6_SL, 2, 0, 0.1*(rs.CurveLength(ULE6_SL)))
    ULE_S = rs.ProjectCurveToSurface(ULE_S_interim, Upper_Surface, (0, 0, 1))
    ULE2_S = rs.ProjectCurveToSurface(ULE2_S_interim, Upper_Surface, (0, 0, 1))
    ULE3_S = rs.ProjectCurveToSurface(LLE3_S_interim, Upper_Surface, (0, 0, 1))
    ULE4_S = rs.ProjectCurveToSurface(ULE4_S_interim, Upper_Surface, (0, 0, 1))
    ULE5_S = rs.ProjectCurveToSurface(ULE5_S_interim, Upper_Surface, (0, 0, 1))
    ULE6_S = rs.ProjectCurveToSurface(ULE6_S_interim, Upper_Surface, (0, 0, 1))
    Mid_Control_Line4_S = rs.ProjectCurveToSurface(LLE7_ref_SL, Surface_Camber, (0, 0, 1))
    LLE2_S_points = AddCurvePoints(LLE2_S)
    LLE4_S_points = AddCurvePoints(LLE4_S)

    # Generate the LE curved surface
    LE_part1 = rs.AddLoftSrf([ULE2_S, ULE_S, LLE2_S, LLE_S], loft_type=1)
    TE_part1 = rs.AddLoftSrf([ULE2_S, ULE3_S, LLE3_S, LLE_S], loft_type=1)

    LE_part2 = rs.AddLoftSrf([ULE2_S, ULE4_S, LLE3_S, LLE4_S], loft_type=1)
    TE_part2 = rs.AddLoftSrf([ULE6_S, ULE5_S, Mid_Control_Line4_S, LLE7_S, LLE5_S], loft_type=1)

    LE_part3 = rs.AddLoftSrf([ULE6_S, ULE5_S, Mid_Control_Line4_S, LLE5_S, LLE6_S], loft_type=1)

    # Split the camber surface and lower surface where they intersect with the LE curved surface
    # Once this is done, identify the surfaces that are part of the flap.
    Split_Surface = rs.SplitBrep(Lower_Surface, LE_part2, True)
    Split_Areas = []
    for i, j in enumerate(Split_Surface):
        Split_SA = rs.SurfaceArea(Split_Surface[i])[0]
        list.append(Split_Areas, Split_SA)
    Min_Area = min(Split_Areas)
    Min_Area_index = [i for i, j in enumerate(Split_Areas) if j == Min_Area]

    Split_Surface2 = rs.SplitBrep(Split_Surface[Min_Area_index[0]], TE_part2, True)
    Split_Areas2 = []
    for i, j in enumerate(Split_Surface2):
        Split_SA2 = rs.SurfaceArea(Split_Surface2[i])[0]
        list.append(Split_Areas2, Split_SA2)
    Min_Area2 = min(Split_Areas2)
    Min_Area_index2 = [i for i, j in enumerate(Split_Areas2) if j == Min_Area2]
    Max_Area2 = max(Split_Areas2)
    Max_Area_index2 = [i for i, j in enumerate(Split_Areas2) if j == Max_Area2]

    Split_Surface3 = rs.SplitBrep(Split_Surface2[Min_Area_index2[0]], LE_part3, True)
    Split_Areas3 = []
    for i, j in enumerate(Split_Surface3):
        Split_SA3 = rs.SurfaceArea(Split_Surface3[i])[0]
        list.append(Split_Areas3, Split_SA3)
    Max_Area3 = max(Split_Areas3)
    Max_Area_index3 = [i for i, j in enumerate(Split_Areas3) if j == Max_Area3]

    Split_Surface4 = rs.SplitBrep(Upper_Surface, LE_part2, True)
    Split_Areas4 = []
    for i, j in enumerate(Split_Surface4):
        Split_SA4 = rs.SurfaceArea(Split_Surface4[i])[0]
        list.append(Split_Areas4, Split_SA4)
    Min_Area4 = min(Split_Areas4)
    Min_Area_index4 = [i for i, j in enumerate(Split_Areas4) if j == Min_Area4]

    Split_Surface5 = rs.SplitBrep(Split_Surface4[Min_Area_index4[0]], LE_part3, True)
    Split_Areas5 = []
    for i, j in enumerate(Split_Surface5):
        Split_SA5 = rs.SurfaceArea(Split_Surface5[i])[0]
        list.append(Split_Areas5, Split_SA5)
    Min_Area5 = min(Split_Areas5)
    Min_Area_index5 = [i for i, j in enumerate(Split_Areas5) if j == Min_Area5]
    Max_Area5 = max(Split_Areas5)
    Max_Area_index5 = [i for i, j in enumerate(Split_Areas5) if j == Max_Area5]

    # Join parts
    Joined_part1 = rs.JoinSurfaces([LE_part1, TE_part1], True)
    Joined_part2 = rs.JoinSurfaces([LE_part2, Split_Surface2[Max_Area_index2[0]], Split_Surface5[Max_Area_index5[0]], TE_part2], True)
    Joined_part3 = rs.JoinSurfaces([LE_part3, Split_Surface3[Max_Area_index3[0]], Split_Surface5[Min_Area_index5[0]]], True)

    # Add endplates
    Device_Edges_part1 = rs.DuplicateEdgeCurves(Joined_part1)
    # This check is required because some of the wings defined are not closed aerofoils and hence have 2 trailing edges.
    if len(Device_Edges_Main) < 4:
        End_Plate_1_1 = rs.AddEdgeSrf([Device_Edges_part1[0], Device_Edges_part1[4]])
        End_Plate_1_2 = rs.AddEdgeSrf([Device_Edges_part1[2], Device_Edges_part1[5]])
    else:
        End_Plate_1_1 = rs.AddEdgeSrf([Device_Edges_part1[0], Device_Edges_part1[2]])
        End_Plate_1_2 = rs.AddEdgeSrf([Device_Edges_part1[1], Device_Edges_part1[4]])

    Device_Edges_part2 = rs.DuplicateEdgeCurves(Joined_part2)
    if len(Device_Edges_Main) < 4:
       End_Plate_2_1 = rs.AddEdgeSrf([Device_Edges_part2[0], Device_Edges_part2[5], Device_Edges_part2[6], Device_Edges_part2[8]])
       End_Plate_2_2 = rs.AddEdgeSrf([Device_Edges_part2[2], Device_Edges_part2[4], Device_Edges_part2[7], Device_Edges_part2[10]])
    else:
       End_Plate_2_1 = rs.AddEdgeSrf([Device_Edges_part2[0], Device_Edges_part2[5], Device_Edges_part2[6], Device_Edges_part2[8]])
       End_Plate_2_2 = rs.AddEdgeSrf([Device_Edges_part2[2], Device_Edges_part2[4], Device_Edges_part2[7], Device_Edges_part2[10]])

    Device_Edges_part3 = rs.DuplicateEdgeCurves(Joined_part3)
    if len(Device_Edges_Main) < 4:
        End_Plate_3_1 = rs.AddEdgeSrf([Device_Edges_part3[0], Device_Edges_part3[5], Device_Edges_part3[8]])
        End_Plate_3_2 = rs.AddEdgeSrf([Device_Edges_part3[2], Device_Edges_part3[6], Device_Edges_part3[7]])
    else:
        End_Plate_3_1 = rs.AddEdgeSrf([Device_Edges_part3[0], Device_Edges_part3[5], Device_Edges_part3[9]])
        End_Plate_3_2 = rs.AddEdgeSrf([Device_Edges_part3[2], Device_Edges_part3[6], Device_Edges_part3[8]])

    # Join the parts with endplates and obtain the SA
    Complete_part1 = rs.JoinSurfaces([Joined_part1, End_Plate_1_1, End_Plate_1_2], True)
    Part1_SA = rs.SurfaceArea(Complete_part1)[0]

    Complete_part2 = rs.JoinSurfaces([Joined_part2, End_Plate_2_1, End_Plate_2_2], True)
    Part2_SA = rs.SurfaceArea(Complete_part2)[0]

    Complete_part3 = rs.JoinSurfaces([Joined_part3, End_Plate_3_1, End_Plate_3_2], True)
    Part3_SA = rs.SurfaceArea(Complete_part3)[0]

    # Add the centre of rotation, and the vector (COR)
    Translation_vector_1 = rs.VectorCreate(LLE4_S_points[2], LLE2_S_points[2])
    COR_1 = LLE4_S
    COR_1_points = AddCurvePoints(COR_1)
    COR_1_vector = rs.VectorCreate(COR_1_points[0], COR_1_points[2])

    # Deploy
#    Deployed_part1_1 = rs.MoveObject(Complete_part1, Translation_vector_1)
#    Deployed_part1 = rs.RotateObject(Deployed_part1_1, COR_1_points[1], DeflectionAngle*0.5, COR_1_vector)
#
#    COR_2 = rs.MoveObject(LLE4_S, 0.85*Translation_vector_1)    #change 0.85 to change distance of the COR of midflap
#    COR_2_points = AddCurvePoints(COR_2)
#    COR_2_vector = rs.VectorCreate(COR_2_points[0], COR_2_points[2])
#
#    Deployed_part2_1 = rs.MoveObject(Complete_part2, 1.4*Translation_vector_1)  #change 1.4 to change length of translation of midflap
#    Deployed_part2 = rs.RotateObject(Deployed_part2_1, COR_2_points[1], DeflectionAngle, COR_2_vector)
#
#    COR_3_interim = rs.MoveObject(LLE5_S, 1.4*Translation_vector_1)     #change 1.4 to change distance of the COR of aft flap
#    COR_3 = rs.RotateObject(COR_3_interim, COR_2_points[1], DeflectionAngle, COR_2_vector)
#    COR_3_points = AddCurvePoints(COR_3)
#    COR_3_vector = rs.VectorCreate(COR_3_points[0], COR_3_points[2])
#
#    Deployed_part3_1 = rs.MoveObject(Complete_part3, 1.6*Translation_vector_1)      #change 1.6 to change length of translation of aft flap
#    Deployed_part3_2 = rs.RotateObject(Deployed_part3_1, COR_2_points[1], DeflectionAngle, COR_2_vector)
#    Deployed_part3 = rs.RotateObject(Deployed_part3_2, COR_3_points[1], DeflectionAngle*0.35, COR_3_vector) #change the 0.35 to alter the relative deflection between mid and aft flaps

    # Delete objects
    rs.DeleteObjects([Aerofoil_Tip, Aerofoil_Root, TE_Curve, LE_Curve, Device_LE_Curve, Span_Root, Span_Tip, TE])
    rs.DeleteObjects([Upper_Surface, Lower_Surface])
    rs.DeleteObjects([LLE_ref, LLE2_ref, LLE_ref_SL, LLE2_ref_SL, LLE_S_interim, LLE2_S_interim])
    rs.DeleteObjects([LLE3_ref, LLE4_ref, LLE3_ref_SL, LLE4_ref_SL, LLE3_S_interim, LLE4_S_interim])
    rs.DeleteObjects([LLE5_ref, LLE6_ref, LLE5_ref_SL, LLE6_ref_SL, LLE5_S_interim, LLE6_S_interim])
    rs.DeleteObjects([LLE7_ref, LLE8_ref, LLE7_ref_SL, LLE8_ref_SL, LLE7_S_interim, LLE8_S_interim])
    rs.DeleteObjects([ULE5, ULE6, ULE5_SL, ULE6_SL, ULE5_S_interim, ULE6_S_interim])
    rs.DeleteObjects([ULE, ULE2])
    #rs.DeleteObjects([Mid_Control_Line4_S])
    rs.DeleteObjects([ULE4, ULE4_SL])
    rs.DeleteObjects(Split_Surface)
    rs.DeleteObjects(Split_Surface2)
    rs.DeleteObjects(Split_Surface3)
    rs.DeleteObjects(Split_Surface4)
    rs.DeleteObjects(Split_Surface5)
    rs.DeleteObjects(Device_Edges_part1)
    rs.DeleteObjects(Device_Edges_part2)
    rs.DeleteObjects(Device_Edges_part3)
    rs.DeleteObjects(Device_Edges_Main)
    rs.DeleteObjects([ULE_SL, ULE2_SL])

    # Return
    #return Deployed_part1, Part1_SA, Deployed_part2, Part2_SA, Deployed_part3, Part3_SA

def kruger_geometry(surface_id, type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle):
    # Generate the domain over the wing lofter surface, and generate some important curves.
    u0, u1, v0, v1 = domain_gen(surface_id)
    Aerofoil_Tip = rs.AddInterpCrvOnSrfUV(surface_id, [[u1,v0],[u1,v1]])
    Aerofoil_Root = rs.AddInterpCrvOnSrfUV(surface_id, [[u0,v0],[u0,v1]])
    TE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u0,v0],[u1,v0]])
    LE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u0, v1/2.0],[u1,v1/2.0]])

    # Device Curves (only for ref.)
    Device_LE_Curve = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart, v1/2.0],[u1*SpanEnd,v1/2.0]])
    Span_Root = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart,v0],[u1*SpanStart,v1]])
    Span_Tip = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanEnd,v0],[u1*SpanEnd,v1]])

    # Correction factor, Chord2 (0.5) is applied
    Device_Span = rs.TrimSurface(surface_id, 0, (u1*SpanStart, u1*SpanEnd))
    Chord2 = 0.5
    FactorisedChord2 = 0.5*(v1-v0)*Chord

    # Trim to upper and Lower surfaces
    Upper_Surface = rs.TrimSurface(Device_Span, 1, (v0, v1*Chord2))
    Lower_Surface = rs.TrimSurface(Device_Span, 1, (v1*(1-Chord2), v1), True)

    # Generate the Leading and Trailing Edges
    ULE = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart,v1*Chord2],[u1*SpanEnd,v1*Chord2]])
    Span_TE = rs.AddInterpCrvOnSrfUV(surface_id, [[u1*SpanStart,v1],[u1*SpanEnd,v1]])

    # Generate mid surface that runs along the aerofoil camber
    Surface_Camber = rs.AddLoftSrf([Device_LE_Curve, Span_TE], loft_type=1)
    a0, a1, b0, b1 = domain_gen(Surface_Camber)

    # Add the control lines that define the curvature of the kruger. Distance is measured from the leading edge, and is a multiple of the chord.
    # Add the ULE curves (these are on the surface camber). These will later on be projected on to the respective surfaces, as required.
    TE = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(Chord),b0],[a1*(Chord*Taper),b1]])
    Mid_Control_Line = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(Chord)*0.5,b0],[a1*(Chord*Taper)*0.5,b1]])
    TE2 = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[a1*(Chord)+0.05*(a1-a0),b0],[a1*(Chord*Taper)+0.05*(a1-a0),b1]])
    ULE2 = rs.AddInterpCrvOnSrfUV(Surface_Camber, [[0.05*(a1-a0),b0],[0.05*(a1-a0),b1]])

    # Generate the start/mid/end points of the curves
    TE_points = AddCurvePoints(TE)
    TE2_points = AddCurvePoints(TE2)
    Mid_Control_Line_points = AddCurvePoints(Mid_Control_Line)

    # Join the start/end points to create a straight line (SL)
    TE_SL = rs.AddLine(TE_points[0], TE_points[2])
    TE2_SL = rs.AddLine(TE2_points[0], TE2_points[2])
    Mid_Control_Line_SL = rs.AddLine(Mid_Control_Line_points[0], Mid_Control_Line_points[2])

    # Project the SL curves to the respective surfaces (S-> to Surface)
    TE_interim = rs.ExtendCurveLength(TE_SL, 2, 1, 0.1*(rs.CurveLength(TE_SL)))
    TE2_interim = rs.ExtendCurveLength(TE2_SL, 2, 1, 0.1*(rs.CurveLength(TE2_SL)))
    Mid_Control_Line_interim = rs.ExtendCurveLength(Mid_Control_Line_SL, 2, 1, 0.1*(rs.CurveLength(Mid_Control_Line_SL)))

    Mid_Control_Line_S = rs.ProjectCurveToSurface(Mid_Control_Line_interim, Lower_Surface, (0, 0, 1))
    TE_S = rs.ProjectCurveToSurface(TE_interim, Lower_Surface, (0, 0, 1))
    TE2_S = rs.ProjectCurveToSurface(TE2_interim, Lower_Surface, (0, 0, 1))
    rs.ReverseCurve(TE_S)   #reverse curves on lower surface, so they are on the same direction as the rest, for the loft
    rs.ReverseCurve(TE2_S)
    rs.ReverseCurve(Mid_Control_Line_S)

    # Generate the inner curved surface
    Inner_Surface = rs.AddLoftSrf([ULE2, Mid_Control_Line_S, TE, TE2_S, TE_S], loft_type=1)

    # Split the camber surface and lower surface where they intersect with the inner curved surface
    # Once this is done, identify the lowest area surfaces, since this is part of the kruger
    Split_Surface = rs.SplitBrep(Lower_Surface, Inner_Surface, True)
    Split_Areas = []
    for i, j in enumerate(Split_Surface):
        Split_SA = rs.SurfaceArea(Split_Surface[i])[0]
        list.append(Split_Areas, Split_SA)
    Min_Area = min(Split_Areas)
    Min_Area_index = [i for i, j in enumerate(Split_Areas) if j == Min_Area]

    Split_Surface2 = rs.SplitBrep(Surface_Camber, Inner_Surface, True)
    Split_Areas2 = []
    for i, j in enumerate(Split_Surface2):
        Split_SA = rs.SurfaceArea(Split_Surface2[i])[0]
        list.append(Split_Areas2, Split_SA)
    Min_Area2 = min(Split_Areas2)
    Min_Area_index2 = [i for i, j in enumerate(Split_Areas2) if j == Min_Area2]

    Joined_Surfaces = rs.JoinSurfaces([Inner_Surface, Split_Surface[Min_Area_index[0]], Split_Surface2[Min_Area_index2[0]]], True)

    # Add endplates
    Device_Edges = rs.DuplicateEdgeCurves(Joined_Surfaces)
    End_Plate_1 = rs.AddPlanarSrf([Device_Edges[0], Device_Edges[6], Device_Edges[7]])
    End_Plate_2 = rs.AddPlanarSrf([Device_Edges[2], Device_Edges[5], Device_Edges[8]])

    # Join the kruger flap with endplates and obtain the SA
    Complete_Kruger = rs.JoinSurfaces([Joined_Surfaces, End_Plate_1, End_Plate_2], True)
    Kruger_SA = rs.SurfaceArea(Complete_Kruger)[0]

    # Add the centre of rotation, and the vector (COR)
    COR = ULE
    COR_points = AddCurvePoints(COR)
    COR_vector = rs.VectorCreate(COR_points[0], COR_points[2])

    # Deploy
    Deployed_Kruger = rs.RotateObject(Complete_Kruger, COR_points[1], DeflectionAngle, COR_vector)

    # Delete objects
    rs.DeleteObjects([Aerofoil_Tip, Aerofoil_Root, TE_Curve, LE_Curve, Device_LE_Curve, Span_Root, Span_Tip])
    rs.DeleteObjects([TE, TE_SL, TE_interim, TE_S, TE2, TE2_SL, TE2_interim, TE2_S])
    rs.DeleteObjects([ULE, ULE2, Span_TE])
    rs.DeleteObjects([Upper_Surface, Lower_Surface, Surface_Camber])
    rs.DeleteObjects(Split_Surface)
    rs.DeleteObjects(Split_Surface2)
    rs.DeleteObjects([Mid_Control_Line, Mid_Control_Line_SL, Mid_Control_Line_interim, Mid_Control_Line_S])
    rs.DeleteObjects(Device_Edges)

    # Return
    return Deployed_Kruger, Kruger_SA

def AddHighLiftDevices(surface_id, devices):
    #Function to assist in submitting all the high lift device configurations as a single list
    for device in devices:
        [type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle] = device
        type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle = param_check(type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle)
        if type == "slat":
            slat_geometry(surface_id, type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle)
        elif type == "aileron":
            aileron_geometry(surface_id, type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle)
        elif type == "simpleflap":
            Deployed_Aileron, Aileron_SA, OversizedAileron, OversizedAileronUp, OversizedAileronDn, CutBrick = aileron_geometry(surface_id, type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle)
            return Deployed_Aileron, Aileron_SA, OversizedAileron, OversizedAileronUp, OversizedAileronDn, CutBrick
        elif type == "fowlerflap":
            fowler_geometry(surface_id, type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle)
        elif type == "splitflap":
            splitflap_geometry(surface_id, type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle)
        elif type == "spoiler":
            spoiler_geometry(surface_id, type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle)
        elif type == "singleslot":
            singleslot_geometry(surface_id, type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle)
        elif type == "doubleslot":
            doubleslot_geometry(surface_id, type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle)
        elif type == "tripleslot":
            tripleslot_geometry(surface_id, type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle)
        elif type == "kruger":
            kruger_geometry(surface_id, type, SpanStart, SpanEnd, Chord, Taper, DeflectionAngle)