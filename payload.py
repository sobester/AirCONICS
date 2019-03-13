# PAYLOAD POD GEOMETRY GENERATION ==============================================
# This is a dependency of appendix.py.

from __future__ import division
import math, rhinoscriptsyntax as rs
import primitives 

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

#========= END OF THE DEFINITION OF THE POD ====================================

