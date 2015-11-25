# Example script for generating a UAV box wing 
# ==============================================================================
# AirCONICS
# Aircraft CONfiguration through Integrated Cross-disciplinary Scripting 
# version 0.2
# Andras Sobester, 2015.
# Bug reports to a.sobester@soton.ac.uk or @ASobester please.
# ==============================================================================

import math
import primitives, airconics_setup, liftingsurface, AirCONICStools as act


# Definition of the variation of spanwise parameters. Key here is the variation
# dihedral (which defines how the wing 'folds back') and the variation of the 
# airfoil section (in sync with the variation of the fold to ensure the smooth
# reversal of the camber at the same time). See Section 9.3.1 (page 183) of the
# book for a detailed discussion of this example. 

def myDihedralFunctionBoxWing(Epsilon):
    # User-defined function describing the variation of dihedral as a function
    # of the leading edge coordinate
    D1 = 0
    D2 = 180
    Transition1 = 0.45
    Transition2 = 0.55
    
    if Epsilon < Transition1:
        return D1
    elif Epsilon > Transition2:
        return D2
    else:
        return D1 + ((Epsilon - Transition1)/(Transition2 - Transition1))*(D2-D1)

def myTwistFunctionBoxWing(Epsilon):
    # User-defined function describing the variation of twist as a function
    # of the leading edge coordinate
    RootTwist = 0
    TipTwist  = 0
    return RootTwist + Epsilon*TipTwist

def myChordFunctionBoxWing(Epsilon):
    # User-defined function describing the variation of chord as a function of 
    # the leading edge coordinate
    return 1

def myAirfoilFunctionBoxWing(Epsilon, LEPoint, ChordFunct, ChordFactor, DihedralFunct, TwistFunct):
    # Defines the variation of cross section as a function of Epsilon
    
    AirfoilChordLength = (ChordFactor*ChordFunct(Epsilon))/math.cos(math.radians(TwistFunct(Epsilon)))

    Af = primitives.Airfoil(LEPoint, AirfoilChordLength, DihedralFunct(Epsilon), TwistFunct(Epsilon))

    SmoothingPasses = 1

    Camber1 = 5.0
    Camber2 = -5.0
    Transition1 = 0.45
    Transition2 = 0.55
    
    if Epsilon < Transition1:
        Camber = Camber1
    elif Epsilon > Transition2:
        Camber = Camber2
    else:
        Camber =  Camber1 + ((Epsilon - Transition1)/(Transition2 - Transition1))*(Camber2-Camber1)

    Airf, Chrd = primitives.Airfoil.AddNACA4(Af, Camber, 3, 10, SmoothingPasses)

    return Airf, Chrd


def mySweepAngleFunctionBoxWing(Epsilon):
    # User-defined function describing the variation of sweep angle as a function
    # of the leading edge coordinate
    
    S1 = 25
    S2 = -25
    Boundary1 = 0.45
    Boundary2 = 0.55
    
    if Epsilon < Boundary1:
        return S1
    elif Epsilon > Boundary2:
        return S2
    else:
        return S1 + ((Epsilon - Boundary1)/(Boundary2 - Boundary1))*(S2-S1)

if __name__ == '__main__':
    # Script for generating the box wing geometry
    
    P = (0,0,0)
    LooseSurf = 1
    SegmentNo = 101
    
    # The wing tip is turned off, as a box wing has no exposed tip
    Wing = liftingsurface.LiftingSurface(P, mySweepAngleFunctionBoxWing, myDihedralFunctionBoxWing, myTwistFunctionBoxWing, myChordFunctionBoxWing, myAirfoilFunctionBoxWing, LooseSurf, SegmentNo, TipRequired = False)
    
    ChordFactor = 0.1
    ScaleFactor = 20
    
    Wing.GenerateLiftingSurface(ChordFactor, ScaleFactor)
