# Example illustrating the construction of a turbofan nacelle geometry as a wing
# with linearly varying dihedral (from 0 to 360 degrees).
# IMPORTANT:
# This is merely an illustration of the way in which dihedral definition functions
# can be used to turn a lifting surface into unusual geometries - revolving an
# airfoil section around an axis is a more flexible way of generating a nacelle.
# Use the TurbofanNacelle method in engine.py for this.
# ==============================================================================
# AirCONICS
# Aircraft CONfiguration through Integrated Cross-disciplinary Scripting 
# version 0.2
# Andras Sobester, 2015.
# Bug reports to a.sobester@soton.ac.uk or @ASobester please.
# ==============================================================================

from __future__ import division
import math
import primitives, airconics_setup, liftingsurface, AirCONICStools as act

# User-defined functions describing the spanwise variations of the various
# lifting surface parameters (here 'spanwise' is not a very intutitve term,
# as the variations are actually circumferential, due to the way in which
# the 'dihedral' is defined below)

def myDihedralFunctionNacelle(Epsilon):
    return -360*Epsilon

def myTwistFunctionNacelle(Epsilon):
    return 0

def myChordFunctionNacelle(Epsilon):
    return 1

def myAirfoilFunctionNacelle(Epsilon, LEPoint, ChordFunct, ChordFactor, DihedralFunct, TwistFunct):
    AirfoilChordLength = (ChordFactor*ChordFunct(Epsilon))/math.cos(math.radians(TwistFunct(Epsilon)))
    Af = primitives.Airfoil(LEPoint, AirfoilChordLength, DihedralFunct(Epsilon), TwistFunct(Epsilon),airconics_setup.SeligPath)
    SmoothingPasses = 1
    Airf, Chrd = primitives.Airfoil.AddAirfoilFromSeligFile(Af, 'sc20610', SmoothingPasses)
    return Airf, Chrd

def mySweepAngleFunctionNacelle(Epsilon):
    return 0


if __name__ == "__main__":
    #The actual script for generating the nacelle and positioning it in a particular
    #point in space (defined by P)
    
    P = (17.75,9.87,0.65)
    
    LooseSurf = 1
    SegmentNo = 10
    
    Wing = liftingsurface.LiftingSurface(P, mySweepAngleFunctionNacelle, myDihedralFunctionNacelle, myTwistFunctionNacelle, myChordFunctionNacelle, myAirfoilFunctionNacelle, LooseSurf, SegmentNo, TipRequired = False)
    
    ChordFactor = 0.59
    ScaleFactor = 9.7
    
    Wing.GenerateLiftingSurface(ChordFactor, ScaleFactor)