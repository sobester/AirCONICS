# Example script for replicating the wing geometry of a Jetstream 31. Also, an
# example of specifying high level target parameters of the wing, such as aspect
# ratio and area.
# ==============================================================================
# AirCONICS
# Aircraft CONfiguration through Integrated Cross-disciplinary Scripting 
# version 0.2.0
# Andras Sobester, 2015.
# Bug reports to a.sobester@soton.ac.uk or @ASobester please.
# ==============================================================================

from __future__ import division
import rhinoscriptsyntax as rs
import math
import primitives 
import airconics_setup
import liftingsurface
import AirCONICStools as act

#===============================================================================
# EXAMPLE - British Aerospace Jetstream 31 wing
# The BAe Jetstream 31 is a commuter class turboprop with an empty mass of 4990kg
# and a MTOW of 7059kg. It is powered by two Garrett TPE 331 engines and carries
# 19 passengers.
#
# Data from Cooke, A. K., "A Simulation Model of the NFLC Jetstream 31", Cranfield 
# College of Aeronautics Report No. 0402, May 2006.
#
# Wing geometry data:
#-------------------------------------------------------------------------------
# Span: 15.85m
# Wing planform gross area: 25.08m^2
# Exposed planform area: 20.94m^2
# Root datum chord (on centreline): 2.38m
# Twist: -2deg (washout)
# Airfoils: root NACA63A418, tip NACA63A412
# Aspect ratio: 10.0
# Taper ratio: tip/centreline: 0.333
# Sweep on 30% chordline: 0
# Leading edge sweep: ~3deg
# Dihedral (from one fuselage radius away from centre): 7deg
#===============================================================================

def myDihedralFunctionJetstream(Epsilon):
    # User-defined function describing the variation of dihedral as a function
    # of the leading edge coordinate
    return 7

def myTwistFunctionJetstream(Epsilon):
    # User-defined function describing the variation of twist as a function
    # of the leading edge coordinate
    RootTwist = 0
    TipTwist  = -2
    return RootTwist + Epsilon*TipTwist

def myChordFunctionJetstream(Epsilon):
    # User-defined function describing the variation of chord as a function of 
    # the leading edge coordinate
    
    ChordLengths = [1, 0.333]
    EpsArray = [0, 1]
    
    f = act.linear_interpolation(EpsArray, ChordLengths)
    return f(Epsilon)


def myAirfoilFunctionJetstream(Epsilon, LEPoint, ChordFunct, ChordFactor, DihedralFunct, TwistFunct):
    # Defines the variation of cross section as a function of Epsilon
    
    AirfoilChordLength = (ChordFactor*ChordFunct(Epsilon))/math.cos(math.radians(TwistFunct(Epsilon)))

    Af = primitives.Airfoil(LEPoint, AirfoilChordLength, DihedralFunct(Epsilon), TwistFunct(Epsilon), 
    airconics_setup.SeligPath)
    
    if Epsilon==0:
        # *Important note - if you had anything other than a simply tapered wing
        # (like the one of the J31), you would have to specify a smooth variation
        # of the airfoil section, not just the end points like done here.
        # Root airfoil 
        Airf, Chrd = primitives.Airfoil.AddAirfoilFromSeligFile(Af, 'naca63a418')
    else:
        # Tip airfoil
        Airf, Chrd = primitives.Airfoil.AddAirfoilFromSeligFile(Af, 'naca63a412')

    return Airf, Chrd
    
    
def mySweepAngleFunctionJetstream(Epsilon):
    # User-defined function describing the variation of sweep angle as a function
    # of the leading edge coordinate
    return 3



if __name__ == "__main__":
    # Wing apex location
    P = (0,0,0)
    
    LooseSurf = 1
    SegmentNo = 1
    
    Wing = liftingsurface.LiftingSurface(P, mySweepAngleFunctionJetstream, myDihedralFunctionJetstream, myTwistFunctionJetstream, myChordFunctionJetstream, myAirfoilFunctionJetstream, LooseSurf, SegmentNo)
    
    # Specify the desired aspect ratio and span
    Wing.TargetAspectRatio = 10.0
    Wing.wTargetAspectRatio = 1
    Wing.TargetArea = 25.08
    Wing.wTargetArea = 1
    
    # The optimization should yield these values:
    #ChordFactor = 0.298
    #ScaleFactor = 7.95
    
    # When OptimizeChordScale=1, ChordFactor and ScaleFactor are used simply as the 
    # starting point of the optimization
    ChordFactor = 0.1
    ScaleFactor = 10
    OptimizeChordScale = 1
    
    # Turn off screen refresh - redrawing the geometry for each optimization iteration
    # can be time-consuming
    rs.EnableRedraw(False)
    
    Wing.GenerateLiftingSurface(ChordFactor, ScaleFactor, OptimizeChordScale)
    
    rs.EnableRedraw()