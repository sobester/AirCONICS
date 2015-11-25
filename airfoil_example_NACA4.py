# Example script for generating a 4-digit NACA airfoil object. 
# ==============================================================================
# AirCONICS
# Aircraft CONfiguration through Integrated Cross-disciplinary Scripting 
# version 0.2.0
# Andras Sobester, 2015.
# Bug reports to a.sobester@soton.ac.uk or @ASobester please.
# ==============================================================================

import primitives, airconics_setup, AirCONICStools as act

#===============================================================================
# NACA 4-digit parametric airfoil
# Leading edge point in origin, unit chord along x axis, 10 degree washout angle.
# If a smooth surface is more important than a strict adherence to
# the original NACA geometry, set SmoothingPasses to a non-zero value (typically
# 1 or 2).
# 
# NOTE: you may have to hit 'zoom extents' to bring the airfoil into view.
#===============================================================================

LEPoint = (0, 0, 0)
ChordLength = 1
Rotation = 0
Twist = 10

# Instantiate class to set up a generic airfoil with these basic parameters
Af = primitives.Airfoil(LEPoint, ChordLength, Rotation, Twist, EnforceSharpTE = False)

SmoothingPasses = 1

# Add NACA2212 airfoil curve to document and retrieve handles to it and its chord
# MaxCamberPercChord = 2, MaxCamberLocTenthChord = 2, MaxThicknessPercChord = 12
AfCurve,Chrd = primitives.Airfoil.AddNACA4(Af, 2, 2, 12, SmoothingPasses)

# Optional: uncomment this to close the trailing edge with a line.
# act.AddTEtoOpenAirfoil(AfCurve)