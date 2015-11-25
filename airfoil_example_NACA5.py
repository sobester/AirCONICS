# Example script for generating a 5-digit NACA airfoil object. 
# ==============================================================================
# AirCONICS
# Aircraft CONfiguration through Integrated Cross-disciplinary Scripting 
# version 0.2.0
# Andras Sobester, 2015.
# Bug reports to a.sobester@soton.ac.uk or @ASobester please.
# ==============================================================================

import primitives, airconics_setup, AirCONICStools as act

#===============================================================================
# NACA 5-digit parametric airfoil
# Leading edge point in origin, unit chord along x axis, 10 degree tilt.
# If a smooth surface is more important than a strict adherence to
# the original NACA geometry, set SmoothingPasses to a non-zero value (typically
# 1 or 2).
# 
# NOTE: you may have to hit 'zoom extents' to bring the airfoil into view.
#===============================================================================

LEPoint = (0, 0, 0)
ChordLength = 1
Rotation = 10
Twist = 0

# Instantiate class to set up a generic airfoil with these basic parameters
# With EnforceSharpTE set to True, the trailing edge of the airfoil is closed
# When set to False, the original 5-digit NACA foil is produced
Af = primitives.Airfoil(LEPoint, ChordLength, Rotation, Twist, EnforceSharpTE = False)

SmoothingPasses = 0

# Add airfoil curve to document and retrieve handles to it and its chord
# - in this case NACA23012, with DesignLiftCoefficient = 0.3, 
# MaxCamberLocFracChord = 0.15 and MaxThicknessPercChord = 12
AfCurve,Chrd = primitives.Airfoil.AddNACA5(Af, 0.3, 0.15, 12, SmoothingPasses)


# Optional: uncomment this to close the trailing edge with a line. For this
# example the thickness of the trailing edge is 0.25% chord
# act.AddTEtoOpenAirfoil(AfCurve)