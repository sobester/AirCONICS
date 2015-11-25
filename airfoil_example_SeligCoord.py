# Example script for generating an airfoil object given a set of coordinates spe-
# cified in a Selig format. The file airconics_setup.py must contain the path to
# the folder containing the Selig-formatted airfoil coordinate library.
# ==============================================================================
# AirCONICS
# Aircraft CONfiguration through Integrated Cross-disciplinary Scripting 
# version 0.2.0
# Andras Sobester, 2015.
# Bug reports to a.sobester@soton.ac.uk or @ASobester please.
# ==============================================================================

import primitives, airconics_setup

#===============================================================================
# Example: airfoil from Selig-formatted coordinate file. Leading edge point
# in origin, unit chord along x axis, no rotation around the x or y axes. 
# Coordinates for Drela DAE11 low Reynolds number section, two smoothing 
# iterations.
#===============================================================================
LEPoint = (0, 0, 0)
ChordLength = 1
Rotation = 0
Twist = 0

# Instantiate class to set up a generic airfoil with these basic parameters
Af = primitives.Airfoil(LEPoint,ChordLength, Rotation, Twist, airconics_setup.SeligPath)

# Name of the file containing the airfoil coordinates + smoothing
AirfoilSeligName = 'dae11'
SmoothingPasses = 1

# Add airfoil curve to document, and retrieve handles to it and its chord
AfCurve,Chrd = primitives.Airfoil.AddAirfoilFromSeligFile(Af, AirfoilSeligName, SmoothingPasses)