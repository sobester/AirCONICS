# HARPOONCFG.PY ==============================================================
# This module contains the definitions of various elements that go to make
# up a Haroon cfg meshing file.
#
# Assumes that an AirConics Rhino generated STL is available and goes on to
# create a standard six sided Harpoon bounding box and three part airframe
# surface (named rhinoceros_binary_stl___...). Also allows six sided
# refinment zones to be added for wing tips, trailing edges, etc.
#
# A.J. Keane, January 2016
# ==========================================================================

from __future__ import division
import rhinoscriptsyntax as rs
import math


def harpoonData(fo, FilNam, DesiredyPlus, Chord, rho_C, V_T, Visc_C, RefLev, FirstCellHtMult):
    
    ReynoldsNo = rho_C * V_T * Chord / Visc_C
    Cf = math.pow(2 * math.log10 ( ReynoldsNo ) -0.65 , -2.3)
    TauWall = Cf * 0.5 * rho_C * V_T * V_T
    uStar = math.sqrt ( TauWall / rho_C )
    FirstCellHt = DesiredyPlus * Visc_C / (rho_C * uStar)
    BaseLev = FirstCellHt * FirstCellHtMult
    
    fo.write('%s %s\n' % ("import stl", FilNam))
    # write a second file to append TE wake
    fo.write('%s %s\n' % ("append stl", FilNam))
    fo.write('%s %s \n' % ("baselev ", BaseLev))
    fo.write('%s\n' % ("farfield body"))
    # use a farfield of -0.5 for ymin if a half body on the +ve side
    # or -0.4995 if wing has dihedral to ensure closure on CLine
    # is to be meshed rather than the whole configuration (symmetry)
    # but ensure body reaches the centreplane correctly
    # use zmin/max of 20 for an aircraft but 50 for just a wing or airfoil
    fo.write('%s\n' % ("farfield xmin -15.0"))
    # fo.write('%s\n' % ("farfield ymin -5.0"))
    fo.write('%s\n' % ("farfield ymin -0.4995"))
    fo.write('%s\n' % ("farfield zmin -50.0"))
    fo.write('%s\n' % ("farfield xmax 20.0"))
    fo.write('%s\n' % ("farfield ymax 5.0"))
    fo.write('%s\n' % ("farfield zmax 50.0"))
    fo.write('%s\n' % ("wlevel xmax -10"))
    fo.write('%s\n' % ("setbc farfield_minx velocity-inlet"))
    fo.write('%s\n' % ("setbc farfield_miny symmetry"))
    fo.write('%s\n' % ("setbc farfield_minz velocity-inlet"))
    fo.write('%s\n' % ("setbc farfield_maxx pressure-outlet"))
    fo.write('%s\n' % ("setbc farfield_maxy symmetry"))
    fo.write('%s\n' % ("setbc farfield_maxz pressure-outlet"))

def harpoonClose(fo, FilNam, DesiredyPlus, Chord, rho_C, V_T, Visc_C, RefLev, FirstCellHtMult):
    
    ReynoldsNo = rho_C * V_T * Chord / Visc_C
    Cf = math.pow(2 * math.log10 ( ReynoldsNo ) -0.65 , -2.3)
    TauWall = Cf * 0.5 * rho_C * V_T * V_T
    uStar = math.sqrt ( TauWall / rho_C )
    FirstCellHt = DesiredyPlus * Visc_C / (rho_C * uStar)
    BaseLev = FirstCellHt * FirstCellHtMult
    
    fo.write('%s \n' % ("**MESH METHODS**"))
    fo.write('%s \n' % ("type hex"))
    fo.write('%s \n' % ("expand slow"))
    fo.write('%s \n' % ("mesh external"))
    fo.write('%s \n' % ("stretch 2.0 1.0 1.0"))
    # NB fluent guidlines for fine meshes are an inflation factor of 1.15 or 1.2, rather that the DPW value of 1.25
    # this means that typically 20 or even 25 layers may be needed to get from the wall to the rest of the mesh.
    fo.write('%s %s %s\n' % ("layer 1 ", FirstCellHt, " 25 0 1.15 0"))
    fo.write('%s %s %s\n' % ("layer 2 ", FirstCellHt, " 25 0 1.15 0"))
    # next line needed if we have a separate wake mesh
    fo.write('%s \n' % ("**SINGLE LEVEL"))
    fo.write('%s \n' % ("level 5"))
    fo.write('%s \n' % ("plevel 1 -1 -1 0   !Decode"))
    fo.write('%s \n' % ("llevel 1 -1"))
    # next two lines needed if we have a separate wake mesh
    fo.write('%s \n' % ("plevel 2 -1 -1 0   !Decode"))
    fo.write('%s \n' % ("llevel 2 -1"))
    fo.write('%s %s\n' % ("**export fluent volume",FilNam))

def harpoonRefZone(fo, TEpoint1, TEpoint2, RLevel, RLength, RThick, UTaper, LTaper):
    # User-defined function describing the CFD refinement zone as a function
    # typically of two trailing edge coordinates
    ZuTaper = RLength * math.sin(math.radians(UTaper))
    ZlTaper = RLength * math.sin(math.radians(LTaper))
    
    fo.write('%s \n' % ("refine"))
    fo.write('%s %s \n' % ("2", RLevel))
    fo.write('%s %s %s \n' % ((TEpoint1[0]+RLength*0.9), TEpoint1[1], (TEpoint1[2]+RThick/2.0+ZuTaper)))
    fo.write('%s %s %s \n' % ((TEpoint1[0]-RLength*0.1), TEpoint1[1], (TEpoint1[2]+RThick/2.0)))
    fo.write('%s %s %s \n' % ((TEpoint1[0]-RLength*0.1), TEpoint1[1], (TEpoint1[2]-RThick/2.0)))
    fo.write('%s %s %s \n' % ((TEpoint1[0]+RLength*0.9), TEpoint1[1], (TEpoint1[2]-RThick/2.0-ZlTaper)))
    fo.write('%s %s %s \n' % ((TEpoint2[0]+RLength*0.9), TEpoint2[1], (TEpoint2[2]+RThick/2.0+ZuTaper)))
    fo.write('%s %s %s \n' % ((TEpoint2[0]-RLength*0.1), TEpoint2[1], (TEpoint2[2]+RThick/2.0)))
    fo.write('%s %s %s \n' % ((TEpoint2[0]-RLength*0.1), TEpoint2[1], (TEpoint2[2]-RThick/2.0)))
    fo.write('%s %s %s \n' % ((TEpoint2[0]+RLength*0.9), TEpoint2[1], (TEpoint2[2]-RThick/2.0-ZlTaper)))
    return 0

if __name__ == "__main__":
# now write out matching Harpoon cfg file
    
    Chord=0.25 # in metres
    V_T=16.00 # in m/s
    rho_C=1.2107
    Visc_C=1.785E-05
    
    # meshing details for Harpoon (if using symmetry edit fafield ymin to be -0.5 in body units)
    # note that Fluent scripts should include y+ adaptation as well
    # also turn off BL mesh for coarse model as Fluent will adapt back what is required
    DesiredyPlus = 1.0 # use 50 for coarse mesh and 1.0 for fine one
    RefLev = 3 # use 3 for coarse mesh and 3 for fine one (nb tailplane outer zones are one level coarser)
    FirstCellHtMult = 5000 # multiply by 750 for coarse mesh and 5000 for fine one
    
    fo = open("harpoon_meshing_file.cfg", "w")
    harpoonData(fo, "C:\\Users\\Andy\\Documents\\ajk\\UAV design\\Decode1.stl", DesiredyPlus, Chord, rho_C, V_T, Visc_C, RefLev, FirstCellHtMult)
    harpoonRefZone(fo, [0.0, 0.2, 0.5], [0.0, 0.25, 0.5], RefLev, Chord*3, 0.03, 6.0, 2.0)
    harpoonRefZone(fo, [0.0, -0.2, 0.5], [0.0, -0.25, 0.5], RefLev, Chord*3, 0.03, 6.0, 2.0)
    harpoonClose(fo, "C:\\Users\\Andy\\Documents\\ajk\\UAV design\\Decode1.cas", DesiredyPlus, Chord, rho_C, V_T, Visc_C, RefLev, FirstCellHtMult)
    fo.close()