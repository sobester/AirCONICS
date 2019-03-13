# XFLR5XML.PY ==============================================================
# This module contains the definitions of various elements that go to make
# up an XFLR5 xml plane file.
#
# Assumes that an AirConics Rhino generated STL is available and goes on to
# create a standard plane model with main wing, elevator, one or two fins and
# a single central fuselage body
#
# A.J. Keane, March 2016
# ==========================================================================

from __future__ import division
import rhinoscriptsyntax as rs
import math


def XFLR5Data(fo, WingName):
    
    fo.write('%s\n' % ("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"))
    fo.write('%s\n' % ("<!DOCTYPE explane>"))
    fo.write('%s\n' % ("<explane version=\"1.0\">"))
    fo.write('%s\n' % ("<Units>"))
    fo.write('%s\n' % ("<length_unit_to_meter>0.001</length_unit_to_meter>"))
    fo.write('%s\n' % ("<mass_unit_to_kg>1</mass_unit_to_kg>"))
    fo.write('%s\n' % ("</Units>"))
    fo.write('%s\n' % ("<Plane>"))
    fo.write('%s%s%s\n' % ("<Name>",WingName,"</Name>"))
    fo.write('%s\n' % ("<Description></Description>"))
    fo.write('%s\n' % ("<Inertia/>"))

def XFLR5Close(fo):
    
    fo.write('%s\n' % ("</Plane>"))
    fo.write('%s\n' % ("</explane>"))

def XFLR5MainWing(fo, Posn, NSects, y_position, Chord, xOffset, Dihedral, Twist, Foil):
    fo.write('%s\n' % ("<wing>"))
    fo.write('%s\n' % ("<Name>MainWing</Name>"))
    fo.write('%s\n' % ("<Type>MAINWING</Type>"))
    fo.write('%s\n' % ("<Color>"))
    fo.write('%s\n' % ("<red>157</red>"))
    fo.write('%s\n' % ("<green>100</green>"))
    fo.write('%s\n' % ("<blue>214</blue>"))
    fo.write('%s\n' % ("<alpha>255</alpha>"))
    fo.write('%s\n' % ("</Color>"))
    fo.write('%s\n' % ("<Description>Main_Wing</Description>"))
    fo.write('%s %s %s\n' % ("<Position>",Posn,",0,0</Position>"))
    fo.write('%s\n' % ("<Tilt_angle>  0.000</Tilt_angle>"))
    fo.write('%s\n' % ("<Symetric>true</Symetric>"))
    fo.write('%s\n' % ("<isFin>false</isFin>"))
    fo.write('%s\n' % ("<isDoubleFin>false</isDoubleFin>"))
    fo.write('%s\n' % ("<isSymFin>false</isSymFin>"))
    fo.write('%s\n' % ("<Inertia>"))
    fo.write('%s\n' % ("<Volume_Mass>  0.000</Volume_Mass>"))
    fo.write('%s\n' % ("</Inertia>"))
    fo.write('%s\n' % ("<Sections>"))
    for i in range(0, NSects):
        fo.write('%s\n' % ("<Section>"))
        fo.write('%s %s %s\n' % ("<y_position>",y_position[i],"</y_position>"))
        fo.write('%s %s %s\n' % ("<Chord>",Chord[i],"</Chord>"))
        fo.write('%s %s %s\n' % ("<xOffset>",xOffset[i],"</xOffset>"))
        fo.write('%s %s %s\n' % ("<Dihedral>",Dihedral[i],"</Dihedral>"))
        fo.write('%s %s %s\n' % ("<Twist>",Twist[i],"</Twist>"))
        fo.write('%s\n' % ("<x_number_of_panels>13</x_number_of_panels>"))
        fo.write('%s\n' % ("<x_panel_distribution>COSINE</x_panel_distribution>"))
        fo.write('%s\n' % ("<y_number_of_panels>5</y_number_of_panels>"))
        fo.write('%s\n' % ("<y_panel_distribution>INVERSE SINE</y_panel_distribution>"))
        fo.write('%s%s%s\n' % ("<Left_Side_FoilName>",Foil[i],"</Left_Side_FoilName>"))
        fo.write('%s%s%s\n' % ("<Right_Side_FoilName>",Foil[i],"</Right_Side_FoilName>"))
        fo.write('%s\n' % ("</Section>"))
    fo.write('%s\n' % ("</Sections>"))
    fo.write('%s\n' % ("</wing>"))

def XFLR5Elevator(fo, Posn, NSects, y_position, Chord, xOffset, Dihedral, Twist, Foil):
    fo.write('%s\n' % ("<wing>"))
    fo.write('%s\n' % ("<Name>Elevator</Name>"))
    fo.write('%s\n' % ("<Type>ELEVATOR</Type>"))
    fo.write('%s\n' % ("<Color>"))
    fo.write('%s\n' % ("<red>86</red>"))
    fo.write('%s\n' % ("<green>88</green>"))
    fo.write('%s\n' % ("<blue>208</blue>"))
    fo.write('%s\n' % ("<alpha>255</alpha>"))
    fo.write('%s\n' % ("</Color>"))
    fo.write('%s\n' % ("<Description>Main_Wing</Description>"))
    fo.write('%s %s %s\n' % ("<Position>",Posn,",0,0</Position>"))
    fo.write('%s\n' % ("<Tilt_angle>  0.000</Tilt_angle>"))
    fo.write('%s\n' % ("<Symetric>true</Symetric>"))
    fo.write('%s\n' % ("<isFin>false</isFin>"))
    fo.write('%s\n' % ("<isDoubleFin>false</isDoubleFin>"))
    fo.write('%s\n' % ("<isSymFin>false</isSymFin>"))
    fo.write('%s\n' % ("<Inertia>"))
    fo.write('%s\n' % ("<Volume_Mass>  0.000</Volume_Mass>"))
    fo.write('%s\n' % ("</Inertia>"))
    fo.write('%s\n' % ("<Sections>"))
    for i in range(0, NSects):
        fo.write('%s\n' % ("<Section>"))
        fo.write('%s %s %s\n' % ("<y_position>",y_position[i],"</y_position>"))
        fo.write('%s %s %s\n' % ("<Chord>",Chord[i],"</Chord>"))
        fo.write('%s %s %s\n' % ("<xOffset>",xOffset[i],"</xOffset>"))
        fo.write('%s %s %s\n' % ("<Dihedral>",Dihedral[i],"</Dihedral>"))
        fo.write('%s %s %s\n' % ("<Twist>",Twist[i],"</Twist>"))
        fo.write('%s\n' % ("<x_number_of_panels>13</x_number_of_panels>"))
        fo.write('%s\n' % ("<x_panel_distribution>COSINE</x_panel_distribution>"))
        fo.write('%s\n' % ("<y_number_of_panels>5</y_number_of_panels>"))
        fo.write('%s\n' % ("<y_panel_distribution>INVERSE SINE</y_panel_distribution>"))
        fo.write('%s%s%s\n' % ("<Left_Side_FoilName>",Foil[i],"</Left_Side_FoilName>"))
        fo.write('%s%s%s\n' % ("<Right_Side_FoilName>",Foil[i],"</Right_Side_FoilName>"))
        fo.write('%s\n' % ("</Section>"))
    fo.write('%s\n' % ("</Sections>"))
    fo.write('%s\n' % ("</wing>"))

def XFLR5Fin(fo, XPosn, YPosn, NSects, y_position, Chord, xOffset, Dihedral, Twist, Foil, Tilt_angle, Symetric, isDoubleFin, isSymFin):
    fo.write('%s\n' % ("<wing>"))
    fo.write('%s\n' % ("<Name>Fin</Name>"))
    fo.write('%s\n' % ("<Type>FIN</Type>"))
    fo.write('%s\n' % ("<Color>"))
    fo.write('%s\n' % ("<red>68</red>"))
    fo.write('%s\n' % ("<green>170</green>"))
    fo.write('%s\n' % ("<blue>45</blue>"))
    fo.write('%s\n' % ("<alpha>255</alpha>"))
    fo.write('%s\n' % ("</Color>"))
    fo.write('%s\n' % ("<Description>Main_Wing</Description>"))
    fo.write('%s%s%s%s%s\n' % ("<Position>",XPosn,",",YPosn,",0.0</Position>"))
    fo.write('%s %s %s\n' % ("<Tilt_angle>",Tilt_angle,"</Tilt_angle>"))
    if(Symetric):
        fo.write('%s\n' % ("<Symetric>true</Symetric>"))
    else:
        fo.write('%s\n' % ("<Symetric>false</Symetric>"))
    fo.write('%s\n' % ("<isFin>true</isFin>"))
    if(isDoubleFin):
        fo.write('%s\n' % ("<isDoubleFin>true</isDoubleFin>"))
    else:
        fo.write('%s\n' % ("<isDoubleFin>false</isDoubleFin>"))
    if(isSymFin):
        fo.write('%s\n' % ("<isSymFin>true</isSymFin>"))
    else:
        fo.write('%s\n' % ("<isSymFin>false</isSymFin>"))
    fo.write('%s\n' % ("<Inertia>"))
    fo.write('%s\n' % ("<Volume_Mass>  0.000</Volume_Mass>"))
    fo.write('%s\n' % ("</Inertia>"))
    fo.write('%s\n' % ("<Sections>"))
    for i in range(0, NSects):
        fo.write('%s\n' % ("<Section>"))
        fo.write('%s %s %s\n' % ("<y_position>",y_position[i],"</y_position>"))
        fo.write('%s %s %s\n' % ("<Chord>",Chord[i],"</Chord>"))
        fo.write('%s %s %s\n' % ("<xOffset>",xOffset[i],"</xOffset>"))
        fo.write('%s %s %s\n' % ("<Dihedral>",Dihedral[i],"</Dihedral>"))
        fo.write('%s %s %s\n' % ("<Twist>",Twist[i],"</Twist>"))
        fo.write('%s\n' % ("<x_number_of_panels>13</x_number_of_panels>"))
        fo.write('%s\n' % ("<x_panel_distribution>COSINE</x_panel_distribution>"))
        fo.write('%s\n' % ("<y_number_of_panels>5</y_number_of_panels>"))
        fo.write('%s\n' % ("<y_panel_distribution>INVERSE SINE</y_panel_distribution>"))
        fo.write('%s%s%s\n' % ("<Left_Side_FoilName>",Foil[i],"</Left_Side_FoilName>"))
        fo.write('%s%s%s\n' % ("<Right_Side_FoilName>",Foil[i],"</Right_Side_FoilName>"))
        fo.write('%s\n' % ("</Section>"))
    fo.write('%s\n' % ("</Sections>"))
    fo.write('%s\n' % ("</wing>"))

if __name__ == "__main__":
# now write out matching XFLR5 XML file
    
    fo = open("XFLR5_plane.xml", "w")
    Posn=0.0
    NSects=2
    y_position=[0.0, 2000.0]
    Chord=[750.0, 500.0]
    xOffset=[0.0, 50.0]
    Dihedral=[3.0, 3.0]
    Twist=[0.0, -2.0]
    Foil=["NACA 64-210", "NACA 64-210"]
    XFLR5Data(fo, "TestWing")
    XFLR5MainWing(fo, Posn, NSects, y_position, Chord, xOffset, Dihedral, Twist, Foil)
    y_position=[0.0, 400.0]
    Chord=[350.0, 200.0]
    xOffset=[0.0, 0.0]
    Dihedral=[0.0, 0.0]
    Twist=[0.0, 0.0]
    Posn=1600.0
    XFLR5Elevator(fo, Posn, NSects, y_position, Chord, xOffset, Dihedral, Twist, Foil)
    Dihedral=[10.0, 10.0]
    YPosn=200.0
    Tilt_angle=10.0
    Symetric=True
    isDoubleFin=False
    isSymFin=False
    XFLR5Fin(fo, Posn, YPosn, NSects, y_position, Chord, xOffset, Dihedral, Twist, Foil, Tilt_angle, Symetric, isDoubleFin, isSymFin)
    XFLR5Close(fo)
    fo.close()