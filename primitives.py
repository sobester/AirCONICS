# PRIMITIVES.PY ================================================================
# This module contains the definitions of classes of essential 2d geometrical
# primitives required for the construction of aircraft geometries.
# 
# Usage examples can be found in the airfoil_example_*.py files.
# ==============================================================================
# AirCONICS
# Aircraft CONfiguration through Integrated Cross-disciplinary Scripting 
# version 0.1.1
# Andras Sobester, 2014.
# Bug reports to a.sobester@soton.ac.uk or @ASobester please.
# ==============================================================================

# Preamble
from __future__ import division
import math, cmath
import os, rhinoscriptsyntax as rs, AirCONICStools as act
import airconics_setup
import CRMfoil


class Airfoil:
# Lifting surface section primitive class

    # Class globals
    ClosedCurve = []
    SmoothingIterations = 1


    def __init__(self, LeadingEdgePoint, ChordLength, Rotation, Twist, SeligPath = "", EnforceSharpTE = False):
        self.LeadingEdgePoint = LeadingEdgePoint
        self.ChordLength = ChordLength
        self.Rotation = Rotation
        self.Twist = Twist
        if len(SeligPath)==0:
            execfile('airconics_setup.py')
        self.SeligPath = SeligPath
        self.EnforceSharpTE = EnforceSharpTE


    def _NACA4cambercurve(self, MaxCamberLocTenthChord, MaxCamberPercChord):
        """ Generates the camber curve of a NACA 4-digit airfoil
        """
        
        # Using the original notation of Jacobs et al.(1933)
        xmc     = MaxCamberLocTenthChord /10.0;
        zcammax = MaxCamberPercChord    /100.0;
        
        # Protect against division by zero on airfoils like NACA0012
        if xmc==0:
            xmc = 0.2 

        # Sampling the chord line
        ChordCoord, NCosPoints = act.coslin(xmc)
        
        # Compute the two sections of the camber curve and its slope
        zcam = []
        dzcamdx = []
        for cc in ChordCoord[0:NCosPoints]:
            list.append(zcam, (zcammax/(xmc ** 2))*(2*xmc*cc - cc ** 2))
            list.append(dzcamdx, (zcammax/xmc ** 2)*(2*xmc - 2*cc))
        for cc in ChordCoord[NCosPoints:]:
            list.append(zcam, (zcammax/((1-xmc) ** 2))*(1-2*xmc+2*xmc*cc-(cc ** 2)))
            list.append(dzcamdx, (zcammax/(1-xmc) ** 2)*(2*xmc - 2*cc));
        return ChordCoord, zcam, dzcamdx



    def _NACA5cambercurve(self, MaxCamberLocFracChord, DesignLiftCoefficient):
        # Generates the camber curve of a NACA 5-digit airfoil
        
        xmc = MaxCamberLocFracChord

        # Determine the transition point m that separates the polynomial
        # forward section from the linear aft section
        R = act.cubic(-3, 6*xmc, -3*xmc**2)
        m = R[2].real
    
        # Sampling the chord line
        ChordCoord, NCosPoints = act.coslin(xmc)
        
        # As per equation (A-13) in Bill Mason's Geometry for
        # Aerodynamicists
        QQ = (3*m-7*m**2+8*m**3-4*m**4) / (math.sqrt(m*(1-m))) - 3/2*(1-2*m)*(math.pi/2-math.asin(1-2*m))
        k1 = 6*DesignLiftCoefficient/QQ
    
        # Compute the two sections of the camber curve and its slope
        zcam = []
        dzcamdx = []
        for cc in ChordCoord[0:NCosPoints]:
            list.append(zcam, (1/6)*k1*(cc**3 - 3*m*cc**2 + m**2*(3-m)*cc))
            list.append(dzcamdx, (1/6)*k1*(3*cc**2-6*m*cc+m**2*(3-m)))
        for cc in ChordCoord[NCosPoints:]:
            list.append(zcam, (1/6)*k1*m**3*(1-cc))
            list.append(dzcamdx, -(1/6)*m**3);
        return ChordCoord, zcam, dzcamdx



    def _NACA4halfthickness(self, ChordCoord, MaxThicknessPercChord):
        # Given  a set of ordinates  and  a  maximum thickness value
        # (expressed in units of chord) it computes the NACA 4-digit
        # half-thickness distribution.  The abscissas must be in the
        # range [0,1].

        # Max thickness in units of chord
        tmax = MaxThicknessPercChord / 100.0

        # Coefficient tweak to close off the trailing edge if required
        a0 =  0.2969/0.2
        a1 = -0.1260/0.2
        a2 = -0.3516/0.2
        a3 =  0.2843/0.2
        a4 = -0.1015/0.2
        # The highest order term could be fudged to make t(1) = 0, thus producing
        # a sharp trailing edge (NACA4s by definition have a finite thickness TE).
        # However, this is probably better enforced by removing a wedge from the 
        # coordinate sets (a generic method). Still, this might be a NACA-specific
        # alternative:
        # t_at_one = a0+a1+a2+a3+a4
        # a4 = a4 - t_at_one

        # Half-thickness polynomial
        t = []
        for cc in ChordCoord:
            list.append(t, tmax*(a0*cc**0.5 + a1*cc + a2*cc**2.0 + a3*cc**3.0 + a4*cc**4.0))
        return t


    def _camberplusthickness(self, ChordCoord, zcam, dzcamdx, t):
        # Internal function. Adds a thickness distribution to a specified camber line.
        # The slope is an input here, because it is usually possible to compute it
        # analytically at the same time as the curve itself is computed.

        # Theta angle (slope of the camber curve)
        Theta = []
        for dz in dzcamdx:
            list.append(Theta, math.atan(dz))
        xu = [];zu = [];xl = [];zl = []
        for i, Th in enumerate(Theta):
            list.append(xu,ChordCoord[i] - t[i]*math.sin(Th))
            list.append(xl,ChordCoord[i] + t[i]*math.sin(Th))
            list.append(zu,zcam[i]       + t[i]*math.cos(Th))
            list.append(zl,zcam[i]       - t[i]*math.cos(Th))
            
        # Correct small abscissa positioning errors in case of sharp TE
        if self.EnforceSharpTE:
            xu[-1] = ChordCoord[-1]
            xl[-1] = ChordCoord[-1]

        return xu, zu, xl, zl, Theta


    def _mergesurfaces(self, xu, zu, xl, zl, RemoveFiniteTE):
        # Combine the upper and lower surfaces into one
        
        if RemoveFiniteTE:
            # Remove wedge to sharpen trailing edge if needed
            for i, x in enumerate(xu):
                zu[i] = zu[i] - x * zu[-1]
            for i, x in enumerate(xl):
                zl[i] = zl[i] - x * zl[-1]    
            
        xu.reverse() # Top surface from right to left
        zu.reverse()
        x = xu + xl[1:] # Remove duplicate leading edge point 
        z = zu + zl[1:]
        return x,z

    
    def _NACA4digitPnts(self, MaxCamberPercChord, MaxCamberLocTenthChord, MaxThicknessPercChord):
        # Generates a set of points that define a NACA 4-digit airfoil

        ChordCoord, zcam, dzcamdx = self._NACA4cambercurve(MaxCamberLocTenthChord, MaxCamberPercChord)

        t = self._NACA4halfthickness(ChordCoord, MaxThicknessPercChord)

        xu, zu, xl, zl, Theta = self._camberplusthickness(ChordCoord, zcam, dzcamdx, t)
        
        # Leading edge radius
        RLE = 1.1019*(MaxThicknessPercChord/100.0)**2.0
        
        x,z = self._mergesurfaces(xu,zu,xl,zl,self.EnforceSharpTE)
        
        return x, z, xu, zu, xl, zl, RLE
    
    

    def _NACA5digitPnts(self, DesignLiftCoefficient, MaxCamberLocFracChord, MaxThicknessPercChord):
        """ Generates a set of points that define a NACA 5-digit airfoil
             - Additional inputs:
                   DesignLiftCoefficient
                   MaxCamberLocFracChord (between 0.05 and 0.25)
                   MaxThicknessPercChord (percentage)
            EXAMPLES:
            The 'originals', as per Jacobs and Pinkerton (1935):
            NACA5digitPnts(0.3, 0.05, 12) - NACA 21012
            NACA5digitPnts(0.3, 0.10, 12) - NACA 22012
            NACA5digitPnts(0.3, 0.15, 12) - NACA 23012
            NACA5digitPnts(0.3, 0.20, 12) - NACA 24012
            NACA5digitPnts(0.3, 0.25, 12) - NACA 25012
        """
        ChordCoord, zcam, dzcamdx = self._NACA5cambercurve(MaxCamberLocFracChord, DesignLiftCoefficient)
    
        # Same thickness as the NACA 4-digit
        t = self._NACA4halfthickness(ChordCoord, MaxThicknessPercChord)
        
        xu, zu, xl, zl, Theta = self._camberplusthickness(ChordCoord, zcam, dzcamdx, t)
        
        # Leading edge radius
        RLE = 1.1019*(MaxThicknessPercChord/100.0)**2.0
        
        x,z = self._mergesurfaces(xu,zu,xl,zl,self.EnforceSharpTE)
        return x, z, xu, zu, xl, zl, RLE


    def _fitAirfoiltoPoints(self, x, z):
        # Fits a curve to a list of (x,z) coordinates on the y=0 plane
        PointsList = []
        for i in range(len(x)):
            p = rs.AddPoint(((x[i],0,z[i])))
            list.append(PointsList,p)
        AfCurveOpen = rs.AddInterpCurve(PointsList)
        rs.DeleteObjects(PointsList)
        return AfCurveOpen        


    def _TransformAirfoil(self, C):
        # Internal function. Given a normal airfoil, unit chord, nose in origin,
        # chord along x axis, applies scaling, rotations, positioning and smoothing

        # Smoothing
        for i in range(1,self.SmoothingIterations+1):
            rs.FairCurve(C)

        # Find the actual leading edge point - the airfoil may have stretched as
        # as a result of the smoothing or it may have been incorrectly defined 
        # through a series of coordinates
        RefLine = rs.AddLine((-100,0,-100),(-100,0,100))
        ClosestPoints = rs.CurveClosestObject(RefLine,C)
        P = rs.AddPoint(ClosestPoints[1])
        MoveVec = rs.VectorCreate((0,0,0),P)
        rs.MoveObject(C, MoveVec)
        # Garbage collection
        rs.DeleteObjects((RefLine,P))
        # Now find the trailing edge points
        PUpper = rs.CurveStartPoint(C)
        PLower = rs.CurveEndPoint(C)
        TECentre = ((PUpper[0]+PLower[0])/2,(PUpper[1]+PLower[1])/2,(PUpper[2]+PLower[2])/2)
        if PUpper[2]<PLower[2]:
            print "Warning: the upper and lower surface intersect at the TE."
        TECentrePoint = rs.AddPoint(TECentre)
        AxisOfRotation = rs.VectorCreate((0,0,0),(0,1,0))
        
        L1 = rs.AddLine((0,0,0),(1,0,0))
        L2 = rs.AddLine((0,0,0),TECentrePoint)
        AngRot = rs.Angle2(L1,L2)
        # The angle returned by Angle2 is always positive so:
        if TECentre[2] < 0:
            rs.RotateObject(C,(0,0,0), AngRot[0], AxisOfRotation)
        else:
            rs.RotateObject(C,(0,0,0), -AngRot[0], AxisOfRotation)
        # Garbage collection
        rs.DeleteObjects((TECentrePoint, L1, L2))

        # Find the trailing edge point again after rotating it onto the x axis
        PUpper = rs.CurveStartPoint(C)
        PLower = rs.CurveEndPoint(C)
        TECentre = [(PUpper[0]+PLower[0])/2,(PUpper[1]+PLower[1])/2,(PUpper[2]+PLower[2])/2]
        ActualChordLength = TECentre[0]
     
        # Scale the airfoil to unit chord
        #rs.ScaleObject(C, (0,0,0), (1/ActualChordLength, 1, 1/ActualChordLength))
        act.ScaleObjectWorld000(C, (1/ActualChordLength, 1, 1/ActualChordLength))

        # Now we can assume that airfoil is normalised to the unit chord, with
        # its leading edge in the origin, trailing edge in (1,0,0)
        Chrd = rs.AddLine((0,0,0),(1,0,0))
        
        # Scaling
        ScaleFact = (self.ChordLength, self.ChordLength, self.ChordLength)
        
        # same as rs.ScaleObject(C, (0,0,0), ScaleFact)
        act.ScaleObjectWorld000(C, ScaleFact)
        
        # same as rs.ScaleObject(Chrd, (0,0,0), ScaleFact)
        act.ScaleObjectWorld000(Chrd, ScaleFact)
        
        # Twist
        rs.RotateObject(C, (0,0,0), self.Twist, rs.VectorCreate((0,0,0),(0,1,0)))
        rs.RotateObject(Chrd, (0,0,0), self.Twist, rs.VectorCreate((0,0,0),(0,1,0)))
        # Dihedral
        rs.RotateObject(C, (0,0,0), -self.Rotation, rs.VectorCreate((0,0,0),(1,0,0)))
        rs.RotateObject(Chrd, (0,0,0), -self.Rotation, rs.VectorCreate((0,0,0),(1,0,0)))
        # 3d positioning
        MoveVec = rs.VectorCreate(self.LeadingEdgePoint, (0,0,0))
        rs.MoveObject(C, MoveVec)
        rs.MoveObject(Chrd, MoveVec)
        return C, Chrd

    def AddNACA4(self, MaxCamberPercChord, MaxCamberLocTenthChord, MaxThicknessPercChord, Smoothing=1):
        # Adds a NACA 4 digit airfoil to the current document
        x, z, xu, zu, xl, zl, RLE = self._NACA4digitPnts(MaxCamberPercChord, MaxCamberLocTenthChord, MaxThicknessPercChord)
        C = self._fitAirfoiltoPoints(x, z)
        if 'Smoothing' in locals():
            self.SmoothingIterations = Smoothing
        C, Chrd = self._TransformAirfoil(C)
        return C, Chrd

    def AddNACA5(self, DesignLiftCoefficient, MaxCamberLocFracChord, MaxThicknessPercChord, Smoothing=1):
        # Adds a NACA 5 digit airfoil to the current document
        x, z, xu, zu, xl, zl, RLE = self._NACA5digitPnts(DesignLiftCoefficient, MaxCamberLocFracChord, MaxThicknessPercChord)
        C = self._fitAirfoiltoPoints(x, z)
        if 'Smoothing' in locals():
            self.SmoothingIterations = Smoothing
        C, Chrd = self._TransformAirfoil(C)
        return C, Chrd
        
    def AddCRMLinear(self, Epsilon, Smoothing=1):
        x,z = CRMfoil.CRMlinear(Epsilon)
        C = self._fitAirfoiltoPoints(x, z)
        if 'Smoothing' in locals():
            self.SmoothingIterations = Smoothing
        C, Chrd = self._TransformAirfoil(C)
        return C, Chrd

    def _AirfoilPointsSeligFormat(self, FileNameWithPath):
        # Extracts airfoil coordinates from a file, assuming that they are specified
        # in the Selig format, i.e., header line, followed by x column, z column, 
        # from upper trailing edge to lower trailing edge.
        with open(FileNameWithPath,'r') as f:
            lines = f.readlines()
        f.close()
        x = []
        z = []
        for l in lines:
            try:
                l = l.split()
                newx = float(l[0])
                newz = float(l[1])
                list.append(x, newx)
                list.append(z, newz)
            except:
                pass
        return x, z

    def AddAirfoilFromSeligFile(self, AirfoilSeligName, Smoothing=1):
        # Adds an airfoil to the current document generated by fitting a smoothed
        # NURBS curve to a set of points whose coordinates are given in a Selig 
        # formatted file

        FileNameWithPath = self.SeligPath + AirfoilSeligName + '.dat'
        x, z = self._AirfoilPointsSeligFormat(FileNameWithPath)
        C = self._fitAirfoiltoPoints(x, z)
        if 'Smoothing' in locals():
            self.SmoothingIterations = Smoothing
        C, Chrd = self._TransformAirfoil(C)
        return C, Chrd