# LIFTINGSURFACE.PY ============================================================
# This module contains the definition of the class of 3d lifting surfaces.
# This class can be instantiated to generate wings, tailplanes, fins, propeller-
# or rotor blades, etc.
#
# See wing_example*.py for a number of example scripts illustrating usage or the
# parametric airliner in transonic_airliner.py.
# ==============================================================================
# AirCONICS
# Aircraft CONfiguration through Integrated Cross-disciplinary Scripting 
# version 0.2.0
# Andras Sobester, 2015.
# Bug reports to a.sobester@soton.ac.uk or @ASobester please.
# ==============================================================================

# Preamble
from __future__ import division
import rhinoscriptsyntax as rs
import math, cmath, os, bisect
import primitives, AirCONICStools as act


class LiftingSurface:

    def __init__(self, ApexPoint, SweepFunct, DihedralFunct, TwistFunct, ChordFunct, AirfoilFunct, LooseSurf=1, SegmentNo=11, TipRequired = True):

        self.ApexPoint = ApexPoint
        self.SweepFunct = SweepFunct
        self.DihedralFunct = DihedralFunct
        self.TwistFunct = TwistFunct
        self.ChordFunct = ChordFunct
        self.AirfoilFunct = AirfoilFunct
        self.LooseSurf = LooseSurf
        self.SegmentNo = SegmentNo
        self.TipRequired = TipRequired

        self._CreateConstructionGeometry()


    def _CreateConstructionGeometry(self):
        self.PCG1 = rs.AddPoint([-100,-100,0])
        self.PCG2 = rs.AddPoint([-100,100,0])
        self.PCG3 = rs.AddPoint([100,100,0])
        self.PCG4 = rs.AddPoint([100,-100,0])
        self.XoY_Plane = rs.AddSrfPt([self.PCG1, self.PCG2, self.PCG3, self.PCG4])
        self.PCG5 = rs.AddPoint([3,3,0])
        self.PCG6 = rs.AddPoint([3,3,100])
        self.ProjVectorZ = rs.VectorCreate(self.PCG5, self.PCG6)


    def _ClearConstructionGeometry(self):
        rs.DeleteObject(self.PCG1)
        rs.DeleteObject(self.PCG2)
        rs.DeleteObject(self.PCG3)
        rs.DeleteObject(self.PCG4)
        rs.DeleteObject(self.XoY_Plane)
        rs.DeleteObject(self.PCG5)
        rs.DeleteObject(self.PCG6)


    def _NormaliseWeightings(self):
        # Internal function to compute normalised weightings
        
        wTotal = self.wTargetArea + \
                 self.wTargetAspectRatio + \
                 self.wTargetSpan + \
                 self.wTargetRootChord + \
                 self.wTargetWettedArea
        
        self.wTargetArea = abs(self.wTargetArea)/wTotal
        self.wTargetAspectRatio = abs(self.wTargetAspectRatio)/wTotal
        self.wTargetSpan = abs(self.wTargetSpan)/wTotal
        self.wTargetRootChord = abs(self.wTargetRootChord)/wTotal
        self.wTargetWettedArea = abs(self.wTargetWettedArea)/wTotal

    def _PrintTargetsAndWeights(self):
        print("Target values of the wing parameters and their weightings:")
        print("----------------------------------------------------------")
        print("Target area: %3.2f (weighting: %3.2f)" %  (self.TargetArea, self.wTargetArea))
        print("Target aspect ratio: %3.2f (weighting: %3.2f)" %  (self.TargetAspectRatio, self.wTargetAspectRatio))
        print("Target span: %3.2f (weighting: %3.2f)" %  (self.TargetSpan, self.wTargetSpan))
        print("Target root chord: %3.2f (weighting: %3.2f)" %  (self.TargetRootChord, self.wTargetRootChord))
        print("Target wetted area: %3.2f (weighting: %3.2f)" %  (self.TargetWettedArea, self.wTargetWettedArea))

    def _GenerateLeadingEdge(self):
        # Epsilon coordinate attached to leading edge defines sweep
        # Returns airfoil leading edge points

        # Start the leading edge at the origin
        XLE = [0.0]
        YLE = [0.0]
        ZLE = [0.0]

        SegmentLength = 1.0/self.SegmentNo

        LEPoints = []
        list.append(LEPoints,rs.AddPoint(XLE[0], YLE[0], ZLE[0]))

        for i in range(1,self.SegmentNo+1):
            # We are essentially reconstructing a curve from known slopes at 
            # known curve length stations - a sort of Hermite interpolation without
            # knowing the ordinate values. If SegmentNo -> Inf, the actual slope
            # at each point -> the sweep angle specified by SweepFunct

            TiltAngle = self.DihedralFunct(((i-1)/float(self.SegmentNo)+i/float(self.SegmentNo))/2)
            SweepAngle = self.SweepFunct(((i-1)/float(self.SegmentNo)+i/float(self.SegmentNo))/2)

            DeltaX = SegmentLength*math.sin(SweepAngle*math.pi/180.0)
            DeltaY = SegmentLength*math.cos(TiltAngle*math.pi/180.0)*math.cos(SweepAngle*math.pi/180.0)
            DeltaZ = DeltaY*math.tan(TiltAngle*math.pi/180.0)

            list.append(XLE, XLE[i-1] + DeltaX)
            list.append(YLE, YLE[i-1] + DeltaY)
            list.append(ZLE, ZLE[i-1] + DeltaZ)

            list.append(LEPoints,rs.AddPoint(XLE[i], YLE[i], ZLE[i]))

        return LEPoints


    def _BuildLS(self, ChordFactor, ScaleFactor):
        # Generates a tentative lifting surface, given the general, nondimensio-
        # nal parameters of the object (variations of chord length, dihedral, etc.)
        # and the two scaling factors.

        LEPoints = self._GenerateLeadingEdge()

        Sections = []
        ProjectedSections = []
        TEPoints_u = []
        TEPoints_l = []
        
        for i, LEP in enumerate(LEPoints):
            Eps = float(i)/self.SegmentNo
            Airfoil, Chrd = self.AirfoilFunct(Eps, LEP, self.ChordFunct, ChordFactor, self.DihedralFunct, self.TwistFunct)
            list.append(Sections, Airfoil)
            
            Pr = rs.ProjectCurveToSurface(Chrd,self.XoY_Plane,self.ProjVectorZ)
            list.append(ProjectedSections, Pr)

            list.append(TEPoints_l, rs.CurveEndPoint(Airfoil))
            list.append(TEPoints_u, rs.CurveStartPoint(Airfoil))

            rs.DeleteObjects(Chrd)

        LS = rs.AddLoftSrf(Sections,loft_type=self.LooseSurf)

        if LS==None:
            # Failed to fit loft surface. Try another fitting algorithm
            TECurve_u = rs.AddInterpCurve(TEPoints_u)
            TECurve_l = rs.AddInterpCurve(TEPoints_l)

            rails = []
            list.append(rails, TECurve_u)
            list.append(rails, TECurve_l)

            # Are the first and last curves identical?
            # AddSweep fails if they are, so if that is the case, one is skipped
            CDev = rs.CurveDeviation(Sections[0],Sections[-1])
            if CDev==None:
                shapes = Sections
                LS = rs.AddSweep2(rails, shapes, False)
            else:
                shapes = Sections[:-1]
                LS = rs.AddSweep2(rails, shapes, True)
            
            rs.DeleteObjects(rails)
            rs.DeleteObjects([TECurve_u, TECurve_l])
            
        WingTip = None

        if self.TipRequired:
            TipCurve = Sections[-1]
            TipCurve = act.AddTEtoOpenAirfoil(TipCurve)
            WingTip = rs.AddPlanarSrf(TipCurve)
            rs.DeleteObject(TipCurve)

        # Calculate projected area
        # In some cases the projected sections cannot all be lofted in one go
        # (it happens when parts of the wing fold back onto themselves), so
        # we loft them section by section and we compute the area as a sum.
        LSP_area = 0
        # Attempt to compute a projected area
        try:
            for i, LEP in enumerate(ProjectedSections):
                if i < len(ProjectedSections)-1:
                    LSPsegment = rs.AddLoftSrf(ProjectedSections[i:i+2])
                    SA = rs.SurfaceArea(LSPsegment)
                    rs.DeleteObject(LSPsegment)
                    LSP_area = LSP_area + SA[0]
        except:
            print "Failed to compute projected area. Using half of surface area instead."
            LS_area = rs.SurfaceArea(LS)
            LSP_area = 0.5*LS_area[0]

        BB = rs.BoundingBox(LS)
        if BB:
            ActualSemiSpan = BB[2].Y - BB[0].Y
        else:
            ActualSemiSpan = 0.0

        # Garbage collection
        rs.DeleteObjects(Sections)
        try:
            rs.DeleteObjects(ProjectedSections)
        except:
            print "Cleanup: no projected sections to delete"
        rs.DeleteObjects(LEPoints)
        
     
        # Scaling
        Origin = rs.AddPoint([0,0,0])
        ScaleXYZ = (ScaleFactor, ScaleFactor, ScaleFactor)
        LS = rs.ScaleObject(LS, Origin, ScaleXYZ)
        if self.TipRequired and WingTip:
            WingTip = rs.ScaleObject(WingTip, Origin, ScaleXYZ)

        rs.DeleteObject(Origin)

        ActualSemiSpan = ActualSemiSpan*ScaleFactor
        LSP_area = LSP_area*ScaleFactor**2.0
        RootChord = (self.ChordFunct(0)*ChordFactor)*ScaleFactor
        AR = ((2.0*ActualSemiSpan)**2.0)/(2*LSP_area)

        return LS, ActualSemiSpan, LSP_area, RootChord, AR, WingTip


    def _LSMetrics(self, CandidateSurface, ActualSemiSpan, CandidateSurfaceProjection_area, ChordFactor, ScaleFactor):
        # This is the objective function of the search for the lifting surface that
        # best matches the targets (i.e., the best scaling factor values). Should
        # delete the surface once the calculation is complete.
        SA  = rs.SurfaceArea(CandidateSurface)
        PSA = CandidateSurfaceProjection_area
        AR = ((2.0*ActualSemiSpan)**2.0)/(2.0*PSA) 
        RootChord = (self.ChordFunct(0)*ChordFactor)*ScaleFactor
        # Weighted combined metric
        WM =\
        self.wTargetArea       *((self.TargetArea       - 2.0*PSA   )         /self.TargetArea       )**2.0+\
        self.wTargetWettedArea *((self.TargetWettedArea - 2.0* SA[0])         /self.TargetWettedArea )**2.0+\
        self.wTargetSpan       *((self.TargetSpan       - 2.0* ActualSemiSpan)/self.TargetSpan       )**2.0+\
        self.wTargetAspectRatio*((self.TargetAspectRatio- AR        )         /self.TargetAspectRatio)**2.0+\
        self.wTargetRootChord  *((self.TargetRootChord  - RootChord )         /self.TargetRootChord  )**2.0
        print("Proj.area:%3.2f Wet.area:%3.2f Span:%3.2f Aspect ratio:%3.2f Root chord:%3.2f" % (2.0*PSA, 2.0* SA[0], 2.0* ActualSemiSpan, AR, RootChord))
        return WM


    def _LSObjective(self, x):
        # Negative factors don't make sense, so mirroring objective landscape
        ChordFactor = abs(x[0])
        ScaleFactor = abs(x[1])
        # Builds a surface and computes the appropriate objective function
        print("Building lifting surface with %5.4f chord scaling and %5.4f overall scaling..." %  (ChordFactor, ScaleFactor))
        LS, ActualSemiSpan, LSP_area, RootChord, AR, WingTip = self._BuildLS(ChordFactor, ScaleFactor)
        ObjFunction = self._LSMetrics(LS, ActualSemiSpan, LSP_area, ChordFactor, ScaleFactor)
        if WingTip:
            rs.DeleteObject(WingTip)
        if LS:
            rs.DeleteObject(LS)
        print("Objective value: %3.2f" % ObjFunction)
        return ObjFunction


    def _CheckOptParCorrectlySpec(self):
        # Create weights if they do not exist
        try:
            self.wTargetAspectRatio
        except:
            self.wTargetAspectRatio = 0

        try:
            self.wTargetRootChord
        except:
            self.wTargetRootChord = 0

        try:
            self.wTargetSpan
        except:
            self.wTargetSpan = 0

        try:
            self.wTargetArea
        except:
            self.wTargetArea = 0

        try:
            self.wTargetWettedArea
        except:
            self.wTargetWettedArea = 0

        # At least two of them must be positive, non-zero
        WeightList = (self.wTargetAspectRatio, self.wTargetRootChord, self.wTargetSpan,
        self.wTargetArea, self.wTargetWettedArea)
        
        if sum(w>0 for w in WeightList)<2:
            error("At least two targets must be specified if optimization is requested.")
        
        # Now check for the actual target values - make sure that if the weighting is non-zero
        # a target is actually specified
        try:
            self.TargetAspectRatio
        except:
            if self.wTargetAspectRatio>0:
                error("Target AspectRatio value must be specified if weighting is non-zero.")
            else:
                self.TargetAspectRatio = 1

        try:
            self.TargetRootChord
        except:
            if self.wTargetRootChord>0:
                error("Target RootChord value must be specified if weighting is non-zero.")
            else:
                self.TargetRootChord = 1
        
        try:
            self.TargetSpan
        except:
            if self.wTargetSpan>0:
                error("Target Span value must be specified if weighting is non-zero.")
            else:
                self.TargetSpan = 1

        try:
            self.TargetArea
        except:
            if self.wTargetArea>0:
                error("Target Area value must be specified if weighting is non-zero.")
            else:
                self.TargetArea = 1

        try:
            self.TargetWettedArea
        except:
            if self.wTargetWettedArea>0:
                error("Target WettedArea value must be specified if weighting is non-zero.")
            else:
                self.TargetWettedArea = 1


    def GenerateLiftingSurface(self, ChordFactor, ScaleFactor, OptimizeChordScale=0):
        # This is the main method of this class. It builds a lifting surface
        # (wing, tailplane, etc.) with the given ChordFactor and ScaleFactor or 
        # an optimized ChordFactor and ScaleFactor, with the local search started
        # from the two given values.

        x0 = [ChordFactor, ScaleFactor]

        if OptimizeChordScale:
            self._CheckOptParCorrectlySpec()
            self._NormaliseWeightings()
            self._PrintTargetsAndWeights()
            print("Optimizing scale factors...")
            # An iterative local hillclimber type optimizer is needed here. One
            # option might be SciPy's fmin as below:
            # x0, fopt, iter, funcalls, warnflag, allvecs = scipy.optimize.fmin(self._LSObjective, x0, retall=True, xtol=0.025, full_output=True)
            # However, SciPy is not supported on 64-bit Rhino installations, so 
            # so here we use an alternative: a simple evoltionary optimizer
            # included with AirCONICS_tools.
            MaxIter = 50
            xtol = 0.025
            deltax = [x0[0]*0.25,x0[1]*0.25]
            x0, fopt = act.boxevopmin2d(self._LSObjective, x0, deltax, xtol, MaxIter)
            x0[0] = abs(x0[0])
            x0[1] = abs(x0[1])
            print("Optimum chord factor %5.3f, optimum scale factor %5.3f" % (x0[0], x0[1]))

        LS, ActualSemiSpan, LSP_area,  RootChord, AR, WingTip = self._BuildLS(x0[0], x0[1])
        
        self._ClearConstructionGeometry()
        
        # Moving the wing into position
        AP = rs.AddPoint(self.ApexPoint)
        MoveVec = rs.VectorCreate(AP, (0,0,0))
        rs.MoveObject(LS, MoveVec)
        if WingTip:
            rs.MoveObject(WingTip, MoveVec)
        rs.DeleteObject(AP)

        # FINAL REPORT ON THE COMPLETED SURFACE
        SA  = rs.SurfaceArea(LS)
        print("Wing complete. Key features (all related to both wings):")
        print("Proj.area: %5.4f   Wet.area: %5.4f   Span:%5.4f  Aspect ratio:  %5.4f  Root chord: %5.4f" % (2*LSP_area, 2.0*SA[0], 2.0*ActualSemiSpan, AR, RootChord))

        return LS, ActualSemiSpan, LSP_area,  RootChord, AR, WingTip