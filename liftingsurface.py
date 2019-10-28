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

    def __init__(self, ApexPoint, SweepFunct, DihedralFunct, TwistFunct, ChordFunct, AirfoilFunct, LooseSurf=1, SegmentNo=11, TipRequired = 1, SectionsRequired = False):

        self.ApexPoint = ApexPoint
        self.SweepFunct = SweepFunct
        self.DihedralFunct = DihedralFunct
        self.TwistFunct = TwistFunct
        self.ChordFunct = ChordFunct
        self.AirfoilFunct = AirfoilFunct
        self.LooseSurf = LooseSurf
        self.SegmentNo = SegmentNo
        self.TipRequired = TipRequired
        self.SectionsRequired = SectionsRequired

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



        if self.TipRequired==1:
            # Simple flat wingtip
            TipCurve = rs.CopyObject(Sections[-1])
            TipCurve = act.AddTEtoOpenAirfoil(TipCurve)
            WingTip = rs.AddPlanarSrf(TipCurve)
            rs.DeleteObject(TipCurve)
        elif self.TipRequired==2:
            # Rounded wingtip
            
            SemiCircList = []
            MidPointList = []
            Rail1Points = []
            Rail2Points = []
            
            TipCurve = rs.CopyObject(Sections[-1])
            TipCurve = act.AddTEtoOpenAirfoil(TipCurve)
            
            AFrontVertPlane = rs.PlaneFromPoints([-10,0,0],[-10,1,0],[-10,1,1])   
            AFrontVertPlaneSrf =  rs.AddPlaneSurface(AFrontVertPlane, 2.5,2.5)
            ClosestPoints = rs.CurveClosestObject(TipCurve, AFrontVertPlaneSrf)
            if ClosestPoints[2][0]>-9:
                LeadingPoint = rs.AddPoint(ClosestPoints[2])
                Xslice0 = ClosestPoints[2][0]
            else:
                LeadingPoint = rs.AddPoint(ClosestPoints[1])
                Xslice0 = ClosestPoints[1][0]

            rs.DeleteObject(AFrontVertPlaneSrf)


            for Xslice in act.pwfrange(Xslice0+0.0001, 1.1, 3000):
                
                
                print Xslice
                
                # Intersect vertical plane with the tip curve
                P = rs.PlaneFromPoints([Xslice,0,0],[Xslice,1,0],[Xslice,1,1])   
                IPs = rs.PlaneCurveIntersection(P,TipCurve,tolerance = 0.00000000001)

                if not(IPs is None) and len(IPs)>1:
                    # Unpack the intersection data
                    # Extract the first point whose abscissa is equal to the plane's
                    if abs(IPs[0][1][0]-Xslice)<0.0000001:
                        iIP0 = 1
                    elif abs(IPs[0][2][0]-Xslice)<0.0000001:
                        iIP0 = 2
                    elif abs(IPs[0][3][0]-Xslice)<0.0000001:
                        iIP0 = 3
                    else:
                        iIP0 = 4

                    if abs(IPs[1][1][0]-Xslice)<0.0000001:
                        iIP1 = 1
                    elif abs(IPs[1][2][0]-Xslice)<0.0000001:
                        iIP1 = 2
                    elif abs(IPs[1][3][0]-Xslice)<0.0000001:
                        iIP1 = 3
                    else:
                        iIP1 = 4

                    IP0 = IPs[0][iIP0]
                    IP1 = IPs[1][iIP1]
                    
                    IP0par = rs.CurveClosestPoint(TipCurve, IP0)
                    IP0a = rs.EvaluateCurve(TipCurve, IP0par)

                    IP1par = rs.CurveClosestPoint(TipCurve, IP1)
                    IP1a = rs.EvaluateCurve(TipCurve, IP1par)

                    PCcoord =   [0.5*(IP0a[0]+IP1a[0]), 0.5*(IP0a[1]+IP1a[1]), 0.5*(IP0a[2]+IP1a[2])]
                    Dia = math.sqrt( (IP0a[0]-IP1a[0])**2 + (IP0a[1]-IP1a[1])**2 + (IP0a[2]-IP1a[2])**2 )

                    # Add a circle centred on the halfway point between the two intersection points

                    C = rs.AddCircle(P,0.5*Dia)
                    C = rs.MoveObject(C, [0,PCcoord[1], PCcoord[2]])

                    # Trim off the part of the circle that is inside the wing
                    CD = rs.CurveDomain(C)
                    TrimLine = rs.AddLine(IPs[0][1],IPs[1][1])
                    TrimVec = rs.VectorCreate(IPs[0][1],IPs[1][1])
                    
                    rs.ReverseCurve(C)
                    rs.UnselectAllObjects()
                    rs.Command("-Trim selid "+str(TrimLine)+" _Enter "+" selid "+str(C)+" _Enter ")
                    C = rs.LastCreatedObjects()
                    rs.RotateObject(C, PCcoord, 180, axis = TrimVec)

                    # Deal with the intersection process yielding arcs that hang in mid-air    
                    if not(rs.IsCurve(C)):
                        print "C is not a curve"

                    CEP = rs.CurveEndPoint(C)
                    CSP = rs.CurveStartPoint(C)

                    if not(rs.IsPointOnCurve(TipCurve, CEP) and rs.IsPointOnCurve(TipCurve,CSP)):
                        rs.DeleteObject(C)
                    else:
                        list.append(SemiCircList, C)
                        CMP = rs.CurveMidPoint(C)
                        list.append(MidPointList, CMP)
                        list.append(Rail1Points, CEP)
                        list.append(Rail2Points, CSP)

                    rs.DeleteObject(TrimLine)

                else:
                    break


            Spine = rs.AddInterpCurve([LeadingPoint] + MidPointList + [TEPoints_u[-1]])
            Rail1 = rs.AddInterpCurve([LeadingPoint] + Rail1Points  + [TEPoints_u[-1]])
            Rail2 = rs.AddInterpCurve([LeadingPoint] + Rail2Points  + [TEPoints_u[-1]])

            WingTip = rs.AddSweep2([Rail1, Rail2], SemiCircList)

            SCS = rs.CurveStartPoint(SemiCircList[0])
            SCE =   rs.CurveEndPoint(SemiCircList[0])
            cdom = rs.CurveDomain(SemiCircList[0])
            SemiCircMid = rs.CurveMidPoint(SemiCircList[0])

            SubCurves = rs.SplitCurve(SemiCircList[0], cdom[0]+0.5*(cdom[1]-cdom[0]))

            L1 = rs.AddLine(SCS, LeadingPoint)
            L2 = rs.AddLine(SCE, LeadingPoint)
            L3 = rs.AddLine(SemiCircMid, LeadingPoint)
            
            LEGAP1 = rs.AddEdgeSrf([SubCurves[1], L3,L2])
            LEGAP2 = rs.AddEdgeSrf([SubCurves[0], L1,L3])
            
            
            WingTip = rs.JoinSurfaces([WingTip, LEGAP1, LEGAP2], True)
            
            rs.DeleteObject(TipCurve)
            rs.DeleteObject(Spine)
            rs.DeleteObjects(SemiCircList)
            rs.DeleteObjects([Rail1, Rail2, L1, L2, L3]+SubCurves)
     



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
        if not(self.SectionsRequired):
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
        if self.SectionsRequired:
            Sections = rs.ScaleObjects(Sections, Origin, ScaleXYZ)
        else:
            Sections = []

        rs.DeleteObject(Origin)

        ActualSemiSpan = ActualSemiSpan*ScaleFactor
        LSP_area = LSP_area*ScaleFactor**2.0
        RootChord = (self.ChordFunct(0)*ChordFactor)*ScaleFactor
        AR = ((2.0*ActualSemiSpan)**2.0)/(2*LSP_area)


        try:
            rs.DeleteObject(LeadingPoint)
        except:
            pass

        return LS, ActualSemiSpan, LSP_area, RootChord, AR, WingTip, Sections


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

        # Tiny scaling values do not make sense either so a large objective value is
        # returned to steer the search away from there
        if ChordFactor > 0.001 and ScaleFactor > 0.001:
            # Builds a surface and computes the appropriate objective function
            print("Building lifting surface with %5.4f chord scaling and %5.4f overall scaling..." %  (ChordFactor, ScaleFactor))
            LS, ActualSemiSpan, LSP_area, RootChord, AR, WingTip, Sections = self._BuildLS(ChordFactor, ScaleFactor)
            ObjFunction = self._LSMetrics(LS, ActualSemiSpan, LSP_area, ChordFactor, ScaleFactor)
        else:
            ObjFunction = 100000000
            print("Scale factor out of bounds - consider different starting values.")
            LS = []
            ActualSemiSpan = 0
            LSP_area = 0
            RootChord = 0
            AR = 0
            WingTip = []
            Sections = []

        if WingTip:
            rs.DeleteObject(WingTip)
        if LS:
            rs.DeleteObject(LS)
        if Sections:
            rs.DeleteObject(Sections)

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
            # However, SciPy is not supported on 64-bit Rhino installations, 
            # so here we use an alternative: a simple evolutionary optimizer
            # included with AirCONICStools.
            MaxIter = 50
            xtol = 0.025
            deltax = [x0[0]*0.25,x0[1]*0.25]
            x0, fopt = act.boxevopmin2d(self._LSObjective, x0, deltax, xtol, MaxIter)
            x0[0] = abs(x0[0])
            x0[1] = abs(x0[1])
            print("Optimum chord factor %5.3f, optimum scale factor %5.3f" % (x0[0], x0[1]))

        self.ChordFactor = x0[0]
        self.ScaleFactor = x0[1]

        LS, ActualSemiSpan, LSP_area,  RootChord, AR, WingTip, Sections = self._BuildLS(x0[0], x0[1])
        
        self._ClearConstructionGeometry()
        
        # Moving the wing into position
        AP = rs.AddPoint(self.ApexPoint)
        MoveVec = rs.VectorCreate(AP, (0,0,0))
        rs.MoveObject(LS, MoveVec)
        if WingTip:
            rs.MoveObject(WingTip, MoveVec)
        if self.SectionsRequired:
            rs.MoveObjects(Sections, MoveVec)
        else:
            Sections = []
        rs.DeleteObject(AP)

        # FINAL REPORT ON THE COMPLETED SURFACE
        SA  = rs.SurfaceArea(LS)
        print("Wing complete. Key features (all related to both wings):")
        print("Proj.area: %5.4f   Wet.area: %5.4f   Span:%5.4f  Aspect ratio:  %5.4f  Root chord: %5.4f" % (2*LSP_area, 2.0*SA[0], 2.0*ActualSemiSpan, AR, RootChord))


        return LS, ActualSemiSpan, LSP_area,  RootChord, AR, WingTip, Sections