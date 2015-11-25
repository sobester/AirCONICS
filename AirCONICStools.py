# AirCONICStools.py ============================================================
# Ancillary methods called by the higher level AirCONICS functions. 
# ==============================================================================
# AirCONICS
# Aircraft CONfiguration through Integrated Cross-disciplinary Scripting 
# version 0.2.1
# Andras Sobester, 2015.
# Bug reports to a.sobester@soton.ac.uk or @ASobester please.
# ==============================================================================

# Preamble
from __future__ import division
import math, random, cmath, os, bisect, rhinoscriptsyntax as rs
import airconics_setup


def AddTEtoOpenAirfoil(AirfoilCurve):
# If the airfoil curve given as an argument is open at the trailing edge, it adds
# a line between the ends of the curve and joins this with the rest of the curve.
    if rs.IsCurveClosed(AirfoilCurve) == False:
        EP1 = rs.CurveEndPoint(AirfoilCurve)
        rs.ReverseCurve(AirfoilCurve)
        EP2 = rs.CurveEndPoint(AirfoilCurve)
        rs.ReverseCurve(AirfoilCurve)
        Closure = rs.AddLine(EP1,EP2)
        rs.UnselectAllObjects()
        rs.SelectObject(Closure)
        rs.SelectObject(AirfoilCurve)
        rs.Command("_Join ")
        LO = rs.LastCreatedObjects()
        AirfoilCurve = LO[0]
        rs.UnselectAllObjects()

    return AirfoilCurve

def ObjectsExtents(ObjectIds):
    # Compute the extents in the X, Y and Z direction (in the current coordinate
    # system) of the objects listed in the argument.

    BB = rs.BoundingBox(ObjectIds)
    
    XVec = []
    YVec = []
    ZVec = []

    for i,P in enumerate(BB):
        list.append(XVec,P.X)
        list.append(YVec,P.Y)
        list.append(ZVec,P.Z)
        
    Xmin = min(XVec)
    Ymin = min(YVec)
    Zmin = min(ZVec)
    
    Xmax = max(XVec)
    Ymax = max(YVec)
    Zmax = max(ZVec)
    
    return (Xmin,Ymin,Zmin,Xmax,Ymax,Zmax)

def MirrorObjectXZ(ObjectId):
# Mirrors an object with respect to the XoZ plane
# Argument: the object to be mirrored.
# Returns: the ID of the object if succesful
    TransMat = []
    TransMat.append([1, 0, 0, 0])
    TransMat.append([0, -1, 0,0])
    TransMat.append([0, 0, 1, 0])
    TransMat.append([0, 0, 0, 1])
    TransObjId = rs.TransformObject(ObjectId, TransMat,True)
    return TransObjId

def ScaleObjectWorld000(ObjectId, Scaling):
# Scales an object in the World coordinate system. Similar functionality to 
# Rhino's ScaleObject, but the latter uses the current construction plane.
# The scaling is done with respect to the origin of the World system (0,0,0)
# Arguments: ObjectId - the object to be scaled
#            Scaling  - a three element list or tuple containg the scaling
#                       factors along x, y and z respectively
    xform = rs.XformScale(Scaling)
    ObjectId = rs.TransformObjects(ObjectId, xform)
    return ObjectId

def frange(r1, r2, incr):
    # A float version of the built-in range method
    # NOTE - due to round-off issues this is due to be replaced in the next ver.
    while r1 <= r2:
        yield r1
        r1 += incr

def pwfrange(start, stop, n):
    # Immune to round-off problems
    L = [0.0] * n 
    nm1 = n - 1 
    nm1inv = 1.0 / nm1 
    for i in range(n): 
        L[i] = nm1inv * (start*(nm1 - i) + stop*i) 
    return L 

def CrossPlatformExtrudeSurface(SurfaceId, CurveId, Capped = True):
    # rs.ExtrudeSurface not implemented in Rhino for OS X

    if airconics_setup.RhinoVersion==1:
        SolidId = rs.ExtrudeSurface(SurfaceId, CurveId, Capped)
    else:
        rs.SelectObject(CurveId)
        rs.Command("_SelNone")
        rs.SelectObject(SurfaceId)
        rs.Command("_ExtrudeSrfAlongCrv _SelPrev")
        SolidId = rs.LastCreatedObjects()
        rs.Command("_SelNone")

    return SolidId

def blendcorners(polyline_id, radius):
    # Fillets the corners of a polyline (from the McNeel website)
    if not polyline_id: return

    vertices = rs.PolylineVertices(polyline_id)
    if not vertices: return

    if radius is None: return

    between = lambda a,b: (a+b)/2.0
    newverts = []
    for i in range(len(vertices)-1):
        a = vertices[i]
        b = vertices[i+1]
        segmentlength = rs.Distance(a, b)
        vec_segment = rs.PointSubtract(b, a)
        vec_segment = rs.VectorUnitize(vec_segment)

        if radius<(0.5*segmentlength):
            vec_segment = rs.VectorScale(vec_segment, radius)
        else:
            vec_segment = rs.VectorScale(vec_segment, 0.5*segmentlength)

        w1 = rs.PointAdd(a, vec_segment)
        w2 = rs.PointSubtract(b, vec_segment)
        newverts.append(a)
        newverts.append(between(a,w1))
        newverts.append(w1)
        newverts.append(between(w1,w2))
        newverts.append(w2)
        newverts.append(between(w2,b))
    newverts.append(vertices[len(vertices)-1])
    CrvId = rs.AddCurve(newverts, 5)
    rs.DeleteObject(polyline_id)
    return CrvId


def coslin(TransitionPoint):
    # Creates a series of abscissas with cosine spacing from 0 to a TransitionPoint
    # and a linear spacing thereafter, up to 1. The TransitionPoint corresponds to 
    # pi. Distribution suitable for airfoils defined by points. TransitionPoint
    # must be in the range [0,1].
    NCosPoints = 8
    NLinPoints = 8
    Abscissa = []
    for ang in frange(0.0, math.pi/2 + 0.001, math.pi/(2*(NCosPoints-1.0))):
        list.append(Abscissa, TransitionPoint*(1.0-math.cos(ang)))
    for stp in frange((1-TransitionPoint)/NLinPoints, 1.0 + 0.001 - TransitionPoint, (1-TransitionPoint)/NLinPoints):
        list.append(Abscissa, stp + TransitionPoint)
    return Abscissa, NCosPoints


# Cubic equation solver based on Simple Recipes in Python by W. Park (1999) ====
def polar(x, y, deg=0): # radian if deg=0; degree if deg=1
    from math import hypot, atan2, pi
    if deg:
        return hypot(x, y), 180.0 * atan2(y, x) / pi
    else:
        return hypot(x, y), atan2(y, x)

def cbrt(x):
    from math import pow
    if x >= 0: 
        return pow(x, 1.0/3.0)
    else:
        return -pow(abs(x), 1.0/3.0)

def quadratic(a, b, c=None):
    import math, cmath
    if c: # (ax^2 + bx + c = 0)
        a, b = b / float(a), c / float(a)
    t = a / 2.0
    r = t**2 - b
    if r >= 0: # real roots
        y1 = math.sqrt(r)
    else: # complex roots
        y1 = cmath.sqrt(r)
    y2 = -y1
    return y1 - t, y2 - t

def cubic(a, b, c, d=None):
    from math import cos
    if d: 
        a, b, c = b / float(a), c / float(a), d / float(a)
    t = a / 3.0
    p, q = b - 3 * t**2, c - b * t + 2 * t**3
    u, v = quadratic(q, -(p/3.0)**3)
    if type(u) == type(0j): # complex cubic root
        r, w = polar(u.real, u.imag)
        
        y1 = 2 * cbrt(r) * cos(w / 3.0)
    else:   # real root
        y1 = cbrt(u) + cbrt(v)
    y2, y3 = quadratic(y1, p + y1**2)
    return (y1 - t, y2 - t, y3 - t)
#===============================================================================

def linear_interpolation(x, y):
    # Returns a function that interpolates the data in the argument (linearly)
    x = x[:]
    y = y[:]
    def fn( v ) :
        j = bisect.bisect_left(x, v)
        i = j-1
        if i < 0 :
            return y[0]
        if j >= len(x) :
            return y[ -1 ]
        return y[i] + (v-x[i])*(y[j]-y[i])/(x[j]-x[i])
    return fn


def boxevopmin2d(funct, x0, deltax, xtol, MaxIter):
# A simple, 2d evolutionary optimizer to eliminate the need for scipy (only
# compatible at the moment on 32-bit Windows installations of Rhino)

    def _evopmatrix2d(x0,deltax):
    # Internal function - generates search matrix around current iteration
    
        x1 = [x0[0]+deltax[0],x0[1]]
        x2 = [x0[0]+deltax[0],x0[1]+deltax[1]]
        x3 = [x0[0]+deltax[0],x0[1]-deltax[1]]
        
        x4 = [x0[0],x0[1]+deltax[1]]
        x5 = [x0[0],x0[1]-deltax[1]]
        
        x6 = [x0[0]-deltax[0],x0[1]]
        x7 = [x0[0]-deltax[0],x0[1]+deltax[1]]
        x8 = [x0[0]-deltax[0],x0[1]-deltax[1]]
        
        return [x0, x1, x2, x3, x4, x5, x6, x7, x8] 

    Iter = 1
    f = []

    # Evaluate the starting point
    list.append(f,funct(x0))


    while (Iter < MaxIter): 
    
        print "EVOP optimizer iteration", Iter
    
        # Generate the eight points around it
        xmat = _evopmatrix2d(x0,deltax)
    
        # ...and evaluate them
        for i in range(1,9):
            list.append(f, funct(xmat[i]))
        # f and xmat should now contain 9 points
    
        m = min(f)
        mi = [i for i, j in enumerate(f) if j == m]
        # Point mi is the best and should form the basis of the next iteration

        x0 = xmat[mi[0]]
        f = []
        list.append(f, m)
        # Ready to construct new matrix around this point or finish
        
        if mi[0]==0:
            # We are stuck, step size reduction is needed
            # - cut both by the golden ratio
            deltax[0] = 0.618*deltax[0]
            deltax[1] = 0.618*deltax[1]
            
            print "Step size reduced to", deltax[0], deltax[1]
            
            if max(deltax) < xtol:
                # Trigger an exit if tolerance xtol is reached
                print "Step size is now below xtol - stopping" 
                Iter = MaxIter

        Iter = Iter + 1

        xmin = x0
        fmin = f[0]

    return xmin, fmin


def AssignMaterial(object_id, Material):
# Adds simple, pre-defined material surface property sets to objects. Note that
# these are simply visual properties, they do not make the model suitable for 
# centre of gravity, etc. calculations.  
    MatInd = rs.AddMaterialToObject(object_id)
    if Material=="White_composite_external":
        rs.MaterialColor(MatInd,(255,255,255))
        rs.MaterialShine(MatInd, 100)
        rs.MaterialTransparency(MatInd, 0)
        rs.MaterialReflectiveColor(MatInd, (255,255,255))
    elif Material=="Plexiglass":
        rs.MaterialColor(MatInd,(255,255,255))
        rs.MaterialShine(MatInd, 255)
        rs.MaterialTransparency(MatInd, 0.8)
    elif Material=="Skin":
        rs.MaterialColor(MatInd,(229,184,143))
        rs.MaterialShine(MatInd, 0)
        rs.MaterialTransparency(MatInd, 0)
    elif Material=="Panel":
        rs.MaterialColor(MatInd,(0,0,0))
        rs.MaterialShine(MatInd, 0)
        rs.MaterialTransparency(MatInd, 0)
    elif Material=="PropDisk":
        rs.MaterialColor(MatInd,(255,255,255))
        rs.MaterialShine(MatInd, 0)
        rs.MaterialTransparency(MatInd, 0.9)
    elif Material=="Structure":
        rs.MaterialColor(MatInd,(0,0,0))
        rs.MaterialShine(MatInd, 0)
        rs.MaterialTransparency(MatInd, 0)
    elif Material=="ShinyBABlueMetal":
        rs.MaterialColor(MatInd,(0,32,91))
        rs.MaterialShine(MatInd, 150)
        rs.MaterialTransparency(MatInd, 0)
    elif Material=="ShinyBARedMetal":
        rs.MaterialColor(MatInd,(218, 41, 28))
        rs.MaterialShine(MatInd, 150)
        rs.MaterialTransparency(MatInd, 0)
    elif Material=="UnpaintedMetal":
        rs.MaterialColor(MatInd,(188,198,204))
        rs.MaterialShine(MatInd, 30)
        rs.MaterialTransparency(MatInd, 0)
    elif Material=="FanDisk":
        rs.MaterialColor(MatInd,(0,0,0))
        rs.MaterialShine(MatInd, 30)
        rs.MaterialTransparency(MatInd, 0.2)
    elif Material=="ShinyBlack":
        rs.MaterialColor(MatInd,(0,0,0))
        rs.MaterialShine(MatInd, 120)
        rs.MaterialTransparency(MatInd, 0)

def CutSect(SurfaceId, SpanStation):
    # SpanStation is assumed to be along the y direction, in the range [0,1]
    
    (Xmin,Ymin,Zmin,Xmax,Ymax,Zmax) = ObjectsExtents(SurfaceId)
    
    YStation = Ymin + (Ymax-Ymin)*SpanStation
    OriginX = Xmin -1
    OriginZ = Zmin -1
    
    
    CutPlane = rs.PlaneFromPoints((OriginX, YStation, OriginZ), (Xmax+1, YStation, OriginZ), (OriginX, YStation, Zmax + 1))
    CutPlaneSrf = rs.AddPlaneSurface(CutPlane, max([(Xmax-Xmin),(Ymax-Ymin),(Zmax-Zmin)])+1, max([(Xmax-Xmin),(Ymax-Ymin),(Zmax-Zmin)])+1 )

    I = rs.IntersectBreps(CutPlaneSrf, SurfaceId)
    Section = I[0]
    rs.DeleteObject(CutPlaneSrf)

    (Xmin,Ymin,Zmin,Xmax,Ymax,Zmax) = ObjectsExtents(Section)
    
    
    # Find the apparent chord of the section (that is, the line connecting the fore
    # most and aftmost points on the curve
    DivPoints = rs.DivideCurve(Section, 200)

    Xs = []
    Ys = []
    Zs = []
    for DP in DivPoints:
        list.append(Xs, DP[0])
        list.append(Ys, DP[1])
        list.append(Zs, DP[2])
        
    val, idx = min((val, idx) for (idx, val) in enumerate(Xs))
    LeadingPoint = [Xs[idx], Ys[idx], Zs[idx]]
    val, idx = max((val, idx) for (idx, val) in enumerate(Xs))
    TrailingPoint = [Xs[idx], Ys[idx], Zs[idx]]
    
    Chord = rs.AddLine(TrailingPoint, LeadingPoint)

    return Section, Chord