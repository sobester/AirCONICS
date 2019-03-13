# Example script for curve2dat
import primitives, airconics_setup, AirCONICStools as act
import rhinoscriptsyntax as rs



def curve2dat(AfCurve, NPoints, DatFileOrChannelName, MoveLEtoZero = False, Channel = True, LocalDihedral = 0):
# Converts an airfoil curve into a Selig-style airfoil coordinate file.
# It assumes that the curve starts at the upper trailing edge - reverse 
# the orientation of the curve (ReverseCurve method) prior to calling this
# if this is not the case.

    PointList3D = rs.DivideCurve(AfCurve,NPoints)
    
    PointListX = []
    PointListZ = []
    for P in PointList3D:
        list.append(PointListX, P.X)
        list.append(PointListZ, P.Z)
    LEval, LEidx = min((val, idx) for (idx, val) in enumerate(PointListX))

    if MoveLEtoZero:
        for i in range(len(PointListX)):
            PointList3D[i].X = PointList3D[i].X - LEval

    if LocalDihedral!=0:
        RCentre  = [PointList3D[LEidx].X, PointList3D[LEidx].Y, PointList3D[LEidx].Z]
        ChordEnd = [PointList3D[LEidx].X + 1, PointList3D[LEidx].Y, PointList3D[LEidx].Z]
        DihedralRAxis    = rs.VectorCreate(RCentre, ChordEnd)
        rs.RotateObject(AfCurve, RCentre, -LocalDihedral, axis = DihedralRAxis)
        PointList3D = rs.DivideCurve(AfCurve,NPoints)
        rs.RotateObject(AfCurve, RCentre, LocalDihedral, axis = DihedralRAxis)


    if Channel:
        for P in PointList3D:
            DatFileOrChannelName.write("{:<15}{:<15}\n".format(P.X,P.Z))
    else:
        with open(DatFileOrChannelName, 'w') as f:
            f.write('%s\n' % (DatFileOrChannelName))
            for P in PointList3D:
                f.write("{:<15}{:<15}\n".format(P.X,P.Z))
        f.close()

    return PointList3D

if __name__ == "__main__":
    # now write out matching Harpoon cfg file
    
    #===============================================================================
    # NACA 4-digit parametric airfoil used as an example
    #===============================================================================
    
    LEPoint = (0, 0, 0)
    ChordLength = 1
    Rotation = 20
    Twist = 10
    
    # Instantiate class to set up a generic airfoil with these basic parameters
    Af = primitives.Airfoil(LEPoint, ChordLength, Rotation, Twist, EnforceSharpTE = False)
    
    SmoothingPasses = 1
    
    # Add NACA2212 airfoil curve to document and retrieve handles to it and its chord
    # MaxCamberPercChord = 2, MaxCamberLocTenthChord = 2, MaxThicknessPercChord = 12
    AfCurve,Chrd = primitives.Airfoil.AddNACA4(Af, 2, 2, 12, SmoothingPasses)
    
    #===============================================================================
    
    
    
    # You can specify a file name:
    PointListXZ = curve2dat(AfCurve,100,'my2212.dat', MoveLEtoZero = True, Channel = False, LocalDihedral=-20)
    
    ##...or an open channel
    #ff = open('test.dat','w')
    #PointListXZ = curve2dat(AfCurve,100,ff, MoveLEtoZero = True, Channel = True, LocalDihedral)
    #ff.close()