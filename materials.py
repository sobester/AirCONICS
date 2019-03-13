act.AssignMaterial(PropDisk, "PropDisk")

try:
    act.AssignMaterial(CompleteNoseGear, "ShinyBlack")
except:
    act.AssignMaterial(TailGearPort, "ShinyBlack")
    act.AssignMaterial(TailGearStbd, "ShinyBlack")


act.AssignMaterial(MainGearPort, "ShinyBlack")
act.AssignMaterial(MainGearStbd, "ShinyBlack")

act.AssignMaterial(AirFrame, "BlueFoam")
act.AssignMaterial(SBoom, "ShinyBlack")
act.AssignMaterial(SBoom1, "ShinyBlack")
act.AssignMaterial(PBoom, "ShinyBlack")
act.AssignMaterial(PBoom1, "ShinyBlack")

try:
    act.AssignMaterial(CentrePod[0], "ShinyBABlueMetal")
except:
    pass
    
try:
    act.AssignMaterial(CentrePod[1], "ShinyBARedMetal")
except:
    pass

act.AssignMaterial(Tail, "BlueFoam")

try:
    act.AssignMaterial(SAileron, "GreenNylon")
    act.AssignMaterial(PAileron, "RedNylon")
except:
    pass

act.AssignMaterial(TailP1, "BlueFoam")
act.AssignMaterial(TailP2, "BlueFoam")
act.AssignMaterial(TailP3, "BlueFoam")
act.AssignMaterial(TailP4, "BlueFoam")

try:
    act.AssignMaterial(TailP5, "BlueFoam")
except:
    pass
    
try:
    act.AssignMaterial(TailP6, "BlueFoam")
except: