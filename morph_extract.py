import os
from qgis.core import *
from qgis.analysis import *



pathDir = r'F:\01_etal\Projects\Modeling\BeaverWaterStorage\wrk_Data\GIS_Data\PondSurveys\BridgeCreek\04_2_UprOwens_2015\ras'
pathDir2 = r'F:\01_etal\Projects\Modeling\BeaverWaterStorage\wrk_Data\GIS_Data\PondSurveys\BridgeCreek\04_2_UprOwens_2015\shp'
fnPolyPond = "dam1_crest_buf.shp"
fnPolyBase = "DEM.tif"
fnPointUS = ""
fnLineCrest= ""


def ClipRasterPolygon(rasterPath, polyPath, outPath):
    os.system("gdalwarp -q -cutline " + polyPath + " -tr 0.1 0.1 -of GTiff " + rasterPath + " " + outPath)

class MorphometryExtractor():

    def __init__(self, dirPath):
        self.dir = dirPath
        os.chdir(self.dir)
        self.fnDem = "DEM.tif"
        self.fnWse = "WSEDEM.tif"
        self.fnWd = "Water_Depth.tif"
        self.fnCrest = "crest.shp"
        self.fnCrestBuf = "crestbuf.shp"
        self.fnPolyPond = "polypond.shp"
        self.fnPolyBase = "polybase.shp"
        self.fnPointUS = "pointus.shp"
        self.fnPointBase = "pointbase.shp"
        self.nDams = None

    def BufferCrest(self):
        layer = QgsVectorLayer(self.fnCrest)
        outLayer = QgsVectorLayer(self.fnCrestBuf)
        iter = layer.getFeatures()
        i = 1
        for feature in iter:
            geom = feature.geometry
            buff = geom.buffer(0.3)
            newFeat = QgsFeature()
            newFeat.setGeometry(buff)
            outLayer.addFeature(newFeat)
            i += 1

    def ClipRasterPolygon(self, rasterPath, polyPath, outPath):
        os.system("gdalwarp -q -cutline " + polyPath + " -tr 0.1 0.1 -of GTiff " + rasterPath + " " + outPath)

    def GetDamCount(self):
        layer = QgsVectorLayer(self.fnCrest)
        self.nDams = layer.featureCount()

class CSVWriter():

    def __init__(self, csvPath):
        self.path = csvPath

class DamPoints():
    def __init__(self, outPath):
        self.path = outPath
        self.fieldNames = ["cr_len_m", "pond_a_m2", "pond_v_m3", "p_wse", "b_wse"]

