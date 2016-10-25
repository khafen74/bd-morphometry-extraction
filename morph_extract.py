import os
from osgeo import ogr, gdal



pathDir = r'F:\01_etal\Projects\Modeling\BeaverWaterStorage\wrk_Data\GIS_Data\PondSurveys\BridgeCreek\04_2_UprOwens_2015\ras'
pathDir2 = r'F:\01_etal\Projects\Modeling\BeaverWaterStorage\wrk_Data\GIS_Data\PondSurveys\BridgeCreek\04_2_UprOwens_2015\shp'
fnPolyPond = "dam1_crest_buf.shp"
fnPolyBase = "DEM.tif"
fnPointUS = ""
fnLineCrest= ""


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
        self.SetDrivers()
        self.nDams = self.GetDamCount()

    def BufferCrest(self):
        source = self.driverSHP.Open(self.fnCrest)
        if source is None:
            print 'layer not open'
        layer = source.GetLayer()
        if os.path.exists(self.fnCrestBuf):
            self.driverSHP.DeleteDataSource(self.fnCrestBuf)
        outds = self.driverSHP.CreateDataSource(self.fnCrestBuf)
        outlyr = outds.CreateLayer(self.fnCrestBuf, geom_type=ogr.wkbPolygon)
        outDfn = outlyr.GetLayerDefn()
        print 'starting buffer loop'
        for feature in layer:
            ingeom = feature.GetGeometryRef()
            geomBuffer = ingeom.Buffer(0.3)
            outFeat = ogr.Feature(outDfn)
            outFeat.SetGeometry(geomBuffer)
            outlyr.CreateFeature(outFeat)


    def ClipRasterPolygon(self, rasterPath, polyPath, outPath):
        os.system("gdalwarp -q -cutline " + polyPath + " -tr 0.1 0.1 -of GTiff " + rasterPath + " " + outPath)

    def GetDamCount(self):
        source = self.driverSHP.Open(self.fnCrest)
        if source is None:
            print 'layer not open'
        layer = source.GetLayer()
        return layer.GetFeatureCount()

    def SetDrivers(self):
        self.driverSHP = ogr.GetDriverByName('ESRI Shapefile')

class CSVWriter():

    def __init__(self, csvPath):
        self.path = csvPath

class DamPoints():
    def __init__(self, outPath):
        self.path = outPath
        self.fieldNames = ["cr_len_m", "pond_a_m2", "pond_v_m3", "p_wse", "b_wse"]

#########################################################
########################## RUN ##########################
#########################################################

path = r'F:\01_etal\Projects\Modeling\BeaverWaterStorage\wrk_Data\GIS_Data\PondSurveys\BridgeCreek\01_Processing\Test'
print 'path set'
extractor = MorphometryExtractor(path)
print 'extractor initialized'
extractor.BufferCrest()
print 'buffer executed'