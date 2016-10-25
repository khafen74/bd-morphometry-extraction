import os
from osgeo import ogr, gdal, osr

class MorphometryExtractor():

    def __init__(self, dirPath):
        self.fnDem = "DEM.tif"
        self.fnWse = "WSEDEM.tif"
        self.fnWd = "Water_Depth.tif"
        self.fnCrest = "crest.shp"
        self.fnCrestBuf = "crestbuf"
        self.fnPolyPond = "polypond.shp"
        self.fnPolyPondOut = "polypond"
        self.fnPolyBase = "polybase.shp"
        self.fnPolyBaseOut = "polybase"
        self.fnPointUS = "pointus.shp"
        self.fnPointBase = "pointbase.shp"
        self.SetDrivers()
        self.InitializeNewDirectory(dirPath)


    def BufferCrest(self):
        source = self.driverSHP.Open(self.fnCrest)
        if source is None:
            print 'layer not open'
        layer = source.GetLayer()
        i=1

        for feature in layer:
            if os.path.exists(self.fnCrestBuf+str(i)+".shp"):
                self.driverSHP.DeleteDataSource(self.fnCrestBuf+str(i)+".shp")
            outds = self.driverSHP.CreateDataSource(self.fnCrestBuf+str(i)+".shp")
            outlyr = outds.CreateLayer(self.fnCrestBuf, srs=self.spatialRef, geom_type=ogr.wkbPolygon)
            outDfn = outlyr.GetLayerDefn()
            ingeom = feature.GetGeometryRef()
            geomBuffer = ingeom.Buffer(0.3)
            outFeat = ogr.Feature(outDfn)
            outFeat.SetGeometry(geomBuffer)
            outlyr.CreateFeature(outFeat)
            i += 1

    def ClipRasters(self):
        self.DeleteExistingRasters()
        self.CreateClippingPolygons(self.fnPolyPond, self.fnPolyPondOut)
        self.CreateClippingPolygons(self.fnPolyBase, self.fnPolyBaseOut)
        self.BufferCrest()

        for i in range(1,self.nDams+1,1):
            self.ClipRasterPolygon(self.fnDem, self.fnPolyPondOut+str(i)+".shp", "DEM_pond"+str(i)+".tif")
            self.ClipRasterPolygon(self.fnDem, self.fnPolyBaseOut+str(i)+".shp", "DEM_base"+str(i)+".tif")
            self.ClipRasterPolygon(self.fnDem, self.fnCrestBuf+str(i)+".shp", "DEM_crest"+str(i)+".tif")
            self.ClipRasterPolygon(self.fnWse, self.fnPolyPondOut+str(i)+".shp", "WSE_pond"+str(i)+".tif")
            self.ClipRasterPolygon(self.fnWse, self.fnPolyBaseOut+str(i)+".shp", "WSE_base"+str(i)+".tif")
            self.ClipRasterPolygon(self.fnWse, self.fnCrestBuf+str(i)+".shp", "WSE_crest"+str(i)+".tif")
            self.ClipRasterPolygon(self.fnWd, self.fnPolyPondOut+str(i)+".shp", "WD_pond"+str(i)+".tif")
            self.ClipRasterPolygon(self.fnWd, self.fnPolyBaseOut+str(i)+".shp", "WD_base"+str(i)+".tif")
            self.ClipRasterPolygon(self.fnWd, self.fnCrestBuf+str(i)+".shp", "WD_crest"+str(i)+".tif")

    def ClipRasterPolygon(self, rasterPath, polyPath, outPath):
        os.system("gdalwarp -dstnodata -9999 -q -cutline " + polyPath + " -tr 0.1 0.1 -of GTiff " + rasterPath + " " + outPath)

    def CreateClippingPolygons(self, inPath, outPath):
        ds = self.driverSHP.Open(inPath)
        if ds is None:
            print 'layer not open'
        lyr = ds.GetLayer()

        i=1
        for feature in lyr:
            outds = self.driverSHP.CreateDataSource(outPath+str(i)+".shp")
            outlyr = outds.CreateLayer(self.fnCrestBuf, srs=self.spatialRef, geom_type=ogr.wkbPolygon)
            outDfn = outlyr.GetLayerDefn()
            ingeom = feature.GetGeometryRef()
            outFeat = ogr.Feature(outDfn)
            outFeat.SetGeometry(ingeom)
            outlyr.CreateFeature(outFeat)
            i += 1

    def DeleteExistingRasters(self):
        for i in range(1,self.nDams+1,1):
            if os.path.exists("DEM_pond"+str(i)+".tif"):
                self.driverTIF.Delete("DEM_pond"+str(i)+".tif")
            if os.path.exists("DEM_base"+str(i)+".tif"):
                self.driverTIF.Delete("DEM_base"+str(i)+".tif")
            if os.path.exists("DEM_crest"+str(i)+".tif"):
                self.driverTIF.Delete("DEM_crest"+str(i)+".tif")
            if os.path.exists("WSE_pond"+str(i)+".tif"):
                self.driverTIF.Delete("WSE_pond"+str(i)+".tif")
            if os.path.exists("WSE_base"+str(i)+".tif"):
                self.driverTIF.Delete("WSE_base"+str(i)+".tif")
            if os.path.exists("WSE_crest"+str(i)+".tif"):
                self.driverTIF.Delete("WSE_crest"+str(i)+".tif")
            if os.path.exists("WD_pond"+str(i)+".tif"):
                self.driverTIF.Delete("WD_pond"+str(i)+".tif")
            if os.path.exists("WD_base"+str(i)+".tif"):
                self.driverTIF.Delete("WD_base"+str(i)+".tif")
            if os.path.exists("WD_crest"+str(i)+".tif"):
                self.driverTIF.Delete("WD_crest"+str(i)+".tif")

    def GetDamCount(self):
        source = self.driverSHP.Open(self.fnCrest)
        if source is None:
            print 'layer not open'
        layer = source.GetLayer()
        return layer.GetFeatureCount()

    def InitializeNewDirectory(self, dirPath):
        self.dir = dirPath
        os.chdir(self.dir)
        self.SetSpatialRef()
        self.nDams = self.GetDamCount()

    def SetDrivers(self):
        self.driverSHP = ogr.GetDriverByName('ESRI Shapefile')
        self.driverTIF = gdal.GetDriverByName('GTiff')

    def SetSpatialRef(self):
        ds = self.driverSHP.Open(self.fnCrest)
        lyr = ds.GetLayer()
        self.spatialRef = lyr.GetSpatialRef()

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
extractor.ClipRasters()
print 'rasters clipped'