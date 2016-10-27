import os
from osgeo import ogr, gdal, osr
import numpy as np

class CSVWriter():

    def __init__(self, csvPath):
        self.path = csvPath
        self.colNames = None
        self.nCols = None

    def SetColNames(self, colNamesList):
        self.colNames = colNamesList
        self.nCols = len(self.colNames)

    def WriteRow(self, rowData):
        #do stuff here
        print ''

class DamPoints():

    def __init__(self, outPath):
        self.path = outPath
        self.fieldNames = ["cr_len_m", "p_area_m2", "p_vol_m3", "p_wse", "b_wse", "p_slp_per", "cr_elev", "b_elev", "d_ht_m", "b_x", "b_y", "u_x", "u_y", "u_elev"]

    def CreateFields(self):
        print''

    def GetFieldNames(self):
        return self.fieldNames

    def WriteFieldValues(self, fieldData):
        #do stuff here
        print''

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
        self.fnDams = "dams.shp"
        self.fnCsv = "out.csv"
        self.SetDrivers()
        self.InitializeNewDirectory(dirPath)
        self.dams = DamPoints(self.fnDams)
        self.csv = CSVWriter(self.fnCsv)
        self.damData = [None] * len(self.dams.GetFieldNames())


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

    def CrestElevation(self, index):
        ds_dem = gdal.Open("DEM_crest"+str(index)+".tif", gdal.GA_ReadOnly)
        ds_wse = gdal.Open("WSE_crest"+str(index)+".tif", gdal.GA_ReadOnly)
        stat_dem = ds_dem.GetRasterBand(1).GetStatistics(0,1)
        stat_wse = ds_wse.GetRasterBand(1).GetStatistics(0,1)
        self.damData[self.dams.GetFieldNames().index("cr_elev")] = max([stat_dem[1], stat_wse[1]])

    def DamBaseData(self, index):
        ds_dem = gdal.Open("DEM_pond"+str(index)+".tif", gdal.GA_ReadOnly)
        ds_wse = gdal.Open("WSE_pond"+str(index)+".tif", gdal.GA_ReadOnly)
        ds_wd = gdal.Open("WD_pond"+str(index)+".tif", gdal.GA_ReadOnly)
        maxWD = ds_wd.GetRasterBand(1).GetStatistics(0,1)[1]
        dem_data = ds_dem.GetRasterBand(1).ReadAsArray()
        wse_data = ds_wse.GetRasterBand(1).ReadAsArray()
        wd_data = ds_wd.GetRasterBand(1).ReadAsArray()
        col = np.where(wd_data == maxWD)[0][0]
        row = np.where(wd_data == maxWD)[1][0]
        self.damData[self.dams.GetFieldNames().index("b_wse")] = wse_data[col, row]
        self.damData[self.dams.GetFieldNames().index("b_elev")] = dem_data[col, row]
        self.damData[self.dams.GetFieldNames().index("b_x")] = self.geot[0] + self.geot[1] * col
        self.damData[self.dams.GetFieldNames().index("b_y")] = self.geot[3] + self.geot[5] * row

    def DamHeight(self):
        b_elev = self.damData[self.dams.GetFieldNames().index("b_elev")]
        cr_elev = self.damData[self.dams.GetFieldNames().index("cr_elev")]
        if (b_elev != None) & (cr_elev != None):
            self.damData[self.dams.GetFieldNames().index("d_ht_m")] = (cr_elev - b_elev)

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
        self.SetGeotransform()
        self.SetSpatialRef()
        self.nDams = self.GetDamCount()

    def PondExtent(self, index):
        maxWSE = self.damData[self.dams.GetFieldNames().index("cr_elev")]
        ds_wse = gdal.Open("WSE_pond"+str(index)+".tif")
        ds_wd = gdal.Open("WD_pond"+str(index)+".tif")
        if os.path.exists("EX_pond"+str(index)+".tif"):
                self.driverTIF.Delete("EX_pond"+str(index)+".tif")
        ds_extent = self.driverTIF.Create("EX_pond"+str(index)+".tif", ds_wse.RasterXSize, ds_wse.RasterYSize, 1, gdal.GDT_Float32)
        ds_extent.SetProjection(ds_wse.GetProjection())
        wse_data = ds_wse.GetRasterBand(1).ReadAsArray()
        wd_data = ds_wd.GetRasterBand(1).ReadAsArray()
        ex_data = np.zeros(wse_data.shape, dtype=np.float32)
        ex_data[(wse_data > -9999.0) & (wse_data <= maxWSE) & (wd_data > 0.0)] = 1.0
        ex_data[ex_data <= 0.0] = 0.0
        ds_extent.GetRasterBand(1).WriteArray(ex_data)
        #ds_extent.GetRasterBand(1).SetNoDataValue(0.0)
        self.damData[self.dams.GetFieldNames().index("p_area_m2")] = np.sum(ex_data) * self.geot[1] * abs(self.geot[5])
        self.damData[self.dams.GetFieldNames().index("p_vol_m3")] = np.sum(np.multiply(ex_data, wd_data)) * self.geot[1] * abs(self.geot[5])
        wse_ave = np.multiply(ex_data, wse_data)
        wse_ave[wse_ave == 0.0] = np.nan
        self.damData[self.dams.GetFieldNames().index("p_wse")] = np.nanmean(wse_ave)

    def SetDrivers(self):
        self.driverSHP = ogr.GetDriverByName('ESRI Shapefile')
        self.driverTIF = gdal.GetDriverByName('GTiff')

    def SetGeotransform(self):
        ds = gdal.Open(self.fnDem, gdal.GA_ReadOnly)
        self.geot = ds.GetGeoTransform()
        self.inv_geot = gdal.InvGeoTransform(self.geot)

    def SetSpatialRef(self):
        ds = self.driverSHP.Open(self.fnCrest)
        lyr = ds.GetLayer()
        self.spatialRef = lyr.GetSpatialRef()

#########################################################
########################## RUN ##########################
#########################################################

path = r'F:\01_etal\Projects\Modeling\BeaverWaterStorage\wrk_Data\GIS_Data\PondSurveys\BridgeCreek\01_Processing\Test'
print 'path set'
extractor = MorphometryExtractor(path)
print 'extractor initialized'
extractor.ClipRasters()
print 'rasters clipped'
extractor.CrestElevation(1)
extractor.PondExtent(1)
extractor.DamBaseData(1)
extractor.DamHeight()
print extractor.dams.GetFieldNames()
print extractor.damData