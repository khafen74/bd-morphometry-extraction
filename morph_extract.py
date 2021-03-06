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

    def __init__(self, outPath, spatialRef):
        self.path = outPath
        self.spatialRef = spatialRef
        self.fieldNames = ["cr_len_m", "p_area_m2", "p_vol_m3", "wd_max", "p_min", "p_wse", "b_wse", "head_diff", "cr_elev", "b_elev", "d_ht_m", "b_x", "b_y", "u_x", "u_y", "u_elev", "p_slp",
                           "max_area", "max_vol", "max_dep", "slp_ave", "relief", "dem_std"]
        self.SetDrivers()
        self.CreateShapefile()
        self.CreateFields()

    def CreateFeature(self, x, y, fieldData):
        defn = self.lyr.GetLayerDefn()
        feat = ogr.Feature(defn)
        point = ogr.Geometry(ogr.wkbPoint)
        point.AddPoint(x, y)
        feat.SetGeometry(point)
        i=0
        for name in self.fieldNames:
            if fieldData[i] is None:
                fieldData[i] = -9999.0
            feat.SetField(str(name), float(fieldData[i]))
            i+=1
        self.lyr.CreateFeature(feat)

    def CreateFields(self):
        for name in self.fieldNames:
            field = ogr.FieldDefn(name, ogr.OFTReal)
            self.lyr.CreateField(field)

    def CreateShapefile(self):
        if os.path.exists(self.path):
                self.driverSHP.DeleteDataSource(self.path)
        self.ds_out = self.driverSHP.CreateDataSource(self.path)
        self.lyr = self.ds_out.CreateLayer(self.path, srs=self.spatialRef, geom_type=ogr.wkbPoint)

    def GetFieldNames(self):
        return self.fieldNames

    def SetDrivers(self):
        self.driverSHP = ogr.GetDriverByName('ESRI Shapefile')

class DataPrepper():

    def __init__(self):
        self.fnDem = "DEM.tif"
        self.fnWse = "WSEDEM.tif"
        self.fnWd = "Water_Depth.tif"
        self.driverTIF = gdal.GetDriverByName('GTiff')

    def CreateMissingRasters(self, dirPath):
        self.SetDir(dirPath)
        if os.path.exists(self.fnDem):
            self.SetGeot()
            if not os.path.exists("DEMHillshade.tif"):
                os.system("gdaldem hillshade -of GTiff " + self.fnDem + " " + "DEMHillshade.tif")
            if not os.path.exists("slope.tif"):
                os.system("gdaldem slope -of GTiff " + self.fnDem + " " + "slope.tif")
            if not os.path.exists(self.fnWd) and os.path.exists(self.fnWse):
                dem = gdal.Open(self.fnDem, gdal.GA_ReadOnly)
                wse = gdal.Open(self.fnWse, gdal.GA_ReadOnly)
                dem_data = dem.GetRasterBand(1).ReadAsArray()
                wse_data = wse.GetRasterBand(1).ReadAsArray()
                wd_data = np.subtract(wse_data, dem_data)
                wd_data[wd_data <= 0.0] = -9999.0
                wd = self.driverTIF.Create(self.fnWd, dem.RasterXSize, dem.RasterYSize, 1, gdal.GDT_Float32)
                wd.SetGeoTransform(self.geot)
                wd.SetProjection(dem.GetProjection())
                wd.GetRasterBand(1).SetNoDataValue(-9999.0)
                wd.GetRasterBand(1).WriteArray(wd_data)

    def SetDir(self, dirPath):
        os.chdir(dirPath)

    def SetGeot(self):
        ds = gdal.Open(self.fnDem, gdal.GA_ReadOnly)
        self.geot = ds.GetGeoTransform()
        self.inv_geot = gdal.InvGeoTransform(self.geot)

class MorphometryExtractor():

    def __init__(self, outShpPath):
        self.wse = False
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
        self.fnDams = outShpPath
        self.fnCsv = "out.csv"
        self.dams = None
        self.SetDrivers()

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
            geomBuffer = ingeom.Buffer(0.25)
            outFeat = ogr.Feature(outDfn)
            outFeat.SetGeometry(geomBuffer)
            outlyr.CreateFeature(outFeat)
            i += 1

    def ClearDamData(self):
        self.damData = [None] * len(self.dams.GetFieldNames())

    def ClipRasters(self):
        self.DeleteExistingRasters()
        self.CreateClippingPolygons(self.fnPolyPond, self.fnPolyPondOut)
        self.CreateClippingPolygons(self.fnPolyBase, self.fnPolyBaseOut)
        self.BufferCrest()

        for i in range(1,self.nDams+1,1):
            self.ClipRasterPolygon(self.fnDem, self.fnPolyBaseOut+str(i)+".shp", "DEM_base"+str(i)+".tif")
            self.ClipRasterPolygon(self.fnDem, self.fnCrestBuf+str(i)+".shp", "DEM_crest"+str(i)+".tif")
            if os.path.exists(self.fnPolyPondOut+str(i)+".shp"):
                self.ClipRasterPolygon(self.fnDem, self.fnPolyPondOut+str(i)+".shp", "DEM_pond"+str(i)+".tif")
                self.ClipRasterPolygon("slope.tif", self.fnPolyPondOut+str(i)+".shp", "slope_pond"+str(i)+".tif")
                if os.path.exists(self.fnWse):
                    self.ClipRasterPolygon(self.fnWse, self.fnPolyPondOut+str(i)+".shp", "WSE_pond"+str(i)+".tif")
                if os.path.exists(self.fnWd):
                    self.ClipRasterPolygon(self.fnWd, self.fnPolyPondOut+str(i)+".shp", "WD_pond"+str(i)+".tif")
            if os.path.exists(self.fnWse):
                self.ClipRasterPolygon(self.fnWse, self.fnPolyBaseOut+str(i)+".shp", "WSE_base"+str(i)+".tif")
                self.ClipRasterPolygon(self.fnWse, self.fnCrestBuf+str(i)+".shp", "WSE_crest"+str(i)+".tif")
            if os.path.exists(self.fnWd):
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
        self.wse = False
        ds_dem = gdal.Open("DEM_crest"+str(index)+".tif", gdal.GA_ReadOnly)
        stat_dem = ds_dem.GetRasterBand(1).GetStatistics(0,1)
        if os.path.exists("WSE_crest"+str(index)+".tif"):
            ds_wse = gdal.Open("WSE_crest"+str(index)+".tif", gdal.GA_ReadOnly)
            stat_wse = ds_wse.GetRasterBand(1).GetStatistics(0,1)
            if sum(stat_wse) > 0.0:
                self.wse = True
            self.damData[self.dams.GetFieldNames().index("cr_elev")] = max([stat_dem[1], stat_wse[1]])
        else:
            self.damData[self.dams.GetFieldNames().index("cr_elev")] = stat_dem[1]

    def CrestLength(self, index):
        src = self.driverSHP.Open(self.fnCrest)
        if src is None:
            print 'layer not open'
        lyr = src.GetLayer()
        feat = lyr.GetFeature(index-1)
        geom = feat.GetGeometryRef()
        length = geom.Length()
        self.damData[self.dams.GetFieldNames().index("cr_len_m")] = length

    def DamBaseData(self, index):
        ds_dem = gdal.Open("DEM_base"+str(index)+".tif", gdal.GA_ReadOnly)
        geot = ds_dem.GetGeoTransform()
        minDem = ds_dem.GetRasterBand(1).GetStatistics(0,1)[0]
        dem_data = ds_dem.GetRasterBand(1).ReadAsArray()
        row = np.where(dem_data == minDem)[0][0]
        col = np.where(dem_data == minDem)[1][0]
        self.damData[self.dams.GetFieldNames().index("b_elev")] = dem_data[row, col]
        self.damData[self.dams.GetFieldNames().index("b_x")] = geot[0] + geot[1] * col
        self.damData[self.dams.GetFieldNames().index("b_y")] = geot[3] + geot[5] * row
        if os.path.exists("WSE_base"+str(index)+".tif"):
            ds_wse = gdal.Open("WSE_base"+str(index)+".tif", gdal.GA_ReadOnly)
            wsgeot = ds_wse.GetGeoTransform()
            newRow = round((wsgeot[3] - self.damData[self.dams.GetFieldNames().index("b_y")]) / abs(wsgeot[5]))
            newCol = round((self.damData[self.dams.GetFieldNames().index("b_x")] - wsgeot[0]) / abs(wsgeot[1]))
            wse_data = ds_wse.GetRasterBand(1).ReadAsArray()
            self.damData[self.dams.GetFieldNames().index("b_wse")] = wse_data[newRow, newCol]

    def DamHeight(self):
        b_elev = self.damData[self.dams.GetFieldNames().index("b_elev")]
        cr_elev = self.damData[self.dams.GetFieldNames().index("cr_elev")]
        if (b_elev != None) & (cr_elev != None):
            self.damData[self.dams.GetFieldNames().index("d_ht_m")] = (cr_elev - b_elev)

    def DeleteExistingRasters(self):
        for i in range(1,self.nDams+1,1):
            if os.path.exists("DEM_pond"+str(i)+".tif"):
                self.driverTIF.Delete("DEM_pond"+str(i)+".tif")
            if os.path.exists("slope_pond"+str(i)+".tif"):
                self.driverTIF.Delete("slope_pond"+str(i)+".tif")
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

    def HeadDifference(self):
        b_wse = self.damData[self.dams.GetFieldNames().index("b_wse")]
        p_wse = self.damData[self.dams.GetFieldNames().index("p_wse")]
        if (b_wse != None) & (p_wse != None) & (p_wse > 0.0):
            self.damData[self.dams.GetFieldNames().index("head_diff")] = (p_wse - b_wse)
        else:
            self.damData[self.dams.GetFieldNames().index("head_diff")] = -9999.0

    def InitializeNewDirectory(self, dirPath):
        self.dir = dirPath
        os.chdir(self.dir)
        if os.path.exists(self.fnDem) and os.path.exists(self.fnCrest):
            self.SetGeotransform()
            self.SetSpatialRef()
            self.nDams = self.GetDamCount()
            if self.dams is None:
                if os.path.exists(self.fnDams):
                    self.driverSHP.DeleteDataSource(self.fnDams)
                self.dams = DamPoints(self.fnDams, self.spatialRef)
            self.csv = CSVWriter(self.fnCsv)
            self.ClearDamData()

    def PondExtent(self, index):
        if self.damData[self.dams.GetFieldNames().index("cr_elev")] is not None and os.path.exists("DEM_pond"+str(index)+".tif"):
            wd = False
            wse = False
            maxWSE = self.damData[self.dams.GetFieldNames().index("cr_elev")]
            ds_dem = gdal.Open("DEM_pond"+str(index)+".tif")
            dem_data = ds_dem.GetRasterBand(1).ReadAsArray()
            if os.path.exists("EX_pond"+str(index)+".tif"):
                    self.driverTIF.Delete("EX_pond"+str(index)+".tif")
            ds_extent = self.driverTIF.Create("EX_pond"+str(index)+".tif", ds_dem.RasterXSize, ds_dem.RasterYSize, 1, gdal.GDT_Float32)
            ds_extent.SetProjection(ds_dem.GetProjection())
            ds_extent.SetGeoTransform(ds_dem.GetGeoTransform())
            if os.path.exists("WD_pond"+str(index)+".tif"):
                wd = True
                ds_wd = gdal.Open("WD_pond"+str(index)+".tif")
                wd_data = ds_wd.GetRasterBand(1).ReadAsArray()
            if os.path.exists("WSE_pond"+str(index)+".tif") and self.wse:
                wse = True
                ds_wse = gdal.Open("WSE_pond"+str(index)+".tif")
                wse_data = ds_wse.GetRasterBand(1).ReadAsArray()
                ex_data = np.zeros(wse_data.shape, dtype=np.float32)
                wse_ave = np.multiply(ex_data, wse_data)
                wse_ave[wse_ave == 0.0] = np.nan
            if wse and wd:
                ex_data[(wse_data > -9999.0) & (wse_data <= maxWSE) & (wd_data > 0.0)] = 1.0
                ex_data[ex_data <= 0.0] = 0.0
                self.damData[self.dams.GetFieldNames().index("p_area_m2")] = np.sum(ex_data) * self.geot[1] * abs(self.geot[5])
                self.damData[self.dams.GetFieldNames().index("p_vol_m3")] = np.sum(np.multiply(ex_data, wd_data)) * self.geot[1] * abs(self.geot[5])
                self.damData[self.dams.GetFieldNames().index("wd_max")] = np.max(wd_data)
                self.damData[self.dams.GetFieldNames().index("p_wse")] = np.max(np.multiply(ex_data, wse_data))
            ex_data = np.zeros(dem_data.shape, dtype=np.float32)
            ex_data[(dem_data > -9999.0) & (dem_data <= maxWSE)] = 1.0
            wd_data = np.subtract(maxWSE, dem_data)
            wd_data[(wd_data < 0.0) | (wd_data > 100.0)] = 0.0
            ex_data[ex_data <= 0.0] = 0.0
            ds_extent.GetRasterBand(1).WriteArray(ex_data)
            ds_extent.GetRasterBand(1).SetNoDataValue(0.0)
            self.damData[self.dams.GetFieldNames().index("max_area")] = np.sum(ex_data) * self.geot[1] * abs(self.geot[5])
            self.damData[self.dams.GetFieldNames().index("max_vol")] = np.sum(np.multiply(ex_data, wd_data)) * self.geot[1] * abs(self.geot[5])
            self.damData[self.dams.GetFieldNames().index("max_dep")] = np.max(wd_data)

            if ex_data.shape == dem_data.shape:
                dem_pond = np.multiply(ex_data, dem_data)
                dem_pond[dem_pond <= 0.0] = np.nan
                self.damData[self.dams.GetFieldNames().index("p_min")] = np.nanmin(dem_pond)
            else:
                self.damData[self.dams.GetFieldNames().index("p_min")] = -9999.0

    def PondSlope(self, index):
        x1 = self.damData[self.dams.GetFieldNames().index("b_x")]
        y1 = self.damData[self.dams.GetFieldNames().index("b_y")]
        x2 = self.damData[self.dams.GetFieldNames().index("u_x")]
        y2 = self.damData[self.dams.GetFieldNames().index("u_y")]
        z1 = self.damData[self.dams.GetFieldNames().index("b_elev")]
        z2 = self.damData[self.dams.GetFieldNames().index("u_elev")]
        if x2 is not None and y2 is not None and z1 is not None and z2 is not None and z2 > 0.0:
            dist = ((x2-x1)**2 + (y2-y1)**2)**0.5
            slp = (z2-z1) / dist
            self.damData[self.dams.GetFieldNames().index("p_slp")] = slp
        if os.path.exists("slope_pond"+str(index)+".tif"):
            ds_ex = gdal.Open("EX_pond"+str(index)+".tif")
            ds_slp = gdal.Open("slope_pond"+str(index)+".tif")
            ex_data = ds_ex.GetRasterBand(1).ReadAsArray()
            slp_data = ds_slp.GetRasterBand(1).ReadAsArray()
            slp_data[(ex_data <= 0.0) | (slp_data < 0.0)] = np.nan
            self.damData[self.dams.GetFieldNames().index("slp_ave")] = np.nanmean(slp_data)

    def Relief(self, index):
        if os.path.exists("DEM_pond"+str(index)+".tif"):
            ds_ex = gdal.Open("EX_pond"+str(index)+".tif")
            ds_dem = gdal.Open("DEM_pond"+str(index)+".tif")
            ex_data = ds_ex.GetRasterBand(1).ReadAsArray()
            dem_data = ds_dem.GetRasterBand(1).ReadAsArray()
            dem_data[(ex_data <= 0.0) | (dem_data < 0.0)] = np.nan
            self.damData[self.dams.GetFieldNames().index("dem_std")] = np.nanstd(dem_data)
            self.damData[self.dams.GetFieldNames().index("relief")] = np.nanmax(dem_data) - np.nanmin(dem_data)

    def Run(self):
        if os.path.exists(self.fnCrest) and os.path.exists(self.fnPolyPond) and os.path.exists(self.fnPolyBase) and os.path.exists(self.fnPolyBase):
            print "running "+self.dir
            self.ClipRasters()
            print 'done clipping'
            for i in range(1, self.nDams+1, 1):
                self.CrestElevation(i)
                print "crest done"
                self.PondExtent(i)
                print "pond extent done"
                self.DamBaseData(i)
                print "base done"
                self.DamHeight()
                print "height done"
                self.HeadDifference()
                print "height done"
                self.CrestLength(i)
                print "crest length done"
                self.UpstreamExtentData(i)
                print "upstream done"
                self.PondSlope(i)
                print "slope done"
                self.Relief(i)
                print "relief done"
                self.dams.CreateFeature(self.damData[self.dams.GetFieldNames().index("b_x")], self.damData[self.dams.GetFieldNames().index("b_y")], self.damData)
                print "feature created"
                self.ClearDamData()
                print str(i) + " done"
        else:
            print self.dir + " does not contain necessary input shapefiles"

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

    def UpstreamExtentData(self, index):
        src = self.driverSHP.Open(self.fnPointUS)
        if src is None:
            print 'layer not open'
        lyr = src.GetLayer()
        if ((index-1) < lyr.GetFeatureCount()):
            feat = lyr.GetFeature(index-1)
            geom = feat.GetGeometryRef()
            x = geom.GetX()
            y = geom.GetY()
            self.damData[self.dams.GetFieldNames().index("u_x")] = x
            self.damData[self.dams.GetFieldNames().index("u_y")] = y
            col, row = gdal.ApplyGeoTransform(self.inv_geot, x, y)
            ds = gdal.Open(self.fnDem, gdal.GA_ReadOnly)
            dem = ds.GetRasterBand(1).ReadAsArray()
            self.damData[self.dams.GetFieldNames().index("u_elev")] = dem[int(row), int(col)]

#########################################################
########################## RUN ##########################
#########################################################

path = r'F:\01_etal\Projects\Modeling\BeaverWaterStorage\wrk_Data\GIS_Data\PondSurveys\BridgeCreek\01_Processing\2013'
#path = r'F:\01_etal\Projects\Modeling\BeaverWaterStorage\wrk_Data\GIS_Data\PondSurveys\SpawnCreek\01_Processing\2009'
#path = r'F:\01_etal\Projects\Modeling\BeaverWaterStorage\wrk_Data\GIS_Data\PondSurveys\CurtisCreek\01_Processing\2012'
pathShp = r'F:\01_etal\Projects\Modeling\BeaverWaterStorage\wrk_Data\GIS_Data\PondSurveys\BridgeCreek\01_Processing\2013\damsout.shp'
#pathShp = r'F:\01_etal\Projects\Modeling\BeaverWaterStorage\wrk_Data\GIS_Data\PondSurveys\SpawnCreek\01_Processing\2009\damsout.shp'
#pathShp = r'F:\01_etal\Projects\Modeling\BeaverWaterStorage\wrk_Data\GIS_Data\PondSurveys\CurtisCreek\01_Processing\2012\damsout.shp'
print 'path set'
# prep = DataPrepper()
# for subdir, dirs, files in os.walk(path):
#     print subdir
#     prep.CreateMissingRasters(subdir)

extractor = MorphometryExtractor(pathShp)
for subdir, dirs, files in os.walk(path):
    extractor.InitializeNewDirectory(subdir)
    extractor.Run()
print 'done'

#extractor = MorphometryExtractor(path2)
#prep = DataPrepper(path2)
#print 'extractor initialized'
#prep.CreateMissingRasters()
#extractor.Run()
#print extractor.dams.GetFieldNames()
#print extractor.damData