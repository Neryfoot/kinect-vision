import vtk
from numpy import random
import numpy as np
import cv2
import frame_convert2
import freenect


def get_depth():
    return frame_convert2.pretty_depth_cv(freenect.sync_get_depth()[0])

class VtkPointCloud:

    def __init__(self, zMin=-10.0, zMax=10.0, maxNumPoints=2e6):
        self.maxNumPoints = maxNumPoints
        self.vtkPolyData = vtk.vtkPolyData()
        self.clearPoints()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.vtkPolyData)
        mapper.SetColorModeToDefault()
        mapper.SetScalarRange(zMin, zMax)
        mapper.SetScalarVisibility(1)
        self.vtkActor = vtk.vtkActor()
        self.vtkActor.SetMapper(mapper)

    def addPoint(self, point):
        if self.vtkPoints.GetNumberOfPoints() < self.maxNumPoints:
            pointId = self.vtkPoints.InsertNextPoint(point[:])
            self.vtkDepth.InsertNextValue(point[2])
            self.vtkCells.InsertNextCell(1)
            self.vtkCells.InsertCellPoint(pointId)
        else:
            r = random.randint(0, self.maxNumPoints)
            self.vtkPoints.SetPoint(r, point[:])
        self.vtkCells.Modified()
        self.vtkPoints.Modified()
        self.vtkDepth.Modified()

    def clearPoints(self):
        self.vtkPoints = vtk.vtkPoints()
        self.vtkCells = vtk.vtkCellArray()
        self.vtkDepth = vtk.vtkDoubleArray()
        self.vtkDepth.SetName('DepthArray')
        self.vtkPolyData.SetPoints(self.vtkPoints)
        self.vtkPolyData.SetVerts(self.vtkCells)
        self.vtkPolyData.GetPointData().SetScalars(self.vtkDepth)
        self.vtkPolyData.GetPointData().SetActiveScalars('DepthArray')


pointCloud = VtkPointCloud()

#f=open("3darray.txt", "r")
#points=f.readlines()

#Intrinsic parameters of kinect
cx = 316.0
cy = 246.7
fx = 585.0
fy = 585.0

def get_depth():
    return frame_convert2.pretty_depth_cv(freenect.sync_get_depth()[0])

#Matrix parameters for 3d reconstruction
P = np.array([[fx, 0, cx],   [0, fy, cy],    [0, 0, 1]])

#Inverse of matrix parameters
IP= np.linalg.inv(P)

#function that gets the x,y,z cartesian coordinates of a single pixel
def depTo3dpoint(v,u,s):
    if s<4.2:
        dpcor=[u*s,-v*s,s]
        coordinates=np.dot(IP,dpcor)
    else:
        coordinates=[0,0,0]
    return coordinates


#function goes through entire image converting depth points to 3d coordinates
def depthTo3d(img):
    img=img/float(51)
    points=np.zeros((307200/skip,3))
    for i in range(0,480/skip):
        for j in range(0,640/skip):
          points[640/skip*i+j]=depTo3dpoint(i*skip,j*skip,img[i*skip][j*skip])
    return points

skip=2

img=get_depth()
cv2.imwrite("./D.jpg", img)
triimg=np.zeros((307200/skip,3))
triimg=depthTo3d(img)

for i in range(0,len(triimg)):
    point = triimg[i]
    pointCloud.addPoint(point)


# Renderer
renderer = vtk.vtkRenderer()
renderer.AddActor(pointCloud.vtkActor)
renderer.SetBackground(.2, .3, .4)
renderer.ResetCamera()

# Render Window
renderWindow = vtk.vtkRenderWindow()
renderWindow.AddRenderer(renderer)

# Interactor
renderWindowInteractor = vtk.vtkRenderWindowInteractor()
renderWindowInteractor.SetRenderWindow(renderWindow)

# Begin Interaction
renderWindow.Render()
renderWindowInteractor.Start()
