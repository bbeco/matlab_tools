import bpy
import mathutils

def retrieveCameraList():
	cameraList = [];
	for obj in bpy.data.objects:
		if obj.type == 'CAMERA':
			cameraList.append(CameraPose(obj))
	return cameraList

def printAll():
	cameraList = retrieveCameraList()
	print('% Locations')
	for c in cameraList:
		print('% ' + c.name)
		c.printLocation()
	print('% Orientations')
	for c in cameraList:
		print('% ' + c.name)
		c.printOrientation()

class CameraPose:
	def _rotationMatrixToStr(mat):
		s = '['
		for i in range(0, 3):
			for j in range(0, 3):
				s = s + ' ' + str(mat[i][j])
			if i != 2:
				s = s + ';'
		s = s + ' ]'
		return s

	def _locationToStr(loc):
		s = '['
		for i in range(0, 3):
			s = s + ' ' + str(loc[i])
		s = s + ']'
		return s

	def __init__(self, cameraObj):
		self.name = cameraObj.name
		self.location = cameraObj.location
		self.rotation = cameraObj.rotation_euler
	
	def __str__(self):
		print('% ' + name)
		print('% Location')
		print(locationToStr(location))
		print('% Orientation')
		print(rotationMatrixToStr(rotation))
	
	def printOrientation(self):
		axis =  ['X', 'Y', 'Z']
		rot = mathutils.Matrix().to_3x3()
		rot.identity()
		mat1 = mathutils.Matrix.Rotation(self.rotation[0], 4, 'X').to_3x3()
		mat2 = mathutils.Matrix.Rotation(self.rotation[1], 4, 'Z').to_3x3()
		mat3 = mathutils.Matrix.Rotation(-self.rotation[2], 4, 'Y').to_3x3()
		rot = mat3 * mat2 * mat1
		# rot is a rotation matrix in the premultiply form. Matlab ViewSet uses 
		# post multiplication form for the orientation matrixes, so rot has to 
		# be transposed.
		rot.transpose()
		print(CameraPose._rotationMatrixToStr(rot))
		
	def printLocation(self):
		print(CameraPose._locationToStr(self.location))
		
if __name__ == "__main__":
	printAll()
