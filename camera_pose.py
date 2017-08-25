import bpy
import mathutils
import copy

def retrieveCameraList():
	cameraList = [];
	for obj in bpy.data.objects:
		if obj.type == 'CAMERA':
			cameraList.append(CameraPose(obj))
	return cameraList

def printAll():
	cameraList = retrieveCameraList()
	print('% Location\nLocation = {')
	for c in cameraList:
		print('% ' + c.name)
		print(c.getLocation())
	print('};\n% Orientation\nOrientation = {')
	for c in cameraList:
		print('% ' + c.name)
		print(c.getOrientation())
	print('};')

class CameraPose:
	def _rotationMatrixToStr(mat):
		s = '['
		for i in range(0, 3):
			for j in range(0, 3):
				s = s + ' ' + str(mat[i][j])
			if i != 2:
				s = s + ';\n'
		s = s + ' ]'
		return s

	def _locationToStr(loc):
		s = '[ ' + str(loc[0]) + ' ' + str(-loc[2]) + ' ' + str(loc[1]) + ' ]'
		return s

	def __init__(self, cameraObj):
		self.name = cameraObj.name
		self.location = copy.copy(cameraObj.location)
		self.rotation = copy.copy(cameraObj.rotation_euler)
	
	def __str__(self):
		s = '% ' + self.name + '\n'
		s = s + '% Location\n'
		s = s + self.getLocation() + '\n'
		s = s + '% Orientation\n'
		s = s + self.getOrientation() + '\n'
		return s
	
	def getOrientation(self):
		rot = mathutils.Matrix().to_3x3()
		rot.identity()
		mat1 = mathutils.Matrix.Rotation(self.rotation[0], 4, 'X').to_3x3()
		mat2 = mathutils.Matrix.Rotation(self.rotation[1], 4, 'Y').to_3x3()
		mat3 = mathutils.Matrix.Rotation(-self.rotation[2], 4, 'Z').to_3x3()
		rot = mat3 * mat2 * mat1
		# rot is a rotation matrix in the premultiply form. Matlab ViewSet uses 
		# post multiplication form for the orientation matrixes, so rot has to 
		# be transposed.
		rot.transpose()
		return CameraPose._rotationMatrixToStr(rot)
		
	def getLocation(self):
		return CameraPose._locationToStr(self.location)

