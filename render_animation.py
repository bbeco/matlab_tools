import bpy

from camera_pose import CameraPose

baseOutputFolder = '/home/andrea/thesys/matlab_tools/images/sfm_test/test8_10bigger/'
firstFrameNumber = 1
lastFrameNumber = 19
cameraName = 'Camera.001'
outputFile = '/tmp/poses.txt'

def listCameraPoses():
	file = open(outputFile, 'w')
	s = ''
	s = s + '% Location\nLocation = {\n'
	for i in range(firstFrameNumber, lastFrameNumber + 1):
		bpy.context.scene.frame_set(i)
		c = CameraPose(bpy.data.objects[cameraName])
		s = s + '% Frame ' + str(i) + '\n'
		s = s + c.getLocation() + '\n'
	s = s + '};\n% Orientation\nOrientation = {\n'
	for i in range(firstFrameNumber, lastFrameNumber + 1):
		bpy.context.scene.frame_set(i)
		c = CameraPose(bpy.data.objects[cameraName])
		s = s + '% Frame ' +  str(i) + '\n'
		s = s + c.getOrientation() + '\n'
	s = s + '};\n'
	
	file.write(s)
	file.close()

def renderAnimation():
	cam = bpy.data.objects[cameraName]
	for i in range(firstFrameNumber, lastFrameNumber + 1):
		#setting the correct frame number
		bpy.context.scene.frame_set(i)
		print('Rendering frame ', i)
		filename = 'll' + str(i) + '.png'
		#Set filepath to store render result
		bpy.data.scenes['Scene'].render.filepath = baseOutputFolder + filename
		bpy.ops.render.render(write_still=True)

