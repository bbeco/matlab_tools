import bpy

from camera_pose import CameraPose

baseOutputFolder = '/home/andrea/matlab_tools/images/sfm_test/test7/'
firstFrameNumber = 2
lastFrameNumber = 25
cameraName = 'Camera.001'

def listCameraPoses():
	print('% Location\nLocation = {')
	for i in range(1, lastFrameNumber + 1):
		bpy.context.scene.frame_set(i)
		c = CameraPose(bpy.data.objects[cameraName])
		print('% Frame ', str(i))
		c.printLocation()
	print('};\n% Orientation\nOrientation = {')
	for i in range(1, lastFrameNumber + 1):
		bpy.context.scene.frame_set(i)
		c = CameraPose(bpy.data.objects[cameraName])
		print('% Frame ', str(i))
		c.printOrientation()
	print('};')

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

