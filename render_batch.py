import bpy

baseOutputFolder = '/home/andrea/matlab_tools/images/sfm_test/test6/'
cameraNames = ['Camera', 'Camera.001', 'Camera.002', 'Camera.003', 'Camera.004']
def batch_render():
	for i in range(0, len(cameraNames)):
		print('Rendering from ', cameraNames[i])
		#Set the active camera
		cam = bpy.data.objects[cameraNames[i]]
		bpy.context.scene.camera = cam
		filename = 'll' + str(i + 1) + '.png'
		#Set filepath to store render result
		bpy.data.scenes['Scene'].render.filepath = baseOutputFolder + filename
		bpy.ops.render.render(write_still=True)

