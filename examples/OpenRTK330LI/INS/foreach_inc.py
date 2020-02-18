import os
import time
import shutil


def mkdir(path):
	path = path.strip()
	path = path.rstrip("\\")
	is_exit = os.path.exists(path)
	if not is_exit:
		os.makedirs(path) 
		print (path+' mkdir suc')
		return True
	else:
		print ('exist')
		return False

def foreach(root_dir):
	print ("start foreach head file...")
	start_time = time.time()
	mkdir('lib_inc')
	mkdir('lib_inc/Legacy')
	mkdir('lib_inc/lwip')
	mkdir('lib_inc/arch')
	for root,dirs,files in os.walk(root_dir):
		if (('Applications' in root) == False):			
			for filename in files:
				if filename[-2:] == '.h':
					filepath = os.path.join(root, filename)
					if ('CPU\STM32F469\HAL\Inc\Legacy' in root):
						shutil.copy(filepath,'lib_inc/Legacy')
					if ('/LWIP\lwip-1.4.1\src' in root):
						shutil.copy(filepath,'lib_inc/lwip')
					elif ('arch' in root):
						shutil.copy(filepath,'lib_inc/arch')
						print ('find')
					else:
						shutil.copy(filepath,'lib_inc')
		
	end_time = time.time()
	print ("time %.2fs" % (end_time - start_time))
dir = '../../../../'
foreach(dir)