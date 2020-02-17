# -*- coding: utf-8 -*-
"""
Created on Wed Feb 12 14:30:53 2020

@author: T530
"""

import fnmatch
import os
import shutil
class_=input("Class: ")
instance_=input("Instance: ")
identifiers = []

for i in range(0, 4): 
    ele = input("Identifier "+str(i+1)+": ") 
    identifiers.append(ele) # adding the element 
for attribute_ in ['color', 'depth', 'pcd']:
    for file in os.listdir('test/bags/'+class_+'/'+instance_+'/'+attribute_):
        if not os.path.exists('data/tmp/'+class_+'/'+instance_+'/'+attribute_):
            os.makedirs('data/tmp/'+class_+'/'+instance_+'/'+attribute_)
        for ide in identifiers:
            if fnmatch.fnmatch(file, '*'+ide+'*'):
                shutil.copy('test/bags/'+class_+'/'+instance_+'/'+attribute_+'/'+file, 'data/tmp/'+class_+'/'+instance_+'/'+attribute_)
for file in os.listdir('images/'+class_+'/'+instance_+'/'):
    if not os.path.exists('data/tmp/'+class_+'/'+instance_+'/image_raw'):
        os.makedirs('data/tmp/'+class_+'/'+instance_+'/image_raw')
    shutil.copy('images/'+class_+'/'+instance_+'/'+file, 'data/tmp/'+class_+'/'+instance_+'/image_raw')
