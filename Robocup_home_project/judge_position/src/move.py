import os
import shutil
import tensorflow.contrib.slim 
# def renew_struture():
#     i = 0
#     for path in os.listdir('/home/athomews1920/new_pics4-12'):      
#          for position in os.listdir('/home/athomews1920/new_pics4-12/'+path):
#              for fi in os.listdir('/home/athomews1920/new_pics4-12/'+path+'/'+position):
#                 shutil.move('/home/athomews1920/new_pics4-12/'+path+'/'+position+'/'+fi,'/home/athomews1920/new_pics4-12/'+path+'/'+str(i)+'.jpeg')
#                 i=i+1

def renew_struture():
    i = 1000
    for fi in os.listdir('/home/athomews1920/new_pics4-12/right_pics'):
        shutil.move('/home/athomews1920/new_pics4-12/right_pics'+'/'+fi,'/home/athomews1920/dataset_gesture/right/'+str(i)+'.jpeg')
        i+=1

if __name__=="__main__":
    # renew_struture()
    # print(tensorflow.__version__)
