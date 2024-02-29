import numpy as np

class K_Nearest_Neighbors():
    def __init__(self,dataset,k):
        self.k = k
        self.dataset = dataset
    
    def normalize_dataset(self,dataset):
        # Normalize the dataset
        for row in dataset:
            norm_row = np.linalg.norm()
        pass

    def two_vec_dist(self, row1, row2):
        # Compute the distance between two vectors

        pass
    

#TODO: Need a method to get data from from the csv file into a dataset
f = open("file_name",'r')
