import numpy as np
from collections import Counter
from matplotlib import pyplot as plt
import matplotlib.patches as mpatches
import random

def separate_lists(input_list: list[list[int]], percentage: float) -> tuple[list[list[int]], list[list[int]]]:
    """
    Separate a list of lists into two groups based on a given percentage.
    
    Parameters:
    - input_list: List of lists to be separated.
    - percentage: Float representing the percentage of the list that goes into the first group.
    
    Returns:
    Tuple of two lists of lists, where the first list contains approximately the given percentage of the original list,
    and the second list contains the rest.
    """
    
    # Shuffle the list to ensure randomness
    random.shuffle(input_list)
    
    # Calculate the split index
    split_index = int(len(input_list) * percentage)
    
    # Split the list into two based on the calculated index
    first_group = input_list[:split_index]
    second_group = input_list[split_index:]
    
    return first_group, second_group

class K_Nearest_Neighbors:
    def __init__(self):
        # k between 1 and 20
        self.k = 10
        self.classes = ["concave", "convex", "straight_wall"]
        self.datafiles = [
            ["concave_57D_11cm_wall_data.csv", "concave"],
            ["concave_57D_16cm_wall_data.csv", "concave"],
            ["concave_57D_22cm_wall_data.csv", "concave"],
            ["convex_57D_11cm_wall_data.csv", "convex"],
            ["convex_57D_16cm_wall_data.csv", "convex"],
            ["convex_57D_22cm_wall_data.csv", "convex"],
            ["straight_wall_11cm_data.csv", "straight_wall"],
            ["straight_wall_16cm_data.csv", "straight_wall"],
            ["straight_wall_22cm_data.csv", "straight_wall"],
        ]
        self.x_data = []
        self.class_data = []
        self.validation_set = []
        self.training_set = []

    def seperate_dataset(self,percentage):
        # Add the class label to the end of the list
        for i in range(len(self.class_data)):
            row = self.x_data[i]
            row.append(self.class_data[i])
            self.validation_set.append(row)

        validation_set, training_set = separate_lists(self.validation_set,percentage)
        self.validation_set = validation_set
        self.training_set = training_set
        return training_set, validation_set

    def normalize_dataset(self, dataset):
        # Normalize the dataset
        norm_dataset = []
        for row in dataset:
            norm_row = np.linalg.norm(row)
            norm_dataset.append([r / norm_row for r in row])
        return norm_dataset

    def fit_data(self):
        # sets up x_data and class_data
        dataset = self.read_data()
        self.x_data = self.normalize_dataset(dataset)

    def two_vec_dist(self, row1, row2):
        # Compute the distance between two vectors
        return np.linalg.norm(np.array(row1) - np.array(row2))

    def predict(self, predict_data):
        # Takes in an unknown array of scan data and returns a classification
        norm = np.linalg.norm(predict_data)
        norm_predict_data = [p / norm for p in predict_data]
        distances = []
        for x in range(len(self.x_data)):
            distances.append([self.two_vec_dist(norm_predict_data, self.x_data[x]), x])
        distances.sort()
        print(distances)
        predictions = []
        for x in range(self.k):
            predictions.append(self.class_data[distances[x][1]])
        print(predictions)
        return Counter(predictions).most_common(1)[0][0]
    
    def validation_predict(self,k):
        # Takes in an unknown array of scan data and returns a classification
        training_set, validation_set = self.seperate_dataset(0.4)
        
        solns = []
        convex_counter = 0
        concave_counter = 0
        straight_wall_counter = 0
        convex_true_counter = 0
        concave_true_counter = 0
        straight_wall_true_counter = 0

        for row in validation_set:
            x_row = row[:-1]
            norm = np.linalg.norm(x_row)
            norm_predict_data = [p / norm for p in x_row]
            distances = []
            for x in range(len(training_set)):
                distances.append([self.two_vec_dist(norm_predict_data, training_set[x][:-1]), x])
            distances.sort()
        
            predictions = []
            for x in range(k):
                predictions.append(training_set[distances[x][1]][-1])

            prediction = Counter(predictions).most_common(1)[0][0]
            # Check prediction against actual

            booll = prediction==row[-1]
            
            if (row[-1] == "concave"):
                concave_counter+=1
                if booll:
                    concave_true_counter+=1
            elif (row[-1] == "convex"):
                convex_counter+=1
                if booll:
                    convex_true_counter+=1
            elif (row[-1] == "straight_wall"):
                straight_wall_counter+=1
                if booll:
                    straight_wall_true_counter+=1
            solns.append([row[-1], booll])
        #for soln in solns:
            #print(soln)
        print(f'True Positive Convex: {convex_true_counter} {convex_counter}')
        print(f'True Positive Concave: {concave_true_counter} {concave_counter}')
        print(f'True Positive Straight Wall: {straight_wall_true_counter} {straight_wall_counter}')
        return solns

    def read_data(self):
        # Put data from csv file into a dataset
        dataset = []
        class_dataset = []
        for d, c in self.datafiles:
            f = open(
                "./" + d, "r"
            )
            row = f.readline()
            while row:  # read until end of file
                row_data = row.split(",")
                # last element might be an empty string
                row_data.pop()
                row_data = list(map(int, row_data))
                dataset.append(row_data)
                class_dataset.append(c)
                row = f.readline()
            f.close()
        self.class_data = class_dataset
        print(dataset[1])
        return dataset
    
    def plot_data(self):
        fig, ax = plt.subplots(1)
        for i in range(len(self.x_data)):
            if (self.class_data[i] == 'concave'):
                ax.scatter(range(len(self.x_data[i])),self.x_data[i],5,color='r',marker='o',label='concave')
            elif(self.class_data[i] =='convex'):
                ax.scatter(range(len(self.x_data[i])),self.x_data[i],5,color='b',marker='^',label='covnvex')
            elif(self.class_data[i] =='straight_wall'):
                ax.scatter(range(len(self.x_data[i])),self.x_data[i],5,color='g',marker='*',label='straight_wall')
        concave_series = mpatches.Patch(color='red', label='concave')
        convex_series = mpatches.Patch(color='blue', label='convex')
        straight_wall_series = mpatches.Patch(color='green', label='straight_wall')
        ax.set_title('Normalized Dataset')
        ax.set_xlabel('Index')
        ax.set_ylabel('Normalized DMS Sensor Reading')
        ax.legend(handles=[concave_series,convex_series,straight_wall_series])
        plt.show()


# files are [concave_57D,convex_57D,straight_wall]_[11cm,16cm,22cm]_wall_data.csv
k_nearest_neighbors = K_Nearest_Neighbors()
# dataset = k_nearest_neighbors.read_data(
# "C:/Users/liban/Desktop/School stuff/CS 301/ME301/ASSN 3/concave_57D_11cm_wall_data.csv"
# )
# k_nearest_neighbors.normalize_dataset(dataset)
k_nearest_neighbors.fit_data()
solns = k_nearest_neighbors.validation_predict(20)


"""
print(
    k_nearest_neighbors.predict(
        [
            600,
            492,
            570,
            637,
            705,
            954,
            812,
            844,
            901,
            926,
            950,
            981,
            993,
            1021,
            1023,
            1023,
            1050,
            1023,
            1023,
            1017,
            998,
            964,
            951,
            911,
            883,
            846,
            796,
            740,
            882,
            675,
        ]
    )
)
"""

#k_nearest_neighbors.plot_data()