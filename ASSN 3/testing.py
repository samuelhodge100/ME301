import numpy as np
from collections import Counter


class K_Nearest_Neighbors:
    def __init__(self):
        # k between 1 and 20
        self.k = 3
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

    def read_data(self):
        # Put data from csv file into a dataset
        dataset = []
        class_dataset = []
        for d, c in self.datafiles:
            f = open(
                "C:/Users/liban/Desktop/School stuff/CS 301/ME301/ASSN 3/" + d, "r"
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
        return dataset


# files are [concave_57D,convex_57D,straight_wall]_[11cm,16cm,22cm]_wall_data.csv
k_nearest_neighbors = K_Nearest_Neighbors()
# dataset = k_nearest_neighbors.read_data(
# "C:/Users/liban/Desktop/School stuff/CS 301/ME301/ASSN 3/concave_57D_11cm_wall_data.csv"
# )
# k_nearest_neighbors.normalize_dataset(dataset)
k_nearest_neighbors.fit_data()
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
