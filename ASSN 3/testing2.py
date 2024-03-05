from typing import List, Tuple
import random

def separate_lists(input_list: List[List[int]], percentage: float) -> Tuple[List[List[int]], List[List[int]]]:
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

# Example usage
input_list = [[1, 2, 3, 4], [5, 6], [7, 8], [9, 10]]

x_row = input_list[0][:-1]
print(x_row)