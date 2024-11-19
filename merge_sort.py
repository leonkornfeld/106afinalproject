import math
class Entry():
    def __init__(self, start, end, level):
        self.start = start
        self.end = end
        self.level = level
    def __repr__(self):
        return f"Entry(start={self.start}, end={self.end}, level={self.level})"

def merge_sort(arr, movements):
    """
    Perform merge sort on a list.
    
    :param arr: List of elements to be sorted
    :return: Sorted list
    """
    if len(arr) <= 1:
        print(arr)
        # movements.append(Entry(arr[0][1],arr[0][1]))
        return arr

    # Split the array into two halves
    mid = len(arr) // 2
    left_half = arr[:mid]
    right_half = arr[mid:]

    # Recursively sort both halves
    left_sorted = merge_sort(left_half, movements)
    right_sorted = merge_sort(right_half, movements)

    # Merge the sorted halves
    return merge(left_sorted, right_sorted, movements)

def merge(left, right, movements):
    """
    Merge two sorted lists into a single sorted list.
    
    :param left: First sorted list
    :param right: Second sorted list
    :return: Merged sorted list
    """
    merged = []
    left_index = right_index = 0

    # Merge while there are elements in both lists
    next = min(left[0][1], right[0][1])
    while left_index < len(left) and right_index < len(right):
        if left[left_index][0] < right[right_index][0]:
            # if next != left[left_index][1]:
            movements.append(Entry(left[left_index][1],next, math.ceil(math.log2((len(right))))))
            merged.append((left[left_index][0], next))
            left_index += 1

        else:
            # if next != right[right_index][1]:
            movements.append(Entry(right[right_index][1],next, math.ceil(math.log2((len(right))))))
            merged.append((right[right_index][0], next))
            right_index += 1
        next+=1
    

    # Add remaining elements from left and right
    if left_index == len(left):
        while right_index < len(right):
            # if next != right[right_index][1]:
            movements.append(Entry(right[right_index][1],next, math.ceil(math.log2((len(right))))))
            merged.append((right[right_index][0], next))
            right_index+=1
            next+=1
    elif right_index == len(right):
        while left_index < len(left):
            merged.append((left[left_index][0], next))
            # if next != left[left_index][1]:
            movements.append(Entry(left[left_index][1], next, math.ceil(math.log2((len(right))))))
            left_index+=1
            next+=1    
    # movements.append(Entry("split","split"))
    print(merged)
    return merged

# Example usage
if __name__ == "__main__":
    movements =[]
    array = [78, 39, 27,144,3,9,1,0]
    array2 = [(array[i], i) for i in range(len(array))]
    print("Original array:", array)
    sorted_array = merge_sort(array2, movements)
    print("Sorted array:", sorted_array)
    # print(movements)
    sorted_movements = sorted(movements, key=lambda x: x.level)
    temp = set([i for i in range(len(array))])
    for movement in sorted_movements:
        if movement.level != 0:
            break
        if movement.start in temp:
            temp.remove(movement.start)
    for num in temp:
        sorted_movements.append(Entry(num,num, 0))    
    sorted_movements = sorted(sorted_movements, key=lambda x: (x.level, x.end))

    print(sorted_movements)