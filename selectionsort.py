
class Entry():
    def __init__(self, start, end):
        self.start = start
        self.end = end
    def __repr__(self):
        return f"Entry(start={self.start}, end={self.end})"

def selection_sort(arr):
    temp = len(arr)
    output_list = []
    # Traverse through all elements in the array
    for i in range(len(arr)):
        # Find the minimum element in the unsorted portion
        min_index = i
        for j in range(i + 1, len(arr)):
            if arr[j] < arr[min_index]:
                min_index = j
        if i != min_index:
            output_list.append(Entry(min_index, temp))
            output_list.append(Entry(i, min_index))
            output_list.append(Entry(temp, i))
        arr[i], arr[min_index] = arr[min_index], arr[i]

    return output_list

arr = [5,4,3,2,1]
l = selection_sort(arr)
print(l)