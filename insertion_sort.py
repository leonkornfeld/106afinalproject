class Entry():
    def __init__(self, start, end):
        self.start = start
        self.end = end
    def __repr__(self):
        return f"Entry(start={self.start}, end={self.end})"

def insertion_sort(arr):
    temp = 6
    output_list = []
    for i in range(1, len(arr)):
        key = arr[i]  # element to be placed correctly
        # print(key)
        j = i - 1
        output_list.append(Entry(j + 1, temp))
        while j >= 0 and arr[j] > key:
            output_list.append(Entry(j,j+1))
            arr[j + 1] = arr[j]
            j -= 1
       
        arr[j + 1] = key  # Place key after the last moved element
        output_list.append(Entry(temp, j+1))
        # print('new', j+1)
    return output_list
print(insertion_sort([5,4,3,2,1]))