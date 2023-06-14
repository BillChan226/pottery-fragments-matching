import numpy as np

# this function is not working properly, filter is way too small
# eventually I only apply the scipy gaussian_filter1d instead
def gaussian_filter_smoothen(boundary_array, order):

    # gaussian filter
    smoothed_boundary_array = []
    filter_width = 6
    sum = 0
    sigma = filter_width/3  # standard deviation of the Gaussian filter

    for j in range(-filter_width, filter_width):
        sum += np.exp(-j**2/(2*sigma**2))

    print("\n")
    for i in range(0, len(order)):
        
        filter = np.exp(-i**2/(2*sigma**2))/sum
        # print("filter", filter)
        p_i = 0
        
        for j in range(-filter_width, filter_width):
            p_i += boundary_array[order[(i-j)%len(order)]]*filter
            
        smoothed_boundary_array.append(p_i)

    return smoothed_boundary_array