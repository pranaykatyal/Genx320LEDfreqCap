import numpy as np

src = [(42,22), (50,25), (54, 25), (61 , 39), (67,42), (74,44), (82,58), (88, 62),(94,63)]
dst = [(123, 115), (133, 119), (143, 122), (153, 146), (163, 150), (173, 153), (184, 174), (193, 178), (202, 182)]
src.sort(key=lambda x:x[0])
dst.sort(key=lambda x:x[0])


def construct_matrix(src_list, dst_list):
    """
    Construct a matrix for calculating the perspective transformation.
    src_list and dst_list are lists of tuples in (col, row) (x, y) format.
    """
    # Initialize an empty matrix
    A = np.zeros((0, 10))

# Iterate through the source and destination points
    for (xs, ys), (xd, yd) in zip(src_list, dst_list):
        # Construct the rows for the matrix
        first_row = [-xs, -ys, -1, 0, 0, 0, 0,0,0, xd]
        second_row = [0, 0, 0, -xs, -ys, -1, 0, 0, 0, yd]
        third_row = [0, 0, 0, 0, 0, 0, -xs, -ys, -1, 1]
# Append the rows to the matrix
    A = np.vstack([A, first_row, second_row,third_row])

# Compute A_T * A
    ATA = np.dot(A.T, A)

# Compute eigenvalues and eigenvectors
    eigenvalues, eigenvectors = np.linalg.eigh(ATA)

#Find the eigenvector corresponding to the smallest eigenvalue
    min_eigenvalue_index = np.argmin(eigenvalues)
    #last=min_eigenvalue_index[-1]
    h_uncropped = eigenvectors[:, min_eigenvalue_index]
    h_uncropped/=h_uncropped[-1]  # Normalize by the last element
    print(h_uncropped)
    h=h_uncropped[:-1]  # Exclude the last element
    
    H = h.reshape((3, 3))
    return  H

H = construct_matrix(src, dst)
print("Homography Matrix:", H)

### dst = H*src
### 
for pixel in src:
    x, y = pixel
    transformed_pixel = np.dot(H, np.array([x, y, 1]))
    transformed_pixel /= transformed_pixel[2]  # Normalize by the third coordinate
    print(f"Original: {pixel}, Transformed: ({transformed_pixel[0]}, {transformed_pixel[1]})")


