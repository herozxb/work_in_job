import numpy as np

# Create a simple 1D array
arr = np.array([1, 2, 3, 4, 5])

# Roll the array to the right by 2 positions
rolled_arr = np.roll(arr, 2)

print("Original array:", arr)
print("Rolled array:", rolled_arr)



#[ 8 1 2 ]
#[ 7 0 3 ]
#[ 6 5 4 ]

cxs = np.array([0, 0, 1, 1, 1, 0,-1,-1,-1])
cys = np.array([0, 1, 1, 0,-1,-1,-1, 0, 1])


Nx                     = 3    # resolution x-dir
Ny                     = 3    # resolution y-dir

NL = 9
idxs = np.arange(NL)

# Initial Conditions
#F = np.ones((Ny,Nx,NL))

#F += np.random.randn(Ny,Nx,NL)


# Calculate the total number of elements
total_elements = Ny * Nx * NL

# Generate an array containing integers from 1 to total_elements
F = np.arange(1, total_elements + 1)

# Reshape the array to the desired shape
F = F.reshape((Ny, Nx, NL))


print("================F[0]==================")
print(F)

# Drift
#for i, cx, cy in zip(idxs, cxs, cys):
#	F[:,:,i] = np.roll(F[:,:,i], cx, axis=1)
#	F[:,:,i] = np.roll(F[:,:,i], cy, axis=0)
	
i=7
cx = cxs[i]
cy = cys[i]
F[:,:,i] = np.roll(F[:,:,i], cx, axis=1)
F[:,:,i] = np.roll(F[:,:,i], cy, axis=0)
	
print("================F[1]==================")
print(F)

print("================F[2]==================")
print(F[0,0,:])

# sum of 9 direction
# F is the mass, or the PDF, PMF
rho = np.sum(F,2)

print("================F[3]==================")
print(rho)


# Assuming F is your array
#F = np.array([[[1, 2, 3], [4, 5, 6]], [[7, 8, 9], [10, 11, 12]]])

# Sum the elements of F along the third axis
#rho = np.sum(F, axis=2)

print("Resulting array rho:")
print(rho)

# Generate an array containing integers from 1 to total_elements
F = np.arange(1, total_elements + 1)

# Reshape the array to the desired shape
F = F.reshape((Ny, Nx, NL))


print(F*cxs)

# get every cell of F, with the velocity in the x direction
print(np.sum(F*cxs,2))
# get every cell of F, with the velocity in the y direction
print(np.sum(F*cys,2))
