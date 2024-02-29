import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

# Simulation parameters
rho0                   = 100    # average density
tau                    = 0.6    # collision timescale
Nt                     = 10000   # number of timesteps

# Define dimensions for the lattice
Nx = 15  # Number of lattice nodes in x-direction
Ny = 15  # Number of lattice nodes in y-direction
Nz = 15  # Number of lattice nodes in z-direction
NL = 19  # Number of lattice velocities


# D3Q19
velocity_directions = np.array([
	[ 0,  0,  0],   # stationary
	[ 1,  0,  0],   # right
	[-1,  0,  0],   # left
	[ 0,  1,  0],   # up
	[ 0, -1,  0],   # down
	[ 0,  0,  1],   # front
	[ 0,  0, -1],   # back
	
	[ 1,  1,  0],   # right-up
	[ 1, -1,  0],   # right-down
	[-1,  1,  0],   # left-up
	[-1, -1,  0],   # left-down
	
	[ 1,  0,  1],   # right-front
	[ 1,  0, -1],   # right-back
	[-1,  0,  1],   # left-front
	[-1,  0, -1],   # left-back

	[ 0,  1,  1],   # up-front
	[ 0,  1, -1],   # up-back
	[ 0, -1,  1],   # down-front
	[ 0, -1, -1]    # down-back
		
])  # Direction vectors for each direction

# Define weights for D3Q19
weights = np.array([3/9, 1/18, 1/18, 1/18, 1/18, 1/18, 1/18, 1/36, 1/36, 1/36, 1/36, 1/36, 1/36, 1/36, 1/36, 1/36, 1/36, 1/36, 1/36])



# Extract cxs, cys, and czs from velocity_directions
cxs = velocity_directions[:, 0]
cys = velocity_directions[:, 1]
czs = velocity_directions[:, 2]


# Initialize the 3D array (volume) with ones
F = np.ones((Ny, Nx, Nz, NL))  # Initial conditions with ones

# Add noise to initial conditions
np.random.seed(42)
F += 0.01 * np.random.randn(Ny, Nx, Nz, NL)

# Meshgrid for x, y, and z coordinates
X, Y, Z = np.meshgrid(range(Nx), range(Ny), range(Nz), indexing='ij')

# Add perturbation to specific velocity component (in this case, index 3)
F[:, :, :, 3] += 2 * (1 + 0.2 * np.cos(2 * np.pi * X / Nx * 4))
#F[:, :, :, 5] += 2 * (1 + 0.2 * np.cos(2 * np.pi * Y / Nx * 4))
#F[:, :, :, 1] += 2 * (1 + 0.2 * np.cos(2 * np.pi * Z / Nx * 4))


# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


def stream(F):
    F_streamed = np.zeros_like(F)
    for i, direction in enumerate(velocity_directions):
        shift = tuple(np.roll(direction, -1))  # Calculate the shift for rolling
        F_streamed[:, :, :, i] = np.roll(F[:, :, :, i], shift, axis=(0, 1, 2))
    return F_streamed


# Create meshgrid for plotting
x, y, z = np.meshgrid(range(Nx), range(Ny), range(Nz), indexing='ij')


for it in range(Nt):
	#print(it)

	
	# Start time
	start_time = time.time()

	for i, cx, cy, cz in zip(range(NL), cxs, cys, czs):
		F[:,:,:,i] = np.roll(F[:,:,:,i], cx, axis=1)
		F[:,:,:,i] = np.roll(F[:,:,:,i], cy, axis=0)
		F[:,:,:,i] = np.roll(F[:,:,:,i], cz, axis=2)

	#F = stream(F)

		
	# Calculate density
	rho = np.sum(F, axis=3)

	ux  = np.sum(F*cxs,3) / rho
	uy  = np.sum(F*cys,3) / rho
	uz  = np.sum(F*czs,3) / rho

	Feq = np.zeros(F.shape)	
	# Calculate equilibrium distribution Feq
	for i, cx, cy, cz, w in zip(range(NL), cxs, cys, czs, weights):
		Feq[:, :, :, i] = rho * w * (1 + 3 * (cx*ux + cy*uy + cz*uz) + 9 * ((cx*ux + cy*uy + cz*uz)**2) / 2 - 3 * (ux**2 + uy**2 + uz**2) / 2)

	# Update distribution functions F based on relaxation process
	F += -(1.0 / tau) * (F - Feq)

	plt.cla()
	#ax.scatter(x, y, z, c=rho.flatten(), cmap='viridis')
	ax.scatter(x, y, z, c=rho.flatten(), cmap='viridis', s=10, rasterized=True ) # Elapsed time: 0.02270030975341797 seconds
	
	# End time
	end_time = time.time()
	elapsed_time = end_time - start_time
	print("Elapsed time:", elapsed_time, "seconds")
	
	#v = np.sqrt(ux**2+uy**2+uz**2)
	#ax.scatter(x, y, z, c=v.flatten(), cmap='viridis')

	#collision = np.sum(Feq,3)
	#ax.scatter(x, y, z, c=collision.flatten(), cmap='viridis')	
	
	# Plot velocity vectors

	# Set labels and title
	#ax.set_xlabel('X')
	#ax.set_ylabel('Y')
	#ax.set_zlabel('Z')
	#ax.set_title('Initial Velocity in 3D Volume')
	
	plt.pause(0.0001)

# Show plot
plt.show()
