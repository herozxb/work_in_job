import matplotlib.pyplot as plt
import numpy as np
import pycuda.autoinit
import pycuda.driver as cuda
from pycuda.compiler import SourceModule
import time

"""
Create Your Own Lattice Boltzmann Simulation (With Python)
Philip Mocz (2020) Princeton Univeristy, @PMocz

Simulate flow past cylinder
for an isothermal fluid

"""


# Define CUDA kernel
mod = SourceModule("""
__global__ void shift_array(float *F, int Ny, int Nx, int NL, int *idxs, int *cxs, int *cys)
{
    int tid_x = threadIdx.x + blockIdx.x * blockDim.x;
    int tid_y = threadIdx.y + blockIdx.y * blockDim.y;
    int idx = tid_y * Nx + tid_x;

    if (tid_x < Nx && tid_y < Ny)
    {
        for (int i = 0; i < NL; ++i)
        {
            int new_x = tid_x + cxs[i];
            int new_y = tid_y + cys[i];
            
            if (new_x >= 0 && new_x < Nx && new_y >= 0 && new_y < Ny)
            {
                int new_idx = new_y * Nx + new_x;
                F[new_idx * NL + i] = F[idx * NL + i];
            }
        }
    }
}
""")


# Define CUDA kernel
mod_Feq = SourceModule("""
__global__ void calculate_Feq(float *Feq, float *rho, float *ux, float *uy, int Ny, int Nx, int NL, int *idxs, int *cxs, int *cys, float *weights)
{
    int tid_x = threadIdx.x + blockIdx.x * blockDim.x;
    int tid_y = threadIdx.y + blockIdx.y * blockDim.y;
    int idx = tid_y * Nx + tid_x;

    if (tid_x < Nx && tid_y < Ny)
    {
        for (int i = 0; i < NL; ++i)
        {
            float cx = (float)cxs[i];
            float cy = (float)cys[i];
            float w = weights[i];
            float rho_val = rho[idx];
            float ux_val = ux[idx];
            float uy_val = uy[idx];

            Feq[(idx * NL) + i] = rho_val * w * (1 + 3 * (cx * ux_val + cy * uy_val) + 9 * pow(cx * ux_val + cy * uy_val, 2) / 2 - 3 * (pow(ux_val, 2) + pow(uy_val, 2)) / 2);
        }
    }
}
""")



# Create GPU arrays for input data





def main():
	""" Lattice Boltzmann Simulation """
	
	# Simulation parameters
	Nx                     = 300    # resolution x-dir
	Ny                     = 100    # resolution y-dir
	rho0                   = 100    # average density
	tau                    = 0.6    # collision timescale
	Nt                     = 400000   # number of timesteps
	plotRealTime = True # switch on for plotting as the simulation goes along
	
	# Lattice speeds / weights
	NL = 9
	idxs = np.arange(NL).astype(np.int32)
	cxs = np.array([0, 0, 1, 1, 1, 0, -1, -1, -1]).astype(np.int32)
	cys = np.array([0, 1, 1, 0, -1, -1, -1, 0, 1]).astype(np.int32)
	weights = np.array([4/9,1/9,1/36,1/9,1/36,1/9,1/36,1/9,1/36]).astype(np.float32) # sums to 1
	
	# Initial Conditions
	F = np.ones((Ny, Nx, NL)).astype(np.float32)
	np.random.seed(42)
	F += 0.01*np.random.randn(Ny,Nx,NL).astype(np.float32)
	X, Y = np.meshgrid(range(Nx), range(Ny))
	#F[:,:,3] += 2 * (1+0.2*np.cos(2*np.pi*X/Nx*4))
	F[:,:,1] += 2 * (1+0.2*np.cos(2*np.pi*Y/Nx*4)).astype(np.float32)
	rho = np.sum(F,2).astype(np.float32)
	for i in idxs:
		F[:,:,i] *= rho0 / rho
	
	# Cylinder boundary
	X, Y = np.meshgrid(range(Nx), range(Ny))
	
	cylinder = ((X - Nx/4)**2 + (Y - Ny/2)**2 < (Ny/4)**2) 
	
	for x in range(Nx):
		for y in range(Ny):
			if y==0 or y==Ny-1 or x==0 or  x==Nx-1 :
				cylinder[y][x] =True
		
	
	# Prep figure
	fig = plt.figure(figsize=(4,2), dpi=80)
	
	
	Feq = np.zeros((Ny, Nx, NL), dtype=np.float32)
	

	
	
	# Simulation Main Loop
	for it in range(Nt):
		print(it)
		
	
	
				

	
	
		# ---------------------------------------[CORE of CDF][start]--------------------------------------------- #
		# ---------------------------------------[0][Drift]------------------------------------------------------- #
		
		#-----------------------[ NO CUDA Elapsed time: 0.0023720264434814453 seconds]---------------------------- #
		#for i, cx, cy in zip(idxs, cxs, cys):
		#	F[:,:,i] = np.roll(F[:,:,i], cx, axis=1)
		#	F[:,:,i] = np.roll(F[:,:,i], cy, axis=0)
		
		#-----------------------[CUDA Elapsed time: 0.0006928443908691406 seconds]-------------------------------- #
		d_F = cuda.mem_alloc(F.nbytes)
		d_idxs = cuda.mem_alloc(idxs.nbytes)
		d_cxs = cuda.mem_alloc(cxs.nbytes)
		d_cys = cuda.mem_alloc(cys.nbytes)

		cuda.memcpy_htod(d_F, F)
		cuda.memcpy_htod(d_idxs, idxs)
		cuda.memcpy_htod(d_cxs, cxs)
		cuda.memcpy_htod(d_cys, cys)

		# Launch CUDA kernel
		func = mod.get_function("shift_array")
		block_size = (16, 16, 1)
		grid_size = ((Nx + block_size[0] - 1) // block_size[0], (Ny + block_size[1] - 1) // block_size[1])
		
		
		
		func(d_F, np.int32(Ny), np.int32(Nx), np.int32(NL), d_idxs, d_cxs, d_cys, block=block_size, grid=grid_size)
		
		# Copy result back to host
		cuda.memcpy_dtoh(F, d_F)
		


		
		# Set reflective boundaries
		bndryF = F[cylinder,:]
		bndryF = bndryF[:,[0,5,6,7,8,1,2,3,4]]
	
		# Start time
		start_time = time.time()
		
		
		#-----------------------[CUDA Elapsed time: 0.004290103912353516 seconds]-------------------------------- #
		# Calculate fluid variables
		# F is the mass, or the PDF, PMF
		# rho is the every cell mass in PMF, or cell density in PDF
		rho = np.sum(F,2).astype(np.float32)
		
		# get every cell of F, with the velocity in the x direction, or the momentum in the x direction when with weights
		ux  = np.sum(F*cxs,2).astype(np.float32) / rho
		# get every cell of F, with the velocity in the y direction, or the momentum in the y direction when with weights
		uy  = np.sum(F*cys,2).astype(np.float32) / rho
		
		''' SLOWER
		d_Feq = cuda.mem_alloc(Feq.nbytes)
		d_rho = cuda.mem_alloc(rho.nbytes)
		d_ux = cuda.mem_alloc(ux.nbytes)
		d_uy = cuda.mem_alloc(uy.nbytes)
		d_weights = cuda.mem_alloc(weights.nbytes)

		cuda.memcpy_htod(d_Feq, Feq)
		cuda.memcpy_htod(d_rho, rho)
		cuda.memcpy_htod(d_ux, ux)
		cuda.memcpy_htod(d_uy, uy)
		cuda.memcpy_htod(d_weights, weights)
		
		# Launch CUDA kernel
		func = mod_Feq.get_function("calculate_Feq")
		block_size = (16, 16, 1)
		grid_size = ((Nx + block_size[0] - 1) // block_size[0], (Ny + block_size[1] - 1) // block_size[1])
		func(d_Feq, d_rho, d_ux, d_uy, np.int32(Ny), np.int32(Nx), np.int32(NL), d_idxs, d_cxs, d_cys, d_weights, block=block_size, grid=grid_size)

		# Copy result back to host
		cuda.memcpy_dtoh(Feq, d_Feq)
		
		# End time
		end_time = time.time()
		elapsed_time = end_time - start_time
		print("Elapsed time:", elapsed_time, "seconds")
		'''
		# ---------------------------------------[0][Collision]---------------------------------------------------_ #
		# Apply Collision
		
		#-----------------------[CUDA Elapsed time: 0.003415822982788086 seconds]-------------------------------- #
		for i, cx, cy, w in zip(idxs, cxs, cys, weights):
			Feq[:,:,i] = rho * w * ( 1 + 3*(cx*ux+cy*uy)  + 9*(cx*ux+cy*uy)**2/2 - 3*(ux**2+uy**2)/2 )
		
		
		F += -(1.0/tau) * (F - Feq)
		# ---------------------------------------[CORE of CDF][end]--------------------------------------------- #
		
		# Apply boundary 
		F[cylinder,:] = bndryF
		
		
		# plot in real time - color 1/2 particles blue, other half red
		if (plotRealTime and (it % 10) == 0) or (it == Nt-1):
			plt.cla()
			ux[cylinder] = 0
			uy[cylinder] = 0
			vorticity = (np.roll(ux, -1, axis=0) - np.roll(ux, 1, axis=0)) - (np.roll(uy, -1, axis=1) - np.roll(uy, 1, axis=1))
			vorticity[cylinder] = np.nan
			vorticity = np.ma.array(vorticity, mask=cylinder)
			
			# show the MASS
			#plt.imshow(rho)
			
			# show the Collision
			#collision = np.sum(Feq,2)
			#plt.imshow(collision)
			
			# show the Velocity
			plt.imshow(np.sqrt(ux**2+uy**2))
			
			# show the vorticity
			#plt.imshow(vorticity, cmap='bwr')
			
			plt.imshow(~cylinder, cmap='gray', alpha=0.3)
			plt.clim(-.1, .1)
			ax = plt.gca()
			ax.invert_yaxis()
			ax.get_xaxis().set_visible(False)
			ax.get_yaxis().set_visible(False)	
			ax.set_aspect('equal')	
			plt.pause(0.001)
			
	
	# Save figure
	plt.savefig('latticeboltzmann.png',dpi=240)
	plt.show()
	    
	return 0



if __name__== "__main__":
  main()

