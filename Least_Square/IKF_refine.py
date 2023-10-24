#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt


def ekf_measurement(x, y, a, b):
  return np.sqrt((x - a)**2 + (y - b)**2)

def ekf_measurement_linearization_10x2(x_hat, y_hat, a, b):
  """Linearizes the EKF measurement function around the current state estimate.

  Args:
    x_hat: The current state estimate.
    y_hat: The current state estimate.
    a: The x-coordinate of the anchor point.
    b: The y-coordinate of the anchor point.

  Returns:
    A 10x2 Jacobian matrix.
  """

  H = np.zeros((10, 2))
  H[:, 0] = -1 * (x_hat - a) / np.sqrt((x_hat - a)**2 + (y_hat - b)**2)
  H[:, 1] = -1 * (y_hat - b) / np.sqrt((x_hat - a)**2 + (y_hat - b)**2)

  return H

def ekf_update(x_hat, P_hat, z, H):
  """Updates the EKF state estimate using the Kalman filter.

  Args:
    x_hat: The current state estimate.
    P_hat: The current covariance matrix.
    z: The measurement.
    a: The x-coordinate of the anchor point.
    b: The y-coordinate of the anchor point.
    H: The Jacobian matrix of the EKF measurement function.

  Returns:
    The updated state estimate and covariance matrix.
  """

  R = np.eye(10)
  #K = np.dot(P_hat, np.dot(H.T, np.linalg.inv(np.dot( np.dot(H, P_hat), H.T)  + R)))

  K = P_hat @ H.T @ np.linalg.inv( H @ P_hat @ H.T  + 0.01 * R )

  x_hat = x_hat + K @ (z - H @ x_hat )
  P_hat = (np.eye(x_hat.shape[0]) - K @ H) @ P_hat

  return x_hat, P_hat



import numpy as np

# Initialize the EKF
x_hat = np.array([0.0, 0.0])
P_hat = np.eye(2)

# Define the anchor point
#a = 10.0
#b = 10.0

a=[0.7984,0.9430,0.6837,0.1321,0.7227,0.1104,0.1175,0.6407,0.3288,0.6538]
b=[0.7491,0.5832,0.7400,0.2348,0.7350,0.9706,0.8669,0.0862,0.3664,0.3692]

bx_t=0.7
by_t=0.37

# Generate the (noisy) data y, and set initial guess
noise_level=0.0
z=np.empty(10)
for i in range(10):
    dx=bx_t-a[i]
    dy=by_t-b[i]
    z[i]=np.sqrt(dx*dx+dy*dy)+noise_level*np.random.random()

z =  z[:, np.newaxis]
x_hat = x_hat[:, np.newaxis]

array_point_x = []
array_point_y = []

for i in range(1):

  # Update the EKF state estimate
  H = ekf_measurement_linearization_10x2(x_hat[0][0], x_hat[1][0], a, b)
  x_hat, P_hat = ekf_update(x_hat, P_hat, z,  H)

  array_point_x.append(x_hat[0][0])
  array_point_y.append(x_hat[1][0])


x_hat = np.array([0.0, 0.0])[:, np.newaxis]

H = ekf_measurement_linearization_10x2(x_hat[0][0], x_hat[1][0], a, b)

X = np.linalg.inv( H.T @ H ) @ H.T 

print("============X============")
print(X)


# Compute the Hessian and add the damping term.
HTH = H.T @ H + 0.1 * np.eye(2)

# Solve for the update step.
delta_params = np.linalg.solve( HTH, -1 * H.T @ z)

print("============delta_params============")
print(delta_params)



# ------------------------------------ what we want to get, transmitter locations --------------------------------------- # 

# Define the transmitter's true location
bx_t=0.7
by_t=0.37

# -------------------------------------already know, beacon locations --------------------------------------- # 

# Define the beacon locations (randomly located in the unit square)

x=[0.7984,0.9430,0.6837,0.1321,0.7227,0.1104,0.1175,0.6407,0.3288,0.6538]
y=[0.7491,0.5832,0.7400,0.2348,0.7350,0.9706,0.8669,0.0862,0.3664,0.3692]


# -------------------------------------already know, distance of sensor --------------------------------------- # 

# Generate the (noisy) data y, and set initial guess
noise_level=0.05
d=np.empty(10)
for i in range(10):
    dx=bx_t-x[i]
    dy=by_t-y[i]
    d[i]=np.sqrt(dx*dx+dy*dy)+noise_level*np.random.random()


# Define the beacon locations (randomly located in the unit square)

params0 = np.array([0.0, 0.0])


# ------------------------------------- bard made LM, CORE--------------------------------------- # 


def distance_residuals(params, x, y):
  """
  Compute the residuals for the distance function.

  Args:
    params: A NumPy array containing the parameters of the distance function.
    x: A NumPy array containing the x-values of the data points.
    y: A NumPy array containing the y-values of the data points.

  Returns:
    A NumPy array containing the residuals.
  """

  a, b = params
  return  np.sqrt( (x - a)**2 + (y - b )**2 ) - d 

def distance_jacobian(params, x, y):
  """
  Compute the residuals for the distance function.

  Args:
    params: A NumPy array containing the parameters of the distance function.
    x: A NumPy array containing the x-values of the data points.
    y: A NumPy array containing the y-values of the data points.

  Returns:
    A NumPy array containing the residuals.
  """

  a, b = params
  bottom_factor =  np.sqrt( (x - a)**2 + (y - b )**2 ) 

  return [ ( a - x ) / bottom_factor, ( b - y ) / bottom_factor ] 


def non_linear_least_squares(f, params, data, max_iter=1000, tol=1e-3):
  """
  Solve a non-linear least squares problem.

  Args:
    f: A function that takes a NumPy array as input and returns a NumPy array
      of residuals.
    x0: A NumPy array containing the initial guess for the solution.
    args: A tuple of arguments to be passed to the function `f`.
    max_iter: The maximum number of iterations.
    tol: The tolerance for convergence.

  Returns:
    A NumPy array containing the least squares solution.
  """

  params = params.copy()
  lambda_ = 1e-3

  a=[0.7984,0.9430,0.6837,0.1321,0.7227,0.1104,0.1175,0.6407,0.3288,0.6538]
  b=[0.7491,0.5832,0.7400,0.2348,0.7350,0.9706,0.8669,0.0862,0.3664,0.3692]

  # Define the transmitter's true location
  bx_t=0.7
  by_t=0.37

  # Initialize the EKF
  x_hat = np.array([0.0, 0.0])
  P_hat = np.eye(2)

  # Generate the (noisy) data y, and set initial guess
  noise_level=0.0
  z=np.empty(10)
  for i in range(10):
      dx=bx_t-a[i]
      dy=by_t-b[i]
      z[i]=np.sqrt(dx*dx+dy*dy)+noise_level*np.random.random()

  z =  z[:, np.newaxis]
  x_hat = x_hat[:, np.newaxis]

  for i in range(1000):
    # Compute the residuals and Jacobian.
    r = f(params, data[0], data[1])
    r = np.array(r).T.reshape(10,1)
    J = np.zeros((len(r), len(params)))

    #print("=============distance_jacobian================")
    #jacobian = distance_jacobian(params, data[0], data[1])
    
    jacobian = ekf_measurement_linearization_10x2( params[0], params[1], data[0], data[1])

    #print( jacobian )

    #for j in range(len(params)):
    #  for i in range(10):
    #    J[i, j] = jacobian[j][i]

    J = -jacobian

    # Compute the Hessian and add the damping term.
    HTH = J.T @ J + lambda_ * np.eye(len(params))
    # Solve for the update step.
    delta_params = np.linalg.solve( HTH, -1 * J.T @ r)
    delta_params = delta_params.reshape(2,)
    # Update the solution.
    #params += 0.01 * delta_params

    print("============delta_params[1]=============")
    print(delta_params)

    array_point_x.append(params[0])
    array_point_y.append(params[1])

    H = -1*J
    R = np.eye(10)

    K = P_hat @ H.T @ np.linalg.inv( H @ P_hat @ H.T  + lambda_ * R )
    #P_hat = (np.eye(2) - K @ H) @ P_hat

    delta_params_by_K = K @ r

    print("============delta_params_by_K[1]=============")
    print(delta_params_by_K)


    params += 0.01 * np.array([delta_params_by_K[0][0],delta_params_by_K[1][0]])

    # Check for convergence.
    if np.linalg.norm(delta_params) < tol:
      print("===========DONE["+str(i)+"]=========")
      break

  return params

print("=============non_linear_least_squares================")
params = non_linear_least_squares(distance_residuals, params0, data=(x, y))

# Print the fitted parameters.
print(params)

'''
#========================key===========================#
r = f(params, data[0], data[1])

def distance_residuals(params, x, y):
  a, b = params
  return  np.sqrt( (x - a)**2 + (y - b )**2 ) - d 

z=np.empty(10)
for i in range(10):
    dx=bx_t-a[i]
    dy=by_t-b[i]
    z[i]=np.sqrt(dx*dx+dy*dy)+noise_level*np.random.random()


  R = np.eye(10)
  K = P_hat @ H.T @ np.linalg.inv( H @ P_hat @ H.T  + 0.01 * R )

  x_hat = x_hat + K @ (z - H @ x_hat )
  P_hat = (np.eye(x_hat.shape[0]) - K @ H) @ P_hat

'''