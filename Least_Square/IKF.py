#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt


# ------------------------------------ what we want to get, transmitter locations --------------------------------------- # 

# Define the transmitter's true location
bx_t=0.7
by_t=0.37

# -------------------------------------already know, beacon locations --------------------------------------- # 

# Define the beacon locations (randomly located in the unit square)
x_beac=[0.7984,0.9430,0.6837,0.1321,0.7227,0.1104,0.1175,0.6407,0.3288,0.6538]
y_beac=[0.7491,0.5832,0.7400,0.2348,0.7350,0.9706,0.8669,0.0862,0.3664,0.3692]

# -------------------------------------already know, distance of sensor --------------------------------------- # 

# Generate the (noisy) data y, and set initial guess
noise_level=0.05
d=np.empty(10)
for i in range(10):
    dx=bx_t-x_beac[i]
    dy=by_t-y_beac[i]
    d[i]=np.sqrt(dx*dx+dy*dy)+noise_level*np.random.random()
    d[i]=0.6
b_init=np.array([0.4,0.9])

# ------------------------------------- the contour to be minimized, CORE--------------------------------------- # 

# The function, phi, to be minimized
def phi(x):
    s=0
    for i in range(10):
        dx=x[0]-x_beac[i]
        dy=x[1]-y_beac[i]
        ss=np.sqrt(dx*dx+dy*dy)-d[i]
        s+=ss*ss
    return s


# Gradient of phi
def grad_phi(x):
    f0=0
    f1=0
    for i in range(10):
        dx=x[0]-x_beac[i]
        dy=x[1]-y_beac[i]
        d=1/np.sqrt(dx*dx+dy*dy)
        f0+=2*dx-2*y[i]*dx*d
        f1+=2*dy-2*y[i]*dy*d
    return np.array([f0,f1])




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
  return ( np.sqrt( (x - a)**2 + (y - b )**2 ) - d )**1

def non_linear_least_squares(f, params, data, max_iter=1000, tol=1e-2):
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
  for i in range(max_iter):
    # Compute the residuals and Jacobian.
    print("========non_linear_least_squares========")
    print(params)
    print(data)
    r = f(params, data[0], data[1])
    print("========1========")
    print(r.reshape(1,10))
    print(np.array(r).T.reshape(10,1))
    r = np.array(r).T.reshape(10,1)
    J = np.zeros((len(r), len(params)))
    print("========2========")
    print(J)
    print(len(params))
    for j in range(len(params)):
      delta = np.zeros(len(params))
      delta[j] = 1e-6
      print(delta)
      
      df = np.array(f(params + delta, data[0], data[1] )).T.reshape(10,1)
      print("===========df============")
      print(df)
      print(df.shape)

      df_dx = ( df - r) / 1e-6

      for i in range(10):
        J[i, j] = df_dx[i]
      print(J)
      print(J.shape)

    # J is right
    print("========3========")
    print(J)      
    print(J.shape)    
    print(J.T)  
    print(J.T.shape)  

    # Compute the Hessian and add the damping term.
    H = J.T @ J + lambda_ * np.eye(len(params))
    print(H)

    # Solve for the update step.
    delta_params = np.linalg.solve( H, -1 * J.T @ r)

    print(delta_params)
    print(delta_params.shape)
    print(params)
    print(params.shape)


    delta_params = delta_params.reshape(2,)
    # Update the solution.
    params += 0.01 * delta_params

    print("========result_params========")
    print(params)
    print(params.shape)
    print(delta_params)
    print(delta_params.shape)

    # Check for convergence.
    if np.linalg.norm(delta_params) < tol:
      print("===========DONE=========["+str(i)+"]")
      break



  return params




x = np.linspace(0, 10, 10)
y = 2 * x + 0 + np.random.randn(10)

# Define the beacon locations (randomly located in the unit square)
x=[0.7984,0.9430,0.6837,0.1321,0.7227,0.1104,0.1175,0.6407,0.3288,0.6538]
y=[0.7491,0.5832,0.7400,0.2348,0.7350,0.9706,0.8669,0.0862,0.3664,0.3692]

print("=============data================")
print(x)
print(y)

params0 = np.array([0.0, 1.0])

from scipy.optimize import least_squares

res = least_squares(distance_residuals, params0, args=(x, y))

print("=============scipy.least_squares================")
print(res.x)



#print("=============distance_residuals==============")
#print(distance_residuals(params0, x, y))


#params = non_linear_least_squares(distance_residuals, params0, data=(x, y))

# Print the fitted parameters.
#print(params)

array_point_x = []
array_point_y = []
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

  for i in range(max_iter):
    # Compute the residuals and Jacobian.
    r = f(params, data[0], data[1])
    r = np.array(r).T.reshape(10,1)
    J = np.zeros((len(r), len(params)))

    for j in range(len(params)):

      delta = np.zeros(len(params))
      delta[j] = 1e-6

      df = np.array(f(params + delta, data[0], data[1] )).T.reshape(10,1)
      df_dx = ( df - r) / 1e-6

      for i in range(10):
        J[i, j] = df_dx[i]

    # Compute the Hessian and add the damping term.
    H = J.T @ J + lambda_ * np.eye(len(params))
    # Solve for the update step.
    delta_params = np.linalg.solve( H, -1 * J.T @ r)
    delta_params = delta_params.reshape(2,)
    # Update the solution.
    params += 0.01 * delta_params

    array_point_x.append(params[0])
    array_point_y.append(params[1])

    # Check for convergence.
    if np.linalg.norm(delta_params) < tol:
      print("===========DONE["+str(i)+"]=========")
      break

  return params

#print("=============self.least_squares================")
#params = non_linear_least_squares(distance_residuals, params0, data=(x, y))

# Print the fitted parameters.
#print(params)


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

  a, b = params[0][0], params[1][0]
  bottom_factor = ( np.sqrt( (x - a)**2 + (y - b )**2 ) )**1

  return [ ( a - x ) / bottom_factor, ( b - y ) / bottom_factor ] 

def iterate_kalman_filter(A, B, H, Q, R, params_init, P_init, d):
  """
  Iterates the Kalman filter to solve a least squares problem.

  Args:
    A: The state transition matrix.
    B: The control input matrix.
    C: The observation matrix.
    Q: The process noise covariance matrix.
    R: The measurement noise covariance matrix.
    x_init: The initial state estimate.
    P_init: The initial state covariance matrix.
    y: The measurement vector.

  Returns:
    x_hat: The estimated state vector.
    P_hat: The estimated state covariance matrix.
  """

  x_hat = params_init
  P_hat = P_init

  #print("==================params=================")
  #print(params_init.shape)
  # Prediction step
  x_pred = A @ x_hat #+ B * np.zeros(B.shape[1])
  P_pred = A @ P_hat @ A.T + Q

  #print(x_pred.shape)

  #print(C.shape)
  #print(C.T.shape)

  J = H.T
  #print(P_pred.shape)
  #print(J.shape)
  #print("================J===================")
  #print(P_pred.shape)
  #print(J.T.shape)
  #print(J @ P_pred @ J.T)
  #print(R*np.eye(10))
  #print(J)

  # Update step
  #K = P_pred * C.T * np.linalg.inv(C * P_pred * C.T + R)
  #x_hat = x_pred + K * (y[i] - C * x_pred)
  #P_hat = (np.eye(P_hat.shape[0]) - K * C) * P_pred

  K = P_pred @ J.T @ np.linalg.inv(J @ P_pred @ J.T + R*np.eye(10))

  print("================K==================")
  #print(K)
  print(K.shape)
  #print(J @ x_pred)
  #print(J.shape )
  print(x_pred.shape)
  #x_hat = x_pred + K @ ( d - ( np.sqrt( ( x_beac - x_pred[0][0] )**2 + ( y_beac - x_pred[1][0] )**2 ) )**1 )
  x_hat = x_pred + K @ (d.reshape(10,1) - J @ x_pred)
  P_hat = (np.eye(P_hat.shape[0]) - K @ J) @ P_pred

  print("===================x==================")
  print( d.reshape(10,1).shape )
  print( (J @ x_pred).shape )
  print( (d - J @ x_pred).shape )
  print(x_hat.shape)
  print(x_hat)
  print(P_hat)

  return x_hat, P_hat


'''
  for i in range(len(d)):
    # Prediction step
    x_pred = A * x_hat #+ B * np.zeros(B.shape[1])
    P_pred = A * P_hat * A.T + Q

    #print(C.shape)
    #print(C.T.shape)
    
    J = []
    J.append([C.T[i][0]])
    J.append([C.T[i][1]])
    #J = C.T[i]
    J = np.array(J)
    #print(J.shape)
    #print("================J===================")
    #print(J)

    # Update step
    #K = P_pred * C.T * np.linalg.inv(C * P_pred * C.T + R)
    #x_hat = x_pred + K * (y[i] - C * x_pred)
    #P_hat = (np.eye(P_hat.shape[0]) - K * C) * P_pred

    K = P_pred * J.T * np.linalg.inv(J * P_pred * J.T + R)
    x_hat = x_pred + K * (d[i] - J * x_pred)
    P_hat = (np.eye(P_hat.shape[0]) - K * J) * P_pred
'''


def least_squares_kalman_filter(x, y):
  """
  Solves a least squares problem using the Kalman filter.

  Args:
    x: The input data.
    y: The output data.

  Returns:
    a: The estimated slope.
    b: The estimated intercept.
  """

  # Define the state transition matrix, control input matrix, observation matrix, process noise covariance matrix, and measurement noise covariance matrix.
  A = np.array([[1.0, 0.0], [0.0, 1.0]])
  B = np.array([[0.0], [0.0]])
  #C = np.array([[0.5, 0.5]])
  Q = np.array([[0.0, 0.0]])
  R = np.array([[0.1]])

  # Initialize the Kalman filter.
  params_init = np.array([[0.0], [0.0]])
  P_init = np.array([[1.0, 0.0], [0.0, 1.0]])

  # least square parameters

  for i in range(1000):

    H = np.array( distance_jacobian(params_init,x,y) )
    #y = d

    # Iterate the Kalman filter.
    params_hat, P_hat = iterate_kalman_filter(A, B, H, Q, R, params_init, P_init, d)

    params_init += 0.01 * np.array([[params_hat[0][0]], [params_hat[1][0]]])
    #P_init = P_hat


  # The estimated slope and intercept are the first two elements of the state estimate.
  a = params_hat[0][0]
  b = params_hat[1][0]

  return a, b

# Example usage:

x = np.linspace(0, 10, 10)
y = 2 * x + 0 + np.random.randn(10)

print("=============data================")
print(x)
print(y)

x = x_beac
y = y_beac

a, b = least_squares_kalman_filter(x, y)

print("===================least_squares_kalman_filter=======================")
print(a)
print(b)


'''
import numpy as np
import matplotlib.pyplot as plt

class IKF:
    def __init__(self, x0, P0, Q, R, H):
        self.x = x0
        self.P = P0
        self.Q = Q
        self.R = R
        self.H = H

    def predict(self):
        self.x = self.x + self.Q

    def update(self, z):
        K = np.dot(self.P, np.dot(self.H, np.linalg.inv(np.dot(self.H, self.P) + self.R)))
        self.x = self.x + K * (z - np.dot(self.H, self.x))
        self.P = (np.eye(self.x.shape[0]) - K * self.H) * self.P

    def get_state(self):
        return self.x

def f(x):
    return x**2 + 1

def h(x):
    return x

def generate_data(x, noise_level=0.1):
    z = f(x) + np.random.randn() * noise_level
    return z

# Initialize the filter
ikf = IKF(np.array([0]), np.eye(1), np.eye(1), np.eye(1),np.array([[1]]))

# Generate data
data = []
for i in range(100):
    x = i / 10
    z = generate_data(x)
    data.append((x, z))

# Solve the non-linear least square problem
for x, z in data:
    ikf.predict()
    ikf.update(z)

# Get the estimated state
estimated_state = ikf.get_state()

# Plot the results
plt.plot(data, 'o')
plt.plot(estimated_state, 'r-')
plt.legend(['Data', 'Estimated state'])
plt.show()

'''