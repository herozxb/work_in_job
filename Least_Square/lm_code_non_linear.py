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

def residual_phi(x):
    residual = []
    for i in range(10):
        dx=x[0]-x_beac[i]
        dy=x[1]-y_beac[i]
        ss=np.sqrt( dx**2 + dy**2 )-d[i]
        residual.append(ss)
    return residual

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
  bottom_factor = ( np.sqrt( (x - a)**2 + (y - b )**2 ) )**1

  return [ ( a - x ) / bottom_factor, ( b - y ) / bottom_factor ] 

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
    
    #print("=============jacobian================")
    #print(J)

    #print("=============distance_jacobian================")
    #jacobian = distance_jacobian(params, data[0], data[1])
    #print( jacobian )

    #for j in range(len(params)):
    #  for i in range(10):
    #    J[i, j] = jacobian[j][i]


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

print("=============self.least_squares================")
params = non_linear_least_squares(distance_residuals, params0, data=(x, y))

# Print the fitted parameters.
print(params)



# Plot results - create contours of phi function
n=100
xx=np.linspace(0,1,n)
yy=np.linspace(0,1,n)
X,Y=np.meshgrid(xx,yy)
pxy=np.zeros((n,n))
for i in range(n):
    for j in range(n):
        pxy[i,j]=phi([X[i,j],Y[i,j]])
plt.contourf(X,Y,pxy,16,alpha=.75)
C = plt.contour(X,Y,pxy,16,colors='black')

# Plot results show beacon positions and true/predicted transmitter location
plt.xlabel('x')
plt.ylabel('y')
plt.plot(x_beac,y_beac,'o',color='red')
plt.plot(array_point_x,array_point_y,'o',color='blue')
plt.plot(bx_t,by_t,'x',color='black')
plt.show()