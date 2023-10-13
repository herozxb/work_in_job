#!/usr/bin/python3
import numpy as np

# ------------------------------------- bard made LM, CORE--------------------------------------- # 

def linear_least_squares(A, b):
  """
  Solve a linear least squares problem.

  Args:
    A: A NumPy array containing the design matrix.
    b: A NumPy array containing the observations.

  Returns:
    A NumPy array containing the least squares solution.
  """

  # Compute the pseudoinverse of the design matrix.
  A_pinv = np.linalg.inv(A.T @ A) @ A.T

  # Compute the least squares solution.
  x = A_pinv @ b

  return x

# Generate the data.
x = np.linspace(0, 10, 10)
y = 2 * x + 3 + np.random.randn(10)

print("=============linear_least_square================")
print(x)
print(y)

# Define the design matrix.
A = np.array([x, np.ones(10)]).T

# Solve the linear least squares problem.
b = linear_least_squares(A, y)
print("=============A================")
print(A)

# Print the fitted parameters.
print(b)


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
  return np.sqrt( (x - a)**6 + (y - b * 2 )**2 )

def non_linear_least_squares(f, params, data, max_iter=500, tol=1e-1):
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

print("=============data================")
print(x)
print(y)

params0 = np.array([0.0, 0.0])

from scipy.optimize import least_squares

res = least_squares(distance_residuals, params0, args=(x, y))

print("=============scipy.least_squares================")
print(res.x)



#print("=============distance_residuals==============")
#print(distance_residuals(params0, x, y))


#params = non_linear_least_squares(distance_residuals, params0, data=(x, y))

# Print the fitted parameters.
#print(params)


def non_linear_least_squares(f, params, data, max_iter=500, tol=1e-1):
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

    # Check for convergence.
    if np.linalg.norm(delta_params) < tol:
      print("===========DONE["+str(i)+"]=========")
      break

  return params

print("=============self.least_squares================")
params = non_linear_least_squares(distance_residuals, params0, data=(x, y))

# Print the fitted parameters.
print(params)



