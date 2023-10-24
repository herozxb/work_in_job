#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt


def ekf_measurement(x, y, a, b):
  return np.sqrt((x - a)**2 + (y - b)**2)

def ekf_measurement_linearization_2x2(x_hat, y_hat, a, b):
  """Linearizes the EKF measurement function around the current state estimate.

  Args:
    x_hat: The current state estimate.
    y_hat: The current state estimate.
    a: The x-coordinate of the anchor point.
    b: The y-coordinate of the anchor point.

  Returns:
    A 2x2 Jacobian matrix.
  """

  H = np.zeros((2, 2))
  #print("===========")
  #print(x_hat)
  #print(y_hat)
  #print(a)
  #print(b)
  #print( np.sqrt((x_hat - a)**2 + (y_hat - b)**2) )
  H[0, 0] = (x_hat - a) / np.sqrt((x_hat - a)**2 + (y_hat - b)**2)
  H[0, 1] = (y_hat - b) / np.sqrt((x_hat - a)**2 + (y_hat - b)**2)
  H[1, 0] = -H[0, 1]
  H[1, 1] = H[0, 0]

  return H

def ekf_measurement_linearization_1x2(x_hat, y_hat, a, b):
  """Linearizes the EKF measurement function around the current state estimate.

  Args:
    x_hat: The current state estimate.
    y_hat: The current state estimate.
    a: The x-coordinate of the anchor point.
    b: The y-coordinate of the anchor point.

  Returns:
    A 2x2 Jacobian matrix.
  """

  H = np.zeros((1, 2))
  #print("===========")
  #print(x_hat)
  #print(y_hat)
  #print(a)
  #print(b)
  #print( np.sqrt((x_hat - a)**2 + (y_hat - b)**2) )
  H[0, 0] = -1 * (x_hat - a) / np.sqrt((x_hat - a)**2 + (y_hat - b)**2)
  H[0, 1] = -1 * (y_hat - b) / np.sqrt((x_hat - a)**2 + (y_hat - b)**2)

  #print("===========H============")
  #print(H)
  return H

def ekf_update(x_hat, P_hat, z, a, b, H):
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

  R = np.array([0.1])
  #K = np.dot(P_hat, np.dot(H.T, np.linalg.inv(np.dot( np.dot(H, P_hat), H.T)  + R)))

  K = P_hat @ H.T @ np.linalg.inv( H @ P_hat @ H.T  + R )

  #print("=======ekf_update======")

  #x_hat = x_hat.T
  #print(x_hat)
  #print(K)
  #print(z)
  #print(H)
  #print(np.dot(H, x_hat))

  x_hat = x_hat + K * (z - np.dot(H, x_hat))
  P_hat = (np.eye(x_hat.shape[0]) - K * H) * P_hat


  return x_hat, P_hat



import numpy as np

# Initialize the EKF
x_hat = np.array([0.0, 0.0])
P_hat = np.eye(2)

# Define the anchor point
a = 10.0
b = 10.0

# Simulate the robot's motion
true_x = np.array([0.0, 0.0])

x_hat = x_hat[:, np.newaxis]
true_x = true_x[:, np.newaxis]

#print(x_hat)
#print(true_x)

array_point_x = []
array_point_y = []

array_point_true_x = []
array_point_true_y = []

for i in range(100):
  # Simulate the robot's motion
  true_x += np.array([0.1, 0.1])[:, np.newaxis]
  #print("=======vector========")
  #print(true_x)
  #print(true_x[1][0])

  array_point_true_x.append(true_x[0][0])
  array_point_true_y.append(true_x[1][0])

  # Generate a noisy distance measurement
  z = np.sqrt((true_x[0][0] - a)**2 + (true_x[1][0] - b)**2) + 5 * np.random.randn()

  # Update the EKF state estimate
  H = ekf_measurement_linearization_1x2(x_hat[0][0], x_hat[1][0], a, b)
  x_hat, P_hat = ekf_update(x_hat, P_hat, z, a, b, H)
  #print("========x_hat========")
  #print(x_hat)
  #print(P_hat)

  array_point_x.append(x_hat[0][0])
  array_point_y.append(x_hat[1][0])



# Plot the results
plt.plot(array_point_true_x, array_point_true_y, 'b-')
plt.plot(array_point_x, array_point_y, 'r-')
plt.legend(['True position', 'Estimated position'])
plt.show()