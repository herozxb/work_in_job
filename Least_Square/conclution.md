# 1 LM algorithm

H = J.T @ J + lambda_ * np.eye(len(2))
delta_params = np.linalg.solve( H, -1 * J.T @ r)

delta_params = np.linalg.inv( J.T @ J + lambda_ * np.eye(len(2)) ) @ J.T @ r

# 2 kalman filter, K

H = -J

K = P_hat @ H.T @ np.linalg.inv( H @ P_hat @ H.T  + lambda_ * R )

delta_params_by_K = K @ r

# the LM is the same as the K, when it comes to Least Square
# but the kalman filter K is for the recursive Least Square
# Kalman filter K is higher level of recursive Least Square

# 3 key of Least Square is the residual(e), jacobian of residual(de/dx)

e = residual

J = e.T * e
dJ/dx = 2 de/dx * e

x = x - 0.01 * dJ/dx

# 4 if kalman filter K is not recursive, then K is degenerative to common LM algorithm