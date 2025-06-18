import matplotlib.pyplot as plt

# --- Parameters ---
Kt = 0.0036284605  # N-m/A
V_batt = 12       # Volts
P_max_system = 120 # Watts
I_max_motor = 15  # Amps
motor_max_angvel = 8000 * 0.10472 # Convert RPM to rad/s (0.10472 rad/s per RPM)
R_wheel = 2 * 0.0254 / 2 # meters (2 inches converted to meters divded by 2)
GR = 3.25           # Gear Reduction (Let's start with 3:1 again, then you can test 100:1)
d_target = 10     # meters
m_robot = 0.75    # kg
delta_t = 0.001  # Small time step (0.1 milliseconds for higher accuracy)


def simulate():
  # Compute constants
  max_vel = motor_max_angvel * R_wheel / GR  # m/s
  print(f"Max velocity: {max_vel:.2f} m/s")

  # Initialize
  t = 0.0
  d = 0.0
  v = 0.0
  t_out = [0.0]
  d_out = [0.0]
  v_out = [0.0]
  a_out = [None]

  # Accel phase
  while d < d_target/2 and v < max_vel:
    # Calculate current
    I = min(I_max_motor, P_max_system / max(V_batt * (v / max_vel), 1e-6))
    # Calculate torque
    T = Kt * I * 2
    # Calculate force
    F = T / R_wheel
    # Calculate acceleration
    a = F / m_robot
    # Update velocity and position
    v += a * delta_t
    d += v * delta_t
    # Update time
    t += delta_t
    # Store outputs
    t_out.append(t)
    d_out.append(d)
    v_out.append(v)
    a_out.append(a)

  # Coast phase
  if d < 5.0:
    accel_dist = d
    while d < d_target - accel_dist:
      # Update velocity and position
      d += v * delta_t
      # Update time
      t += delta_t
      # Store outputs
      t_out.append(t)
      d_out.append(d)
      v_out.append(v)
      a_out.append(0.0)
  
  # Decel phase
  while d <= d_target - 0.01:
    # Calculate current
    I = min(I_max_motor, P_max_system / max(V_batt * (v / max_vel), 1e-6))
    # Calculate torque
    T = Kt * I * 2
    # Calculate force
    F = T / R_wheel
    # Calculate acceleration (negative for deceleration)
    a = -F / m_robot
    # Update velocity and position
    v += a * delta_t
    d += v * delta_t
    # Update time
    t += delta_t
    # Store outputs
    t_out.append(t)
    d_out.append(d)
    v_out.append(v)
    a_out.append(a)
  
  return (t_out, d_out, v_out, a_out)


# Simulate and plot results
t_out, d_out, v_out, a_out = simulate()
print(f"Reached max velocity: {max(v_out):.2f} m/s")
print(f"Final time: {t_out[-1]:.2f} seconds")
fig = plt.figure(figsize=(12, 8))
fig.canvas.manager.set_window_title('Robot Motion Simulation Results')
plt.subplot(3, 1, 1)
plt.plot(t_out, d_out, label='Distance (m)')
plt.title('Distance vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.grid()
plt.subplot(3, 1, 2)
plt.plot(t_out, v_out, label='Velocity (m/s)', color='orange')
plt.title('Velocity vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.grid()
plt.subplot(3, 1, 3)
plt.plot(t_out, a_out, label='Acceleration (m/s²)', color='green')
plt.title('Acceleration vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s²)')
plt.grid()
plt.tight_layout()
plt.show()
