# Helpers for aircraft.yml.erb: sensor_config.yaml's poses are "x y z roll pitch yaw" (in m and deg, fixed axes, as in Gazebo and tf2)
module PoseHelpers
  def quat(roll, pitch, yaw) # Quaternion [x, y, z, w] of Rz(yaw) * Ry(pitch) * Rx(roll)
    cr, sr, cp, sp, cy, sy = [roll, pitch, yaw].flat_map { |deg| [Math.cos(deg * Math::PI / 360), Math.sin(deg * Math::PI / 360)] }
    [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]
  end

  def quat_mul((ax, ay, az, aw), (bx, by, bz, bw))
    [aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz]
  end

  def rot_matrix((x, y, z, w)) # Row-major
    [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
  end

  def transform_matrix(position, quaternion) # Homogeneous, row-major
    rot_matrix(quaternion).zip(position).map { |row, p| row + [p] } + [[0, 0, 0, 1]]
  end

  def mat_vec(matrix, vector) # Matrix-vector product
    matrix.map { |row| row.zip(vector).sum { |a, b| a * b } }
  end

  def transform_point((x, y, z, roll, pitch, yaw), point) # From the frame at "x y z roll pitch yaw" to its parent frame
    mat_vec(transform_matrix([x, y, z], quat(roll, pitch, yaw)), point + [1]).first(3)
  end

  def tf_args(position, quaternion) # As static_transform_publisher's arguments
    %w[--x --y --z --qx --qy --qz --qw].zip((position + quaternion).map { |v| v.round(6) }).map { |flag, value| "#{flag} #{value}" }.join(' ')
  end

  def float_list(values) # As a ROS parameter array
    "[#{values.flatten.map { |v| v.to_f.round(6) }.join(', ')}]"
  end
end
