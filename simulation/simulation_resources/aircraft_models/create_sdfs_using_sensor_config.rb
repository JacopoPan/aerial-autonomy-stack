require 'yaml'
require 'erb'

# This script generates the sensor SDFs and the SDFs of the vehicles given as arguments (all if none) based on the content of sensor_config.yaml

MODELS_DIR = '/aas/simulation_resources/aircraft_models'

# Helper method to render ERB templates and save the output
def render_and_save(template_name, output_name, template_binding)
  template_path = File.join(MODELS_DIR, template_name)
  output_path   = File.join(MODELS_DIR, output_name)
  if File.exist?(template_path)
    rendered_content = ERB.new(File.read(template_path)).result(template_binding)
    File.write(output_path, rendered_content)
    puts "Generated #{output_path}"
  else
    puts "Warning: #{template_path} not found"
  end
end

# Load the configuration file
config = YAML.load_file(File.join(MODELS_DIR, 'sensor_config.yaml'))

# Generate sensor SDFs
camera_intrinsics = config['sensors']['camera_intrinsics']
render_and_save('sensor_camera/model.sdf.erb', 'sensor_camera/model.sdf', binding)

lidar_intrinsics = config['sensors']['lidar_intrinsics']
render_and_save('sensor_lidar/model.sdf.erb', 'sensor_lidar/model.sdf', binding)

# Generate aircraft SDFs
config['aircraft_models'].each do |model_name, extrinsics|
  # Skip the vehicles not given as arguments (simulation.yml.erb passes the ones in use), without arguments generate them all
  next unless ARGV.empty? || ARGV.include?(model_name)
  camera_ext = extrinsics['camera_extrinsics']
  lidar_ext  = extrinsics['lidar_extrinsics']
  
  render_and_save("#{model_name}/model.sdf.erb", "#{model_name}/model.sdf", binding)
end
