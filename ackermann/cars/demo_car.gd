extends Node3D
@onready var pose: Label = $"../Pose"

@export var terrain_probe_distance := 2.0
@export var terrain_probe_height := 50.0
@export var terrain_probe_depth := 100.0

var heading := 0.0
var terrain_reference_height = null

func _ready() -> void:
	DDS.subscribe("X")
	DDS.subscribe("Y")
	DDS.subscribe("Z")
	DDS.subscribe("Theta")
	DDS.subscribe("Slope")
	DDS.subscribe("PayloadMass")
	DDS.subscribe("CargoPhase")

func _physics_process(_delta: float) -> void:
	var probe_position := Vector3(global_position.x, 0.0, global_position.z)
	var forward := Vector3(-sin(heading), 0.0, -cos(heading))
	var terrain_height = _surface_height(probe_position)
	var ahead_height = _surface_height(probe_position + forward * terrain_probe_distance)

	if terrain_height == null:
		return

	if terrain_reference_height == null:
		terrain_reference_height = terrain_height

	var relative_height: float = terrain_height - terrain_reference_height
	var terrain_slope := 0.0
	if ahead_height != null:
		terrain_slope = atan2(ahead_height - terrain_height, terrain_probe_distance)

	DDS.publish("TerrainHeight", DDS.DDS_TYPE_FLOAT, relative_height)
	DDS.publish("TerrainSlope", DDS.DDS_TYPE_FLOAT, terrain_slope)

func _process(delta: float) -> void:
	#print(theRobot.global_position.x, " ", -theRobot.global_position.z, " ", theRobot.global_rotation.y)
	DDS.publish("tick", DDS.DDS_TYPE_FLOAT, delta)

	var x = DDS.read("X")
	var y = DDS.read("Y")
	var z = DDS.read("Z")
	var theta = DDS.read("Theta")
	var slope = DDS.read("Slope")
	var payload_mass = DDS.read("PayloadMass")
	var cargo_phase = DDS.read("CargoPhase")

	if (x != null)and(y != null)and(theta != null):
		heading = theta
		self.global_position.x = -y
		self.global_position.z = -x
		if z != null:
			self.global_position.y = z
		if slope != null:
			self.global_rotation = Vector3(slope, theta, 0.0)
		else:
			self.global_rotation.y = theta

		var height = z if z != null else 0.0
		var incline = slope if slope != null else 0.0
		var payload = payload_mass if payload_mass != null else 0.0
		var cargo_label := "verso A"
		if cargo_phase != null and cargo_phase >= 1.5:
			cargo_label = "scaricato in B"
		elif cargo_phase != null and cargo_phase >= 0.5:
			cargo_label = "in trasporto"
		pose.text = "X: %.3f, Y: %.3f, Z: %.3f\nTheta: %.0f, Slope: %.1f, Payload: %.1f kg\nCargo: %s" % \
			[x, y, height, rad_to_deg(theta), rad_to_deg(incline), payload, cargo_label]

func _surface_height(horizontal_position: Vector3):
	var query := PhysicsRayQueryParameters3D.create(
		horizontal_position + Vector3.UP * terrain_probe_height,
		horizontal_position - Vector3.UP * terrain_probe_depth
	)
	var collision := get_world_3d().direct_space_state.intersect_ray(query)
	if collision.is_empty():
		return null
	return collision["position"].y
